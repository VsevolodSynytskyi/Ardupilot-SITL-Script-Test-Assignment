#include "altctl/drone_interface.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>

#include <mavsdk/log_callback.hpp>
#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/mavlink_direct/mavlink_direct.hpp>
#include <mavsdk/plugins/param/param.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>

#include "altctl/units.hpp"

namespace altctl {

namespace {

using Clock = DroneInterface::Clock;

constexpr double kFastTelemetryRateHz = 50.0;
constexpr uint8_t kAutopilotComponentId = 1;  // MAV_COMP_ID_AUTOPILOT1
constexpr int kMavCmdDoSetMode = 176;
constexpr int kMavModeFlagCustomModeEnabled = 1;
// SET_ATTITUDE_TARGET: ignore body roll/pitch/yaw rates (ArduPilot needs all three or none)
constexpr int kTypeMaskIgnoreBodyRates = 1 | 2 | 4;
constexpr auto kModeCommandResendInterval = std::chrono::seconds(1);
constexpr auto kPollInterval = std::chrono::milliseconds(20);
constexpr auto kReadyPollInterval = std::chrono::milliseconds(500);
constexpr auto kFirstHeartbeatTimeout = std::chrono::milliseconds(3000);

std::atomic<bool> g_connected{false};

// Minimal extraction of a numeric field from MavlinkDirect's flat JSON object.
std::optional<double> extract_json_number(const std::string& json, const std::string& key)
{
    const auto key_position = json.find("\"" + key + "\"");
    if (key_position == std::string::npos) {
        return std::nullopt;
    }
    const auto colon = json.find(':', key_position);
    if (colon == std::string::npos) {
        return std::nullopt;
    }
    try {
        return std::stod(json.substr(colon + 1));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

// Console filter for MAVSDK's own logging: autopilot STATUSTEXT messages become "[ap]" lines,
// MAVSDK debug/info chatter is dropped, warnings and errors are kept.
bool filter_mavsdk_log(mavsdk::log::Level level, const std::string& message, const std::string&,
                       int)
{
    constexpr const char* kStatusTextPrefix = "MAVLink: ";
    if (message.rfind(kStatusTextPrefix, 0) == 0) {
        std::printf("[ap] %s\n", message.c_str() + std::strlen(kStatusTextPrefix));
        std::fflush(stdout);
        return true;
    }
    if (level == mavsdk::log::Level::Debug || level == mavsdk::log::Level::Info) {
        return true;
    }
    // DO_SET_MODE is sent through MavlinkDirect, so MAVSDK's command sender does not know the ack.
    if (message.find("ack for not-existing command") != std::string::npos) {
        return true;
    }
    // MAVSDK sends a heartbeat before the UDP peer is known; harmless until connected.
    if (!g_connected && message.find("Sending message failed") != std::string::npos) {
        return true;
    }
    std::fprintf(stderr, "[mavsdk] %s\n", message.c_str());
    return true;
}

}  // namespace

DroneInterface::DroneInterface(std::string connection_url)
    : connection_url_(std::move(connection_url))
{
    mavsdk::log::subscribe(filter_mavsdk_log);
}

DroneInterface::~DroneInterface()
{
    // Plugins (and their subscriptions) first, while state_mutex_/state_ that the callbacks use
    // are still alive: members are destroyed in reverse declaration order, which would
    // destroy the mutex before the plugins and crash a late callback (SIGABRT on exit).
    mavlink_direct_.reset();
    param_.reset();
    action_.reset();
    telemetry_.reset();
    system_.reset();
    mavsdk_.reset();
}

bool DroneInterface::connect(std::chrono::seconds timeout)
{
    mavsdk_ = std::make_unique<mavsdk::Mavsdk>(
        mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::CompanionComputer});
    if (const auto result = mavsdk_->add_any_connection(connection_url_);
        result != mavsdk::ConnectionResult::Success) {
        std::cerr << "[drone] connection " << connection_url_ << " failed: " << result << "\n";
        return false;
    }
    auto autopilot = mavsdk_->first_autopilot(static_cast<double>(timeout.count()));
    if (!autopilot) {
        std::cerr << "[drone] no autopilot heartbeat on " << connection_url_ << " within "
                  << timeout.count() << " s\n";
        return false;
    }
    system_ = *autopilot;
    g_connected = true;
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    action_ = std::make_unique<mavsdk::Action>(system_);
    param_ = std::make_unique<mavsdk::Param>(system_);
    mavlink_direct_ = std::make_unique<mavsdk::MavlinkDirect>(system_);

    request_telemetry_streams();
    subscribe_telemetry();
    wait_for_first_heartbeat(kFirstHeartbeatTimeout);
    std::cout << "[drone] connected to system " << static_cast<int>(system_->get_system_id())
              << " via " << connection_url_ << "\n";
    return true;
}

// Request every stream we depend on (MAV_CMD_SET_MESSAGE_INTERVAL under the hood). A SITL
// SERIAL port other than SERIAL0 has all default stream rates at 0, so nothing is implied.
void DroneInterface::request_telemetry_streams()
{
    struct StreamRequest {
        const char* message;
        mavsdk::Telemetry::Result (mavsdk::Telemetry::*set_rate)(double) const;
        double rate_hz;
    };
    const StreamRequest requests[] = {
        {"LOCAL_POSITION_NED", &mavsdk::Telemetry::set_rate_position_velocity_ned,
         kFastTelemetryRateHz},
        {"ATTITUDE", &mavsdk::Telemetry::set_rate_attitude_euler, kFastTelemetryRateHz},
        {"EXTENDED_SYS_STATE", &mavsdk::Telemetry::set_rate_in_air, 10.0},
        {"SYS_STATUS (health)", &mavsdk::Telemetry::set_rate_health, 2.0},
        {"GLOBAL_POSITION_INT", &mavsdk::Telemetry::set_rate_position, 5.0},
        {"GPS_RAW_INT", &mavsdk::Telemetry::set_rate_gps_info, 2.0},
        {"HOME_POSITION", &mavsdk::Telemetry::set_rate_home, 1.0},
    };
    for (const auto& request : requests) {
        const auto result = ((*telemetry_).*(request.set_rate))(request.rate_hz);
        if (result != mavsdk::Telemetry::Result::Success) {
            std::cerr << "[drone] warning: rate request " << request.message << ": " << result
                      << "\n";
        }
    }
}

void DroneInterface::subscribe_telemetry()
{
    telemetry_->subscribe_position_velocity_ned(
        [this](mavsdk::Telemetry::PositionVelocityNed position_velocity) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.alt_m = altitude_from_ned(position_velocity.position.down_m);
            state_.climb_mps = climb_from_ned(position_velocity.velocity.down_m_s);
            state_.last_position_update = Clock::now();
            ++state_.position_updates;
        });
    telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle attitude) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.yaw_rad = degrees_to_radians(attitude.yaw_deg);
    });
    telemetry_->subscribe_armed([this](bool armed) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.armed = armed;
    });
    telemetry_->subscribe_in_air([this](bool in_air) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.in_air = in_air;
    });
    const auto autopilot_system_id = system_->get_system_id();
    mavlink_direct_->subscribe_message(
        "HEARTBEAT", [this, autopilot_system_id](mavsdk::MavlinkDirect::MavlinkMessage heartbeat) {
            if (heartbeat.system_id != autopilot_system_id ||
                heartbeat.component_id != kAutopilotComponentId) {
                return;
            }
            if (const auto custom_mode = extract_json_number(heartbeat.fields_json, "custom_mode")) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                state_.custom_mode = static_cast<uint32_t>(*custom_mode);
                state_.last_heartbeat = Clock::now();
            }
        });

    // Seed the snapshot so the first state() after connect() is meaningful.
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.armed = telemetry_->armed();
    state_.in_air = telemetry_->in_air();
}

void DroneInterface::wait_for_first_heartbeat(std::chrono::milliseconds timeout) const
{
    const auto deadline = Clock::now() + timeout;
    while (state().last_heartbeat == Clock::time_point{} && Clock::now() < deadline) {
        std::this_thread::sleep_for(kPollInterval);
    }
}

DroneInterface::State DroneInterface::state() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

bool DroneInterface::set_param_int(const std::string& name, int32_t value)
{
    const auto result = param_->set_param_int(name, value);
    if (result != mavsdk::Param::Result::Success) {
        std::cerr << "[drone] set " << name << "=" << value << " failed: " << result << "\n";
        return false;
    }
    return true;
}

std::optional<int32_t> DroneInterface::get_param_int(const std::string& name)
{
    const auto [result, value] = param_->get_param_int(name);
    if (result != mavsdk::Param::Result::Success) {
        std::cerr << "[drone] get " << name << " failed: " << result << "\n";
        return std::nullopt;
    }
    return value;
}

std::optional<float> DroneInterface::get_param_float(const std::string& name)
{
    const auto [result, value] = param_->get_param_float(name);
    if (result != mavsdk::Param::Result::Success) {
        std::cerr << "[drone] get " << name << " failed: " << result << "\n";
        return std::nullopt;
    }
    return value;
}

bool DroneInterface::wait_ready(std::chrono::seconds timeout, const std::function<bool()>& cancel)
{
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
        if (cancel && cancel()) {
            return false;
        }
        const auto health = telemetry_->health();
        if (health.is_local_position_ok && health.is_global_position_ok &&
            health.is_home_position_ok && health.is_armable) {
            return true;
        }
        std::this_thread::sleep_for(kReadyPollInterval);
    }
    const auto health = telemetry_->health();
    std::cerr << "[drone] not ready: local_pos=" << health.is_local_position_ok
              << " global_pos=" << health.is_global_position_ok
              << " home=" << health.is_home_position_ok
              << " armable(pre-arm)=" << health.is_armable << "\n";
    return false;
}

void DroneInterface::request_position_rate()
{
    telemetry_->set_rate_position_velocity_ned(kFastTelemetryRateHz);
}

double DroneInterface::measure_position_rate(std::chrono::milliseconds window) const
{
    const auto updates_before = state().position_updates;
    const auto start = Clock::now();
    std::this_thread::sleep_for(window);
    const double elapsed_s = std::chrono::duration<double>(Clock::now() - start).count();
    return (state().position_updates - updates_before) / elapsed_s;
}

bool DroneInterface::set_mode(CopterMode mode, std::chrono::seconds timeout)
{
    const auto custom_mode = static_cast<uint32_t>(mode);
    const auto deadline = Clock::now() + timeout;
    auto next_command = Clock::now();
    while (Clock::now() < deadline) {
        if (state().is_in(mode)) {
            return true;
        }
        if (Clock::now() >= next_command) {
            mavsdk::MavlinkDirect::MavlinkMessage command;
            command.message_name = "COMMAND_LONG";
            command.target_system_id = system_->get_system_id();
            command.target_component_id = kAutopilotComponentId;
            char fields[256];
            std::snprintf(fields, sizeof(fields),
                          R"({"command":%d,"confirmation":0,"param1":%d,"param2":%u,)"
                          R"("param3":0,"param4":0,"param5":0,"param6":0,"param7":0})",
                          kMavCmdDoSetMode, kMavModeFlagCustomModeEnabled, custom_mode);
            command.fields_json = fields;
            mavlink_direct_->send_message(command);
            next_command = Clock::now() + kModeCommandResendInterval;
        }
        std::this_thread::sleep_for(kPollInterval);
    }
    std::cerr << "[drone] mode " << custom_mode << " not confirmed by heartbeat (custom_mode="
              << state().custom_mode << ")\n";
    return false;
}

bool DroneInterface::arm()
{
    const auto result = action_->arm();
    if (result != mavsdk::Action::Result::Success) {
        std::cerr << "[drone] arm failed: " << result << "\n";
        return false;
    }
    return true;
}

bool DroneInterface::land()
{
    // MAV_CMD_NAV_LAND: ArduCopter switches to LAND mode
    const auto result = action_->land();
    if (result != mavsdk::Action::Result::Success) {
        std::cerr << "[drone] land failed: " << result << "\n";
        return false;
    }
    return true;
}

bool DroneInterface::send_thrust(double thrust, double yaw_rad)
{
    // Level attitude with the requested yaw: q = (cos(y/2), 0, 0, sin(y/2))
    const double quaternion_w = std::cos(yaw_rad / 2.0);
    const double quaternion_z = std::sin(yaw_rad / 2.0);
    const auto time_boot_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch())
            .count());

    mavsdk::MavlinkDirect::MavlinkMessage message;
    message.message_name = "SET_ATTITUDE_TARGET";
    message.target_system_id = system_->get_system_id();
    message.target_component_id = kAutopilotComponentId;
    char fields[320];
    std::snprintf(fields, sizeof(fields),
                  R"({"time_boot_ms":%u,"type_mask":%d,"q":[%.7f,0,0,%.7f],)"
                  R"("body_roll_rate":0,"body_pitch_rate":0,"body_yaw_rate":0,"thrust":%.5f})",
                  time_boot_ms, kTypeMaskIgnoreBodyRates, quaternion_w, quaternion_z, thrust);
    message.fields_json = fields;
    return mavlink_direct_->send_message(message) == mavsdk::MavlinkDirect::Result::Success;
}

}  // namespace altctl
