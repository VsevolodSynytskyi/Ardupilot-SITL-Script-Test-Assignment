#include "altctl/drone_interface.hpp"

#include <atomic>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <thread>

#include <mavsdk/log_callback.hpp>
#include <mavsdk/mavsdk.hpp>
#include <mavsdk/plugins/action/action.hpp>
#include <mavsdk/plugins/mavlink_direct/mavlink_direct.hpp>
#include <mavsdk/plugins/param/param.hpp>
#include <mavsdk/plugins/telemetry/telemetry.hpp>

namespace altctl {

namespace {

constexpr double kTelemetryRateHz = 50.0;
constexpr uint8_t kAutopilotCompId = 1;  // MAV_COMP_ID_AUTOPILOT1
constexpr int kMavCmdDoSetMode = 176;
constexpr int kMavModeFlagCustomModeEnabled = 1;
// SET_ATTITUDE_TARGET: ignore body roll/pitch/yaw rates (ArduPilot needs all three or none)
constexpr int kTypeMaskIgnoreRates = 1 | 2 | 4;

// Minimal extraction of a numeric field from MavlinkDirect's flat JSON object.
std::optional<double> json_number(const std::string& json, const std::string& key)
{
    const auto pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) {
        return std::nullopt;
    }
    const auto colon = json.find(':', pos);
    if (colon == std::string::npos) {
        return std::nullopt;
    }
    try {
        return std::stod(json.substr(colon + 1));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::atomic<bool> g_link_up{false};

// Console filter for MAVSDK's own logging: autopilot STATUSTEXT messages become "[ap]" lines,
// MAVSDK debug/info chatter is dropped, warnings and errors are kept.
bool filter_mavsdk_log(mavsdk::log::Level level, const std::string& message, const std::string&, int)
{
    constexpr const char* kStatusText = "MAVLink: ";
    if (message.rfind(kStatusText, 0) == 0) {
        std::printf("[ap] %s\n", message.c_str() + std::strlen(kStatusText));
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
    // MAVSDK sends a heartbeat before the UDP peer is known; harmless until the link is up.
    if (!g_link_up && message.find("Sending message failed") != std::string::npos) {
        return true;
    }
    std::fprintf(stderr, "[mavsdk] %s\n", message.c_str());
    return true;
}

}  // namespace

DroneInterface::DroneInterface(std::string connection_url) : url_(std::move(connection_url))
{
    mavsdk::log::subscribe(filter_mavsdk_log);
}

DroneInterface::~DroneInterface()
{
    // Plugins (and their subscriptions) first, while mutex_/state_ that the callbacks use
    // are still alive: members are destroyed in reverse declaration order, which would
    // destroy mutex_ before the plugins and crash a late callback (SIGABRT on exit).
    direct_.reset();
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
    if (const auto res = mavsdk_->add_any_connection(url_);
        res != mavsdk::ConnectionResult::Success) {
        std::cerr << "[drone] connection " << url_ << " failed: " << res << "\n";
        return false;
    }
    auto system = mavsdk_->first_autopilot(static_cast<double>(timeout.count()));
    if (!system) {
        std::cerr << "[drone] no autopilot heartbeat on " << url_ << " within " << timeout.count()
                  << " s\n";
        return false;
    }
    system_ = *system;
    g_link_up = true;
    telemetry_ = std::make_unique<mavsdk::Telemetry>(system_);
    action_ = std::make_unique<mavsdk::Action>(system_);
    param_ = std::make_unique<mavsdk::Param>(system_);
    direct_ = std::make_unique<mavsdk::MavlinkDirect>(system_);

    // Telemetry rates (MAV_CMD_SET_MESSAGE_INTERVAL under the hood)
    const auto r1 = telemetry_->set_rate_position_velocity_ned(kTelemetryRateHz);
    const auto r2 = telemetry_->set_rate_attitude_euler(kTelemetryRateHz);
    const auto r3 = telemetry_->set_rate_in_air(10.0);
    if (r1 != mavsdk::Telemetry::Result::Success || r2 != mavsdk::Telemetry::Result::Success ||
        r3 != mavsdk::Telemetry::Result::Success) {
        std::cerr << "[drone] warning: set telemetry rate: " << r1 << ", " << r2 << ", " << r3
                  << "\n";
    }

    telemetry_->subscribe_position_velocity_ned([this](mavsdk::Telemetry::PositionVelocityNed pv) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.alt_m = altitude_from_ned(pv.position.down_m);
        state_.climb_ms = climb_from_ned(pv.velocity.down_m_s);
        state_.last_position_update = std::chrono::steady_clock::now();
        ++state_.position_updates;
    });
    telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle a) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.yaw_rad = a.yaw_deg * M_PI / 180.0;
    });
    telemetry_->subscribe_armed([this](bool armed) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.armed = armed;
    });
    telemetry_->subscribe_in_air([this](bool in_air) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.in_air = in_air;
    });
    const auto target_sysid = system_->get_system_id();
    direct_->subscribe_message("HEARTBEAT", [this, target_sysid](mavsdk::MavlinkDirect::MavlinkMessage m) {
        if (m.system_id != target_sysid || m.component_id != kAutopilotCompId) {
            return;
        }
        if (const auto mode = json_number(m.fields_json, "custom_mode")) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.custom_mode = static_cast<uint32_t>(*mode);
            state_.last_heartbeat = std::chrono::steady_clock::now();
        }
    });
    {
        // Seed the snapshot so the first state() after connect() is meaningful.
        std::lock_guard<std::mutex> lock(mutex_);
        state_.armed = telemetry_->armed();
        state_.in_air = telemetry_->in_air();
    }
    const auto hb_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (state().last_heartbeat == std::chrono::steady_clock::time_point{} &&
           std::chrono::steady_clock::now() < hb_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cout << "[drone] connected to system " << static_cast<int>(target_sysid) << " via "
              << url_ << "\n";
    return true;
}

DroneInterface::State DroneInterface::state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool DroneInterface::set_param_int(const std::string& name, int32_t value)
{
    const auto res = param_->set_param_int(name, value);
    if (res != mavsdk::Param::Result::Success) {
        std::cerr << "[drone] set " << name << "=" << value << " failed: " << res << "\n";
        return false;
    }
    return true;
}

std::optional<int32_t> DroneInterface::get_param_int(const std::string& name)
{
    const auto [res, value] = param_->get_param_int(name);
    if (res != mavsdk::Param::Result::Success) {
        std::cerr << "[drone] get " << name << " failed: " << res << "\n";
        return std::nullopt;
    }
    return value;
}

std::optional<float> DroneInterface::get_param_float(const std::string& name)
{
    const auto [res, value] = param_->get_param_float(name);
    if (res != mavsdk::Param::Result::Success) {
        std::cerr << "[drone] get " << name << " failed: " << res << "\n";
        return std::nullopt;
    }
    return value;
}

bool DroneInterface::wait_ready(std::chrono::seconds timeout, const std::function<bool()>& cancel)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (cancel && cancel()) {
            return false;
        }
        const auto h = telemetry_->health();
        if (h.is_local_position_ok && h.is_global_position_ok && h.is_home_position_ok &&
            h.is_armable) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    const auto h = telemetry_->health();
    std::cerr << "[drone] not ready: local_pos=" << h.is_local_position_ok
              << " global_pos=" << h.is_global_position_ok << " home=" << h.is_home_position_ok
              << " armable(pre-arm)=" << h.is_armable << "\n";
    return false;
}

double DroneInterface::measure_position_rate(std::chrono::milliseconds window, bool request_rate)
{
    if (request_rate) {
        telemetry_->set_rate_position_velocity_ned(kTelemetryRateHz);
    }
    const auto n0 = state().position_updates;
    const auto t0 = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(window);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return (state().position_updates - n0) / elapsed;
}

bool DroneInterface::set_mode(CopterMode mode, std::chrono::seconds timeout)
{
    const auto wanted = static_cast<uint32_t>(mode);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_send = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline) {
        if (state().custom_mode == wanted) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= next_send) {
            mavsdk::MavlinkDirect::MavlinkMessage msg;
            msg.message_name = "COMMAND_LONG";
            msg.target_system_id = system_->get_system_id();
            msg.target_component_id = kAutopilotCompId;
            char json[256];
            std::snprintf(json, sizeof(json),
                          R"({"command":%d,"confirmation":0,"param1":%d,"param2":%u,)"
                          R"("param3":0,"param4":0,"param5":0,"param6":0,"param7":0})",
                          kMavCmdDoSetMode, kMavModeFlagCustomModeEnabled, wanted);
            msg.fields_json = json;
            direct_->send_message(msg);
            next_send = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::cerr << "[drone] mode " << wanted << " not confirmed by heartbeat (custom_mode="
              << state().custom_mode << ")\n";
    return false;
}

bool DroneInterface::arm()
{
    const auto res = action_->arm();
    if (res != mavsdk::Action::Result::Success) {
        std::cerr << "[drone] arm failed: " << res << "\n";
        return false;
    }
    return true;
}

bool DroneInterface::land()
{
    // MAV_CMD_NAV_LAND: ArduCopter switches to LAND mode
    const auto res = action_->land();
    if (res != mavsdk::Action::Result::Success) {
        std::cerr << "[drone] land failed: " << res << "\n";
        return false;
    }
    return true;
}

bool DroneInterface::send_thrust(double thrust, double yaw_rad)
{
    // Level attitude with the requested yaw: q = (cos(y/2), 0, 0, sin(y/2))
    const double qw = std::cos(yaw_rad / 2.0);
    const double qz = std::sin(yaw_rad / 2.0);
    const auto boot_ms = static_cast<unsigned>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count() &
        0xffffffffu);

    mavsdk::MavlinkDirect::MavlinkMessage msg;
    msg.message_name = "SET_ATTITUDE_TARGET";
    msg.target_system_id = system_->get_system_id();
    msg.target_component_id = kAutopilotCompId;
    char json[320];
    std::snprintf(json, sizeof(json),
                  R"({"time_boot_ms":%u,"type_mask":%d,"q":[%.7f,0,0,%.7f],)"
                  R"("body_roll_rate":0,"body_pitch_rate":0,"body_yaw_rate":0,"thrust":%.5f})",
                  boot_ms, kTypeMaskIgnoreRates, qw, qz, thrust);
    msg.fields_json = json;
    return direct_->send_message(msg) == mavsdk::MavlinkDirect::Result::Success;
}

}  // namespace altctl
