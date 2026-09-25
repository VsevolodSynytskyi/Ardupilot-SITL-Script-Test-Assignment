#include "altctl/mission_runner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <thread>

#if defined(__APPLE__)
#include <pthread/qos.h>
#endif

#include "altctl/altitude_controller.hpp"
#include "altctl/data_logger.hpp"
#include "altctl/drone_interface.hpp"

namespace altctl {

namespace {

using Clock = std::chrono::steady_clock;

double seconds(Clock::duration duration)
{
    return std::chrono::duration<double>(duration).count();
}

Clock::duration duration_from_seconds(double duration_s)
{
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(duration_s));
}

// Telemetry older than this during flight is treated as a lost link (10 samples at 50 Hz).
constexpr double kTelemetryStaleS = 0.2;
// Minimum LOCAL_POSITION_NED rate required before arming.
constexpr double kMinTelemetryRateHz = 25.0;
// After arming, ArduPilot may briefly report disarmed; ignore that for this long.
constexpr double kArmGraceS = 1.0;
// How long LAND is retried (e.g. while the link is down) before giving up.
constexpr double kLandCommandRetryS = 30.0;
// Abort if the vehicle climbs this far above the high target.
constexpr double kAltitudeLimitMarginM = 5.0;

constexpr auto kReadyTimeout = std::chrono::seconds(90);
constexpr auto kTelemetryWaitTimeout = std::chrono::seconds(60);
constexpr auto kFirstRateWindow = std::chrono::milliseconds(1000);
constexpr auto kRetryRateWindow = std::chrono::milliseconds(1500);
constexpr int kTelemetryWaitReportEvery = 4;
constexpr auto kModeChangeTimeout = std::chrono::seconds(5);
constexpr auto kLandModeTimeout = std::chrono::seconds(3);
constexpr auto kDisarmTimeout = std::chrono::seconds(120);
constexpr auto kLandingPollInterval = std::chrono::milliseconds(100);
constexpr double kLandingReportIntervalS = 2.0;
constexpr float kMinPlausibleHoverThrust = 0.1F;
constexpr float kMaxPlausibleHoverThrust = 0.8F;

struct TickStatus {
    bool first_tick = true;
    double loop_period_s = 0.0;
    double telemetry_age_s = 0.0;
    double time_since_start_s = 0.0;
    double time_since_last_send_s = 0.0;
    bool armed = false;
    double alt_m = 0.0;
    double max_alt_m = 0.0;
};

std::optional<std::string> find_safety_violation(const TickStatus& tick, const Config& config)
{
    if (!tick.first_tick && tick.loop_period_s > config.stream_watchdog_s) {
        return "control loop stalled: " + std::to_string(tick.loop_period_s) +
               " s between commands";
    }
    if (tick.telemetry_age_s > kTelemetryStaleS) {
        return "telemetry stale (no LOCAL_POSITION_NED for " +
               std::to_string(tick.telemetry_age_s) + " s)";
    }
    if (!tick.armed && tick.time_since_start_s > kArmGraceS) {
        return std::string("vehicle disarmed unexpectedly");
    }
    if (tick.time_since_last_send_s > config.stream_watchdog_s) {
        return std::string("thrust commands failing to send");
    }
    if (tick.alt_m > tick.max_alt_m) {
        return "altitude " + std::to_string(tick.alt_m) + " m above limit";
    }
    return std::nullopt;
}

bool is_holding(MissionState state)
{
    return state == MissionState::HoldHigh || state == MissionState::HoldLow;
}

}  // namespace

const char* to_string(MissionState state)
{
    switch (state) {
        case MissionState::Init: return "INIT";
        case MissionState::SetParams: return "SET_PARAMS";
        case MissionState::SetGuided: return "SET_GUIDED";
        case MissionState::Arm: return "ARM";
        case MissionState::ClimbToHigh: return "CLIMB_TO_HIGH";
        case MissionState::HoldHigh: return "HOLD_HIGH";
        case MissionState::DescendToLow: return "DESCEND_TO_LOW";
        case MissionState::HoldLow: return "HOLD_LOW";
        case MissionState::Land: return "LAND";
        case MissionState::Done: return "DONE";
        case MissionState::Error: return "ERROR";
    }
    return "?";
}

MissionRunner::MissionRunner(const Config& config, DroneInterface& drone, DataLogger& logger,
                             const std::atomic<bool>& stop_requested)
    : config_(config), drone_(drone), logger_(logger), stop_requested_(stop_requested)
{
}

int MissionRunner::run()
{
    if (!prepare_vehicle()) {
        if (armed_by_mission_) {
            land_and_wait_disarmed();
        }
        return kExitError;
    }
    const auto result = fly_mission();
    if (result == FlightResult::ReleasedToOperator) {
        std::printf("[mission] thrust stream stopped; vehicle left to the operator\n");
        return kExitReleasedToOperator;
    }
    const bool landed = land_and_wait_disarmed();
    if (result == FlightResult::Completed && landed) {
        transition(MissionState::Done, "mission complete");
        return kExitDone;
    }
    return result == FlightResult::Stopped ? kExitInterrupted : kExitError;
}

bool MissionRunner::prepare_vehicle()
{
    if (drone_.state().armed) {
        // Mission and take-off logic assume a vehicle on the ground (ground reference, take-off
        // phase); never take over an armed vehicle.
        return fail("vehicle is already armed; land and disarm it first");
    }
    transition(MissionState::SetParams, "");
    if (!configure_parameters()) {
        return false;
    }
    if (!drone_.wait_ready(kReadyTimeout, [this] { return stop_requested_.load(); })) {
        return fail(stop_requested_ ? "interrupted by user" : "vehicle not ready (EKF / pre-arm)");
    }
    if (!wait_for_telemetry()) {
        return false;
    }

    transition(MissionState::SetGuided, "");
    if (!drone_.set_mode(CopterMode::Guided, kModeChangeTimeout)) {
        return fail("GUIDED not confirmed");
    }

    transition(MissionState::Arm, "");
    if (stop_requested_) {
        return fail("interrupted by user");
    }
    if (!drone_.arm()) {
        return fail("arming rejected");
    }
    armed_by_mission_ = true;
    return true;
}

bool MissionRunner::configure_parameters()
{
    auto guid_options = drone_.get_param_int("GUID_OPTIONS");
    if (!guid_options || (*guid_options & kGuidOptionThrustAsThrust) == 0) {
        std::printf("[mission] GUID_OPTIONS=%d, setting %d (thrust as thrust)\n",
                    guid_options ? *guid_options : -1, kGuidOptionThrustAsThrust);
        if (!drone_.set_param_int("GUID_OPTIONS", kGuidOptionThrustAsThrust)) {
            return fail("cannot set GUID_OPTIONS");
        }
        guid_options = drone_.get_param_int("GUID_OPTIONS");
    }
    if (!guid_options || (*guid_options & kGuidOptionThrustAsThrust) == 0) {
        return fail("GUID_OPTIONS readback does not have bit 3 set");
    }

    const auto hover_thrust = drone_.get_param_float("MOT_THST_HOVER");
    if (!hover_thrust || *hover_thrust < kMinPlausibleHoverThrust ||
        *hover_thrust > kMaxPlausibleHoverThrust) {
        return fail("MOT_THST_HOVER missing or implausible");
    }
    hover_thrust_ = *hover_thrust;

    const auto guided_timeout_s = drone_.get_param_float("GUID_TIMEOUT");
    std::printf("[mission] GUID_OPTIONS=%d MOT_THST_HOVER=%.3f GUID_TIMEOUT=%.1f s\n",
                *guid_options, hover_thrust_, guided_timeout_s ? *guided_timeout_s : -1.0F);
    return true;
}

// The controller needs fresh altitude every tick; never arm on a slow telemetry stream
// (flying on ~3 Hz position data gives bang-bang thrust). Right after SITL start the stream
// may not flow yet even though the health flags are OK, so wait for it.
bool MissionRunner::wait_for_telemetry()
{
    double rate_hz = drone_.measure_position_rate(kFirstRateWindow);
    const auto deadline = Clock::now() + kTelemetryWaitTimeout;
    for (int attempt = 0;
         rate_hz < kMinTelemetryRateHz && Clock::now() < deadline && !stop_requested_; ++attempt) {
        if (attempt % kTelemetryWaitReportEvery == 0) {
            std::printf("[mission] waiting for telemetry: LOCAL_POSITION_NED at %.1f Hz\n",
                        rate_hz);
        }
        drone_.request_position_rate();
        rate_hz = drone_.measure_position_rate(kRetryRateWindow);
    }
    if (stop_requested_) {
        return fail("interrupted by user");
    }
    if (rate_hz < kMinTelemetryRateHz) {
        return fail("telemetry too slow for control (" + std::to_string(rate_hz) + " Hz)");
    }
    std::printf("[mission] telemetry rate %.1f Hz\n", rate_hz);
    return true;
}

MissionRunner::FlightResult MissionRunner::fly_mission()
{
#if defined(__APPLE__)
    // macOS coalesces timers of ordinary background processes (loop gaps up to ~100 ms seen);
    // the control loop is latency-sensitive.
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    const auto initial = drone_.state();
    const double ground_alt_m = initial.alt_m;
    const double yaw_hold_rad = initial.yaw_rad;  // level attitude, heading held from arming
    AltitudeController controller(config_, hover_thrust_);
    controller.reset(ground_alt_m);

    const auto control_period = duration_from_seconds(1.0 / config_.control_rate_hz);
    const auto start = Clock::now();
    auto previous_tick = start;
    auto next_tick = start;
    auto last_successful_send = start;
    uint64_t commands_sent = 0;
    double max_loop_period_s = 0.0;

    enter_flight_state(MissionState::ClimbToHigh, "streaming thrust", start);
    while (true) {
        const auto now = Clock::now();
        const double loop_period_s = seconds(now - previous_tick);
        previous_tick = now;
        max_loop_period_s = std::max(max_loop_period_s, loop_period_s);
        const auto vehicle = drone_.state();

        if (stop_requested_) {
            transition(MissionState::Error, "interrupted by user");
            return FlightResult::Stopped;
        }
        const TickStatus tick{commands_sent == 0,
                              loop_period_s,
                              seconds(now - vehicle.last_position_update),
                              seconds(now - start),
                              seconds(now - last_successful_send),
                              vehicle.armed,
                              vehicle.alt_m,
                              ground_alt_m + config_.alt_high_m + kAltitudeLimitMarginM};
        if (const auto violation = find_safety_violation(tick, config_)) {
            fail(*violation);
            return FlightResult::Failed;
        }
        if (!vehicle.is_in(CopterMode::Guided)) {
            // Someone else (operator / GCS) changed the mode: do not fight them.
            transition(MissionState::Error, "mode changed externally (custom_mode=" +
                                                std::to_string(vehicle.custom_mode) + ")");
            return FlightResult::ReleasedToOperator;
        }

        const double alt_above_ground_m = vehicle.alt_m - ground_alt_m;
        switch (advance_mission_state(alt_above_ground_m, now)) {
            case StepResult::TimedOut:
                fail(std::string(to_string(state_)) + " timed out");
                return FlightResult::Failed;
            case StepResult::MissionComplete:
                std::printf("[mission] %llu commands, %.1f Hz, max loop dt %.1f ms\n",
                            static_cast<unsigned long long>(commands_sent),
                            commands_sent / seconds(now - start), max_loop_period_s * 1000.0);
                return FlightResult::Completed;
            case StepResult::Continue:
                break;
        }

        const auto output = controller.update(ground_alt_m + target_alt_m(), vehicle.alt_m,
                                              vehicle.climb_mps, tick.first_tick ? 0.0 : loop_period_s);
        if (drone_.send_thrust(output.thrust, yaw_hold_rad)) {
            last_successful_send = now;
        }
        ++commands_sent;

        auto logged_output = output;  // altitudes relative to the take-off point
        logged_output.setpoint_alt_m -= ground_alt_m;
        logger_.write({seconds(now - start), to_string(state_), target_alt_m(), alt_above_ground_m,
                       vehicle.climb_mps, loop_period_s, logged_output});
        if (commands_sent % static_cast<uint64_t>(config_.control_rate_hz) == 0) {
            std::printf("[ctl] t=%5.1fs %-14s target=%5.2f sp=%5.2f alt=%5.2f climb=%+5.2f "
                        "thrust=%.3f i=%+.3f\n",
                        seconds(now - start), to_string(state_), target_alt_m(),
                        logged_output.setpoint_alt_m, alt_above_ground_m, vehicle.climb_mps,
                        output.thrust, output.velocity_pid_terms.i);
            std::fflush(stdout);
        }

        next_tick += control_period;
        if (next_tick < Clock::now()) {
            next_tick = Clock::now();  // overrun: don't try to catch up with a burst
        }
        std::this_thread::sleep_until(next_tick);
    }
}

MissionRunner::StepResult MissionRunner::advance_mission_state(double alt_above_ground_m,
                                                               Clock::time_point now)
{
    const bool within_tolerance =
        std::abs(target_alt_m() - alt_above_ground_m) < config_.settle_tolerance_m;
    if (!within_tolerance) {
        within_tolerance_since_ = Clock::time_point{};
    } else if (within_tolerance_since_ == Clock::time_point{}) {
        within_tolerance_since_ = now;
    }
    const bool settled = within_tolerance_since_ != Clock::time_point{} &&
                         seconds(now - within_tolerance_since_) >= config_.settle_time_s;
    const double time_in_state_s = seconds(now - state_entered_);

    char reason[96];
    if (is_holding(state_)) {
        if (time_in_state_s < config_.hold_time_s) {
            return StepResult::Continue;
        }
        if (state_ == MissionState::HoldLow) {
            return StepResult::MissionComplete;
        }
        std::snprintf(reason, sizeof(reason), "held %.1f s", time_in_state_s);
        enter_flight_state(MissionState::DescendToLow, reason, now);
        return StepResult::Continue;
    }

    if (settled) {
        std::snprintf(reason, sizeof(reason), "|err|<%.2f m for %.1f s (after %.1f s)",
                      config_.settle_tolerance_m, config_.settle_time_s, time_in_state_s);
        enter_flight_state(state_ == MissionState::ClimbToHigh ? MissionState::HoldHigh
                                                               : MissionState::HoldLow,
                           reason, now);
        return StepResult::Continue;
    }
    return time_in_state_s > config_.state_timeout_s ? StepResult::TimedOut : StepResult::Continue;
}

bool MissionRunner::land_and_wait_disarmed()
{
    transition(MissionState::Land, "switching to LAND, thrust stream stopped");
    // Retry: after a link loss the command only gets through once the link is back.
    // Meanwhile ArduPilot holds altitude (GUID_TIMEOUT expired with no thrust stream).
    const auto retry_deadline = Clock::now() + duration_from_seconds(kLandCommandRetryS);
    bool land_commanded = false;
    while (!land_commanded && Clock::now() < retry_deadline) {
        land_commanded = drone_.land() || drone_.set_mode(CopterMode::Land, kLandModeTimeout) ||
                         drone_.state().is_in(CopterMode::Land);
    }
    if (!land_commanded) {
        std::printf("[mission] could not command LAND for %.0f s; vehicle is holding altitude in "
                    "GUIDED, take over from the GCS\n",
                    kLandCommandRetryS);
        return false;
    }

    const auto disarm_deadline = Clock::now() + kDisarmTimeout;
    auto last_report = Clock::now();
    while (drone_.state().armed && Clock::now() < disarm_deadline) {
        if (seconds(Clock::now() - last_report) > kLandingReportIntervalS) {
            std::printf("[mission] landing: alt=%.2f m\n", drone_.state().alt_m);
            last_report = Clock::now();
        }
        std::this_thread::sleep_for(kLandingPollInterval);
    }
    if (drone_.state().armed) {
        std::printf("[mission] still armed after LAND timeout\n");
        return false;
    }
    std::printf("[mission] landed and disarmed\n");
    return true;
}

double MissionRunner::target_alt_m() const
{
    return state_ == MissionState::ClimbToHigh || state_ == MissionState::HoldHigh
               ? config_.alt_high_m
               : config_.alt_low_m;
}

void MissionRunner::transition(MissionState next, const std::string& reason)
{
    std::printf("[mission] %s -> %s%s%s\n", to_string(state_), to_string(next),
                reason.empty() ? "" : ": ", reason.c_str());
    std::fflush(stdout);
    state_ = next;
}

void MissionRunner::enter_flight_state(MissionState next, const std::string& reason,
                                       Clock::time_point now)
{
    transition(next, reason);
    state_entered_ = now;
    within_tolerance_since_ = Clock::time_point{};
}

bool MissionRunner::fail(const std::string& reason)
{
    transition(MissionState::Error, reason);
    return false;
}

}  // namespace altctl
