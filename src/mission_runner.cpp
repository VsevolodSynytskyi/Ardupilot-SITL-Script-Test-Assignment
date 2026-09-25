#include "altctl/mission_runner.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
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

double seconds(Clock::duration d)
{
    return std::chrono::duration<double>(d).count();
}

// Telemetry older than this during flight is treated as a lost link (10 samples at 50 Hz).
constexpr double kTelemetryStaleS = 0.2;
// Minimum LOCAL_POSITION_NED rate required before arming.
constexpr double kMinTelemetryRateHz = 25.0;
// After arming, ArduPilot may briefly report disarmed; ignore that for this long.
constexpr double kArmGraceS = 1.0;
// How long LAND is retried (e.g. while the link is down) before giving up.
constexpr double kLandCommandRetryS = 30.0;

}  // namespace

const char* to_string(MissionState s)
{
    switch (s) {
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

MissionRunner::MissionRunner(const Config& cfg, DroneInterface& drone, DataLogger& logger,
                             const std::atomic<bool>& stop_requested)
    : cfg_(cfg), drone_(drone), logger_(logger), stop_requested_(stop_requested)
{
}

void MissionRunner::transition(MissionState next, const std::string& reason)
{
    std::printf("[mission] %s -> %s%s%s\n", to_string(state_), to_string(next),
                reason.empty() ? "" : ": ", reason.c_str());
    std::fflush(stdout);
    state_ = next;
}

bool MissionRunner::fail(const std::string& reason)
{
    transition(MissionState::Error, reason);
    return false;
}

int MissionRunner::run()
{
    if (!prepare()) {
        if (armed_by_us_) {
            land();
        }
        return 1;
    }
    const auto result = fly();
    if (result == FlightResult::ReleasedToOperator) {
        std::printf("[mission] thrust stream stopped; vehicle left to the operator\n");
        return 3;
    }
    const bool landed = land();
    if (result == FlightResult::Completed && landed) {
        transition(MissionState::Done, "mission complete");
        return 0;
    }
    return result == FlightResult::Stopped ? 130 : 1;
}

bool MissionRunner::prepare()
{
    if (drone_.state().armed) {
        // Mission and take-off logic assume a vehicle on the ground (ground reference, take-off
        // phase); never take over an armed vehicle.
        return fail("vehicle is already armed; land and disarm it first");
    }
    transition(MissionState::SetParams, "");
    auto guid = drone_.get_param_int("GUID_OPTIONS");
    if (!guid || (*guid & 8) == 0) {
        std::printf("[mission] GUID_OPTIONS=%d, setting 8 (thrust as thrust)\n", guid ? *guid : -1);
        if (!drone_.set_param_int("GUID_OPTIONS", 8)) {
            return fail("cannot set GUID_OPTIONS");
        }
        guid = drone_.get_param_int("GUID_OPTIONS");
    }
    if (!guid || (*guid & 8) == 0) {
        return fail("GUID_OPTIONS readback is not 8");
    }
    const auto hover = drone_.get_param_float("MOT_THST_HOVER");
    if (!hover || *hover < 0.1f || *hover > 0.8f) {
        return fail("MOT_THST_HOVER missing or implausible");
    }
    hover_thrust_ = *hover;
    const auto guid_timeout = drone_.get_param_float("GUID_TIMEOUT");
    std::printf("[mission] GUID_OPTIONS=%d MOT_THST_HOVER=%.3f GUID_TIMEOUT=%.1f s\n", *guid,
                hover_thrust_, guid_timeout ? *guid_timeout : -1.0f);

    if (!drone_.wait_ready(std::chrono::seconds(90), [this] { return stop_requested_.load(); })) {
        return fail(stop_requested_ ? "interrupted by user" : "vehicle not ready (EKF / pre-arm)");
    }

    // The controller needs fresh altitude every tick; never arm on a slow telemetry stream
    // (seen in SITL after a router stall: ~3 Hz, bang-bang thrust). Right after SITL start the
    // stream may not flow yet even though the health flags are OK, so wait for it.
    double rate = drone_.measure_position_rate(std::chrono::milliseconds(1000));
    const auto rate_deadline = Clock::now() + std::chrono::seconds(60);
    while (rate < kMinTelemetryRateHz && Clock::now() < rate_deadline && !stop_requested_) {
        std::printf("[mission] waiting for telemetry: LOCAL_POSITION_NED at %.1f Hz\n", rate);
        rate = drone_.measure_position_rate(std::chrono::milliseconds(1500), true);
    }
    if (stop_requested_) {
        return fail("interrupted by user");
    }
    if (rate < kMinTelemetryRateHz) {
        return fail("telemetry too slow for control (" + std::to_string(rate) + " Hz)");
    }
    std::printf("[mission] telemetry rate %.1f Hz\n", rate);

    transition(MissionState::SetGuided, "");
    if (!drone_.set_mode(CopterMode::Guided, std::chrono::seconds(5))) {
        return fail("GUIDED not confirmed");
    }

    transition(MissionState::Arm, "");
    if (stop_requested_) {
        return fail("interrupted by user");
    }
    if (!drone_.arm()) {
        return fail("arming rejected");
    }
    armed_by_us_ = true;
    return true;
}

MissionRunner::FlightResult MissionRunner::fly()
{
#if defined(__APPLE__)
    // macOS coalesces timers of ordinary background processes (loop gaps up to ~100 ms seen);
    // the control loop is latency-sensitive.
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    AltitudeController ctl(cfg_, hover_thrust_);
    const auto s0 = drone_.state();
    ctl.reset(s0.alt_m);
    const double yaw_hold = s0.yaw_rad;  // level attitude, heading held from arming
    const double ground_alt = s0.alt_m;
    const double max_alt = cfg_.alt_high_m + 5.0;

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / cfg_.control_rate_hz));
    const auto t_start = Clock::now();
    auto t_state = t_start;       // entry time of current state
    auto t_in_tol = Clock::time_point{};  // since when |error| < tolerance (0 = not in tolerance)
    auto last_tick = t_start;
    auto next_tick = t_start;
    auto last_send_ok = t_start;
    uint64_t ticks = 0;
    double max_dt = 0.0;

    transition(MissionState::ClimbToHigh, "streaming thrust");
    while (true) {
        const auto now = Clock::now();
        const double dt = seconds(now - last_tick);
        last_tick = now;
        max_dt = std::max(max_dt, dt);
        const auto s = drone_.state();
        const double in_state = seconds(now - t_state);

        // --- safety checks -------------------------------------------------------------
        if (stop_requested_) {
            transition(MissionState::Error, "interrupted by user");
            return FlightResult::Stopped;
        }
        if (ticks > 0 && dt > cfg_.stream_watchdog_s) {
            fail("control loop stalled: " + std::to_string(dt) + " s between commands");
            return FlightResult::Failed;
        }
        if (seconds(now - s.last_position_update) > kTelemetryStaleS) {
            fail("telemetry stale (no LOCAL_POSITION_NED for " +
                 std::to_string(seconds(now - s.last_position_update)) + " s)");
            return FlightResult::Failed;
        }
        if (!s.armed && seconds(now - t_start) > kArmGraceS) {
            fail("vehicle disarmed unexpectedly");
            return FlightResult::Failed;
        }
        if (seconds(now - last_send_ok) > cfg_.stream_watchdog_s) {
            fail("thrust commands failing to send");
            return FlightResult::Failed;
        }
        if (s.custom_mode != static_cast<uint32_t>(CopterMode::Guided)) {
            // Someone else (operator / GCS) changed the mode: do not fight them.
            transition(MissionState::Error, "mode changed externally (custom_mode=" +
                                                std::to_string(s.custom_mode) + ")");
            return FlightResult::ReleasedToOperator;
        }
        if (s.alt_m > max_alt) {
            fail("altitude " + std::to_string(s.alt_m) + " m above limit");
            return FlightResult::Failed;
        }

        // --- state machine -------------------------------------------------------------
        const bool climbing_or_holding_high =
            state_ == MissionState::ClimbToHigh || state_ == MissionState::HoldHigh;
        const double target = climbing_or_holding_high ? cfg_.alt_high_m : cfg_.alt_low_m;
        const double error = target - (s.alt_m - ground_alt);
        if (std::abs(error) < cfg_.settle_tolerance_m) {
            if (t_in_tol == Clock::time_point{}) {
                t_in_tol = now;
            }
        } else {
            t_in_tol = Clock::time_point{};
        }
        const bool settled = t_in_tol != Clock::time_point{} &&
                             seconds(now - t_in_tol) >= cfg_.settle_time_s;

        auto enter = [&](MissionState next, const std::string& why) {
            transition(next, why);
            t_state = now;
            t_in_tol = Clock::time_point{};
        };
        char why[96];
        switch (state_) {
            case MissionState::ClimbToHigh:
            case MissionState::DescendToLow:
                if (settled) {
                    std::snprintf(why, sizeof(why), "|err|<%.2f m for %.1f s (after %.1f s)",
                                  cfg_.settle_tolerance_m, cfg_.settle_time_s, in_state);
                    enter(state_ == MissionState::ClimbToHigh ? MissionState::HoldHigh
                                                              : MissionState::HoldLow,
                          why);
                } else if (in_state > cfg_.state_timeout_s) {
                    fail(std::string(to_string(state_)) + " timed out");
                    return FlightResult::Failed;
                }
                break;
            case MissionState::HoldHigh:
            case MissionState::HoldLow:
                if (in_state >= cfg_.hold_time_s) {
                    std::snprintf(why, sizeof(why), "held %.1f s", in_state);
                    if (state_ == MissionState::HoldHigh) {
                        enter(MissionState::DescendToLow, why);
                    } else {
                        std::printf("[mission] %llu commands, %.1f Hz, max loop dt %.1f ms\n",
                                    static_cast<unsigned long long>(ticks),
                                    ticks / seconds(now - t_start), max_dt * 1000.0);
                        return FlightResult::Completed;
                    }
                }
                break;
            default:
                break;
        }

        // --- control + command ------------------------------------------------------------
        const double target_now =
            (state_ == MissionState::ClimbToHigh || state_ == MissionState::HoldHigh)
                ? cfg_.alt_high_m
                : cfg_.alt_low_m;
        const auto out = ctl.update(ground_alt + target_now, s.alt_m, s.climb_ms, ticks ? dt : 0.0);
        if (drone_.send_thrust(out.thrust, yaw_hold)) {
            last_send_ok = now;
        }
        ++ticks;

        auto logged = out;  // log altitudes relative to the take-off point
        logged.alt_setpoint_m -= ground_alt;
        logger_.write({seconds(now - t_start), to_string(state_), target_now, s.alt_m - ground_alt,
                       s.climb_ms, dt, logged});
        if (ticks % static_cast<uint64_t>(cfg_.control_rate_hz) == 0) {
            std::printf("[ctl] t=%5.1fs %-14s target=%5.2f sp=%5.2f alt=%5.2f climb=%+5.2f "
                        "thrust=%.3f i=%+.3f\n",
                        seconds(now - t_start), to_string(state_), target_now,
                        logged.alt_setpoint_m, s.alt_m - ground_alt, s.climb_ms,
                        out.thrust, out.vel_terms.i);
            std::fflush(stdout);
        }

        next_tick += period;
        if (next_tick < Clock::now()) {
            next_tick = Clock::now();  // overrun: don't try to catch up with a burst
        }
        std::this_thread::sleep_until(next_tick);
    }
}

bool MissionRunner::land()
{
    transition(MissionState::Land, "switching to LAND, thrust stream stopped");
    // Retry: after a link loss the command only gets through once the link is back.
    // Meanwhile ArduPilot holds altitude (GUID_TIMEOUT expired with no thrust stream).
    const auto retry_until = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                                std::chrono::duration<double>(kLandCommandRetryS));
    bool commanded = false;
    while (!commanded && Clock::now() < retry_until) {
        commanded = drone_.land() || drone_.set_mode(CopterMode::Land, std::chrono::seconds(3));
        if (drone_.state().custom_mode == static_cast<uint32_t>(CopterMode::Land)) {
            commanded = true;
        }
    }
    if (!commanded) {
        std::printf("[mission] could not command LAND for %.0f s; vehicle is holding altitude in "
                    "GUIDED, take over from the GCS\n",
                    kLandCommandRetryS);
        return false;
    }
    const auto deadline = Clock::now() + std::chrono::seconds(120);
    auto last_print = Clock::now();
    while (drone_.state().armed && Clock::now() < deadline) {
        if (seconds(Clock::now() - last_print) > 2.0) {
            std::printf("[mission] landing: alt=%.2f m\n", drone_.state().alt_m);
            last_print = Clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (drone_.state().armed) {
        std::printf("[mission] still armed after LAND timeout\n");
        return false;
    }
    std::printf("[mission] landed and disarmed\n");
    return true;
}

}  // namespace altctl
