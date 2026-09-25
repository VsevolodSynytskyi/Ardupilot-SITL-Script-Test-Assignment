#include "altctl/diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

#include "altctl/drone_interface.hpp"
#include "altctl/units.hpp"

namespace altctl {

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kTelemetryCheckDurationS = 5.0;
constexpr auto kTelemetryPrintInterval = std::chrono::milliseconds(500);
constexpr double kMinUsableTelemetryRateHz = 20.0;

constexpr double kOpenLoopThrustMargin = 0.10;
constexpr double kOpenLoopTargetAltM = 4.0;
constexpr double kOpenLoopTimeoutS = 15.0;
constexpr double kArmGraceS = 1.0;
constexpr int kProgressEveryTicks = 25;
constexpr auto kReadyTimeout = std::chrono::seconds(60);
constexpr auto kModeChangeTimeout = std::chrono::seconds(5);
constexpr auto kDisarmTimeout = std::chrono::seconds(60);
constexpr auto kDisarmPollInterval = std::chrono::milliseconds(200);

double seconds_since(Clock::time_point start)
{
    return std::chrono::duration<double>(Clock::now() - start).count();
}

void wait_disarmed(const DroneInterface& drone, std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (drone.state().armed && Clock::now() < deadline) {
        std::this_thread::sleep_for(kDisarmPollInterval);
    }
    std::cout << (drone.state().armed ? "[diag] still armed after LAND timeout\n"
                                      : "[diag] landed and disarmed\n");
}

bool ensure_thrust_as_thrust(DroneInterface& drone)
{
    const auto guid_options = drone.get_param_int("GUID_OPTIONS");
    if (guid_options && (*guid_options & kGuidOptionThrustAsThrust) != 0) {
        return true;
    }
    std::cout << "[diag] GUID_OPTIONS=" << (guid_options ? *guid_options : -1) << ", setting "
              << kGuidOptionThrustAsThrust << "\n";
    return drone.set_param_int("GUID_OPTIONS", kGuidOptionThrustAsThrust);
}

}  // namespace

int run_telemetry_check(const DroneInterface& drone)
{
    const auto updates_before = drone.state().position_updates;
    const auto start = Clock::now();
    while (seconds_since(start) < kTelemetryCheckDurationS) {
        const auto vehicle = drone.state();
        std::printf("[telem] t=%4.1fs alt=%6.2f m climb=%+5.2f m/s yaw=%6.1f deg armed=%d "
                    "in_air=%d mode=%u\n",
                    seconds_since(start), vehicle.alt_m, vehicle.climb_mps,
                    radians_to_degrees(vehicle.yaw_rad), vehicle.armed, vehicle.in_air,
                    vehicle.custom_mode);
        std::this_thread::sleep_for(kTelemetryPrintInterval);
    }
    const double rate_hz = (drone.state().position_updates - updates_before) / seconds_since(start);
    std::printf("[telem] LOCAL_POSITION_NED rate: %.1f Hz\n", rate_hz);
    return rate_hz > kMinUsableTelemetryRateHz ? 0 : 1;
}

int run_open_loop_takeoff(const Config& config, DroneInterface& drone,
                          const std::atomic<bool>& stop_requested)
{
    if (!ensure_thrust_as_thrust(drone)) {
        return 1;
    }
    const auto hover_thrust = drone.get_param_float("MOT_THST_HOVER");
    if (!hover_thrust) {
        return 1;
    }
    std::printf("[diag] MOT_THST_HOVER=%.3f\n", *hover_thrust);
    if (!drone.wait_ready(kReadyTimeout) || !drone.set_mode(CopterMode::Guided, kModeChangeTimeout)) {
        return 1;
    }
    std::cout << "[diag] GUIDED confirmed\n";
    if (!drone.arm()) {
        return 1;
    }
    const auto armed_at = Clock::now();
    std::cout << "[diag] armed, streaming thrust\n";

    // Fixed-rate stream, fresh command every tick
    const double thrust =
        std::min(static_cast<double>(*hover_thrust) + kOpenLoopThrustMargin, config.thrust_max);
    const double yaw_hold_rad = drone.state().yaw_rad;
    const auto control_period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.control_rate_hz));
    auto next_tick = Clock::now();
    auto previous_tick = next_tick;
    double max_loop_period_s = 0.0;
    double liftoff_after_s = -1.0;
    int commands_sent = 0;
    int failed_sends = 0;
    bool reached_target = false;
    while (true) {
        const auto vehicle = drone.state();
        if (!vehicle.armed && seconds_since(armed_at) > kArmGraceS) {
            std::cout << "[diag] DISARMED during open-loop takeoff\n";
            break;
        }
        if (liftoff_after_s < 0 && vehicle.alt_m > config.liftoff_alt_m) {
            liftoff_after_s = seconds_since(armed_at);
        }
        reached_target = vehicle.alt_m > kOpenLoopTargetAltM;
        if (reached_target || stop_requested || seconds_since(armed_at) > kOpenLoopTimeoutS) {
            break;
        }
        if (!drone.send_thrust(thrust, yaw_hold_rad)) {
            ++failed_sends;
        }
        ++commands_sent;

        const auto now = Clock::now();
        max_loop_period_s =
            std::max(max_loop_period_s, std::chrono::duration<double>(now - previous_tick).count());
        previous_tick = now;
        if (commands_sent % kProgressEveryTicks == 0) {
            std::printf("[diag] t=%5.2fs thrust=%.3f alt=%5.2f climb=%+5.2f in_air=%d\n",
                        seconds_since(armed_at), thrust, vehicle.alt_m, vehicle.climb_mps,
                        vehicle.in_air);
        }
        next_tick += control_period;
        std::this_thread::sleep_until(next_tick);
    }
    const double elapsed_s = seconds_since(armed_at);
    std::printf("[diag] %s: liftoff(>%.1f m) at %.2fs, end alt=%.2f m at %.2fs, "
                "%d cmds (%.1f Hz, %d failed), max loop dt=%.1f ms\n",
                reached_target ? "OK" : "FAILED", config.liftoff_alt_m, liftoff_after_s,
                drone.state().alt_m, elapsed_s, commands_sent, commands_sent / elapsed_s,
                failed_sends, max_loop_period_s * 1000.0);

    if (!drone.land()) {
        return 1;
    }
    wait_disarmed(drone, kDisarmTimeout);
    return reached_target ? 0 : 1;
}

}  // namespace altctl
