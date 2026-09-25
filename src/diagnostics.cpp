#include "altctl/diagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

#include "altctl/drone_interface.hpp"

namespace altctl {

namespace {

using Clock = std::chrono::steady_clock;

double seconds_since(Clock::time_point t0)
{
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

void wait_disarmed(DroneInterface& drone, std::chrono::seconds timeout)
{
    const auto deadline = Clock::now() + timeout;
    while (drone.state().armed && Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << (drone.state().armed ? "[diag] still armed after LAND timeout\n"
                                      : "[diag] landed and disarmed\n");
}

}  // namespace

int run_telemetry_check(const Config& /*cfg*/, DroneInterface& drone)
{
    constexpr double kDuration = 5.0;
    const auto n0 = drone.state().position_updates;
    const auto t0 = Clock::now();
    while (seconds_since(t0) < kDuration) {
        const auto s = drone.state();
        std::printf("[telem] t=%4.1fs alt=%6.2f m climb=%+5.2f m/s yaw=%6.1f deg armed=%d "
                    "in_air=%d mode=%u\n",
                    seconds_since(t0), s.alt_m, s.climb_ms, s.yaw_rad * 180.0 / M_PI, s.armed,
                    s.in_air, s.custom_mode);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    const double rate = (drone.state().position_updates - n0) / seconds_since(t0);
    std::printf("[telem] LOCAL_POSITION_NED rate: %.1f Hz\n", rate);
    return rate > 20.0 ? 0 : 1;
}

int run_open_loop_takeoff(const Config& cfg, DroneInterface& drone,
                          const std::atomic<bool>& stop_requested)
{
    // Parameters, readiness, mode, arm
    const auto guid = drone.get_param_int("GUID_OPTIONS");
    if (!guid || (*guid & 8) == 0) {
        std::cout << "[diag] GUID_OPTIONS=" << (guid ? *guid : -1) << ", setting 8\n";
        if (!drone.set_param_int("GUID_OPTIONS", 8)) {
            return 1;
        }
    }
    const auto hover = drone.get_param_float("MOT_THST_HOVER");
    if (!hover) {
        return 1;
    }
    std::printf("[diag] GUID_OPTIONS=%d MOT_THST_HOVER=%.3f\n", *drone.get_param_int("GUID_OPTIONS"),
                *hover);
    if (!drone.wait_ready(std::chrono::seconds(60)) ||
        !drone.set_mode(CopterMode::Guided, std::chrono::seconds(5))) {
        return 1;
    }
    std::cout << "[diag] GUIDED confirmed\n";
    if (!drone.arm()) {
        return 1;
    }
    const auto t_arm = Clock::now();
    std::cout << "[diag] armed, streaming thrust\n";

    // Fixed-rate stream, fresh command every tick
    const double thrust = std::min(static_cast<double>(*hover) + 0.10, cfg.thrust_max);
    const double yaw_hold = drone.state().yaw_rad;
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / cfg.control_rate_hz));
    auto next = Clock::now();
    auto last_tick = next;
    double max_dt = 0.0;
    double liftoff_s = -1.0;
    int ticks = 0;
    bool ok = true;
    while (true) {
        const auto s = drone.state();
        if (!s.armed && seconds_since(t_arm) > 1.0) {
            std::cout << "[diag] DISARMED during open-loop takeoff\n";
            ok = false;
            break;
        }
        if (liftoff_s < 0 && s.alt_m > cfg.liftoff_alt_m) {
            liftoff_s = seconds_since(t_arm);
        }
        if (s.alt_m > 4.0 || stop_requested || seconds_since(t_arm) > 15.0) {
            ok = s.alt_m > 4.0;
            break;
        }
        drone.send_thrust(thrust, yaw_hold);
        ++ticks;

        const auto now = Clock::now();
        max_dt = std::max(max_dt, std::chrono::duration<double>(now - last_tick).count());
        last_tick = now;
        if (ticks % 25 == 0) {
            std::printf("[diag] t=%5.2fs thrust=%.3f alt=%5.2f climb=%+5.2f in_air=%d\n",
                        seconds_since(t_arm), thrust, s.alt_m, s.climb_ms, s.in_air);
        }
        next += period;
        std::this_thread::sleep_until(next);
    }
    const double elapsed = seconds_since(t_arm);
    std::printf("[diag] %s: liftoff(>%.1f m) at %.2fs, end alt=%.2f m at %.2fs, "
                "%d cmds (%.1f Hz), max loop dt=%.1f ms\n",
                ok ? "OK" : "FAILED", cfg.liftoff_alt_m, liftoff_s, drone.state().alt_m, elapsed,
                ticks, ticks / elapsed, max_dt * 1000.0);

    drone.land();
    wait_disarmed(drone, std::chrono::seconds(60));
    return ok ? 0 : 1;
}

}  // namespace altctl
