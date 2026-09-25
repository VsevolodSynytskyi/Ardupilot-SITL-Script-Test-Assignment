#pragma once

#include <string>

namespace altctl {

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double derivative_cutoff_hz = 5.0;  // low-pass filter on the D term
    double integral_limit = 0.0;        // |integral contribution| limit (anti-windup)
    double output_min = -1.0;
    double output_max = 1.0;
};

// All tunables. Defaults live here; config/mission.conf overrides them (key = value).
struct Config {
    // Connection
    std::string connection_url = "udpin://0.0.0.0:14551";

    // Mission
    double alt_high_m = 10.0;
    double alt_low_m = 5.0;
    double hold_time_s = 5.0;          // time to hold each altitude once settled
    double settle_tolerance_m = 0.25;  // |alt error| to count as "reached"
    double settle_time_s = 2.0;        // ... continuously for this long
    double state_timeout_s = 60.0;     // climb/descend must settle within this

    // Setpoint trajectory + outer loop: altitude error -> climb-rate setpoint
    double alt_kp = 2.0;  // (m/s) per m
    double max_climb_mps = 2.5;
    double max_descent_mps = 1.5;
    double setpoint_speed_up_mps = 1.5;
    double setpoint_speed_down_mps = 1.0;
    double setpoint_accel_mps2 = 0.7;

    // Inner loop: climb-rate error -> thrust correction around hover
    PidGains velocity_pid{0.7, 0.15, 0.0, 5.0, 0.15, -0.3, 0.3};  // tuned in SITL, see config/mission.conf
    double thrust_min = 0.10;
    double thrust_max = 0.80;
    double liftoff_alt_m = 0.3;           // take-off phase ends this far above the ground
    double takeoff_thrust_margin = 0.10;  // take-off thrust = hover + margin

    // Command stream
    double control_rate_hz = 50.0;
    double stream_watchdog_s = 0.5;  // loop stall / failed sends longer than this -> LAND

    // Logging
    std::string log_path = "logs/flight.csv";

    // Loads "key = value" lines (# comments). Unknown keys are an error.
    static Config load(const std::string& path);
    void set(const std::string& key, const std::string& value);
};

}  // namespace altctl
