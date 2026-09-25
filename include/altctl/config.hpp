#pragma once

#include <string>

namespace altctl {

struct PidGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double d_cutoff_hz = 5.0;  // low-pass filter on the D term
    double i_limit = 0.0;      // |integral contribution| limit (anti-windup)
    double out_min = -1.0;
    double out_max = 1.0;
};

// All tunables. Defaults live here; config/mission.conf overrides them (key = value).
struct Config {
    // Connection
    std::string connection_url = "udpin://0.0.0.0:14551";

    // Mission
    double alt_high_m = 10.0;
    double alt_low_m = 5.0;
    double hold_time_s = 10.0;        // time to hold each altitude once settled
    double settle_tolerance_m = 0.25; // |alt error| to count as "reached"
    double settle_time_s = 2.0;       // ... continuously for this long
    double state_timeout_s = 60.0;    // per-state timeout (climb/descend/hold)

    // Outer loop: altitude error -> climb-rate setpoint
    double alt_kp = 1.0;              // (m/s) per m
    double max_climb_ms = 2.0;
    double max_descent_ms = 1.0;
    double setpoint_rate_up_ms = 2.0;   // altitude setpoint ramp
    double setpoint_rate_down_ms = 1.0;

    // Inner loop: climb-rate error -> thrust correction around hover
    PidGains vel{0.05, 0.02, 0.0, 5.0, 0.15, -0.3, 0.3};
    double thrust_min = 0.10;
    double thrust_max = 0.80;
    double liftoff_alt_m = 0.3;       // integrator frozen below this altitude

    // Command stream
    double control_rate_hz = 50.0;
    double stream_watchdog_s = 0.5;   // loop period above this -> ERROR -> LAND

    // Logging
    std::string log_path = "logs/flight.csv";

    // Loads "key = value" lines (# comments). Unknown keys are an error.
    static Config load(const std::string& path);
    void set(const std::string& key, const std::string& value);
};

}  // namespace altctl
