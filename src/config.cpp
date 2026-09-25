#include "altctl/config.hpp"

#include <fstream>
#include <functional>
#include <map>
#include <stdexcept>

namespace altctl {

namespace {

std::string trim(const std::string& s)
{
    const auto b = s.find_first_not_of(" \t\r");
    if (b == std::string::npos) {
        return "";
    }
    const auto e = s.find_last_not_of(" \t\r");
    return s.substr(b, e - b + 1);
}

}  // namespace

void Config::set(const std::string& key, const std::string& value)
{
    const std::map<std::string, double*> numbers{
        {"alt_high_m", &alt_high_m},
        {"alt_low_m", &alt_low_m},
        {"hold_time_s", &hold_time_s},
        {"settle_tolerance_m", &settle_tolerance_m},
        {"settle_time_s", &settle_time_s},
        {"state_timeout_s", &state_timeout_s},
        {"alt_kp", &alt_kp},
        {"max_climb_ms", &max_climb_ms},
        {"max_descent_ms", &max_descent_ms},
        {"setpoint_rate_up_ms", &setpoint_rate_up_ms},
        {"setpoint_rate_down_ms", &setpoint_rate_down_ms},
        {"setpoint_accel_mss", &setpoint_accel_mss},
        {"vel_kp", &vel.kp},
        {"vel_ki", &vel.ki},
        {"vel_kd", &vel.kd},
        {"vel_d_cutoff_hz", &vel.d_cutoff_hz},
        {"vel_i_limit", &vel.i_limit},
        {"vel_out_min", &vel.out_min},
        {"vel_out_max", &vel.out_max},
        {"thrust_min", &thrust_min},
        {"thrust_max", &thrust_max},
        {"liftoff_alt_m", &liftoff_alt_m},
        {"takeoff_thrust_margin", &takeoff_thrust_margin},
        {"control_rate_hz", &control_rate_hz},
        {"stream_watchdog_s", &stream_watchdog_s},
    };

    if (key == "connection_url") {
        connection_url = value;
    } else if (key == "log_path") {
        log_path = value;
    } else if (auto it = numbers.find(key); it != numbers.end()) {
        try {
            *it->second = std::stod(value);
        } catch (const std::exception&) {
            throw std::runtime_error("config: bad number for '" + key + "': " + value);
        }
    } else {
        throw std::runtime_error("config: unknown key '" + key + "'");
    }
}

Config Config::load(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("config: cannot open " + path);
    }
    Config cfg;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        line = trim(line.substr(0, line.find('#')));
        if (line.empty()) {
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            throw std::runtime_error("config: " + path + ":" + std::to_string(line_no) +
                                     ": expected 'key = value'");
        }
        cfg.set(trim(line.substr(0, eq)), trim(line.substr(eq + 1)));
    }
    return cfg;
}

}  // namespace altctl
