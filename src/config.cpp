#include "altctl/config.hpp"

#include <fstream>
#include <map>
#include <stdexcept>

namespace altctl {

namespace {

constexpr const char* kWhitespace = " \t\r";

std::string trim(const std::string& text)
{
    const auto first = text.find_first_not_of(kWhitespace);
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(kWhitespace);
    return text.substr(first, last - first + 1);
}

double parse_number(const std::string& key, const std::string& value)
{
    std::size_t parsed_chars = 0;
    double number = 0.0;
    try {
        number = std::stod(value, &parsed_chars);
    } catch (const std::exception&) {
        parsed_chars = 0;
    }
    if (parsed_chars == 0 || parsed_chars != value.size()) {
        throw std::runtime_error("config: bad number for '" + key + "': " + value);
    }
    return number;
}

}  // namespace

void Config::set(const std::string& key, const std::string& value)
{
    const std::map<std::string, double*> numeric_fields{
        {"alt_high_m", &alt_high_m},
        {"alt_low_m", &alt_low_m},
        {"hold_time_s", &hold_time_s},
        {"settle_tolerance_m", &settle_tolerance_m},
        {"settle_time_s", &settle_time_s},
        {"state_timeout_s", &state_timeout_s},
        {"alt_kp", &alt_kp},
        {"max_climb_mps", &max_climb_mps},
        {"max_descent_mps", &max_descent_mps},
        {"setpoint_speed_up_mps", &setpoint_speed_up_mps},
        {"setpoint_speed_down_mps", &setpoint_speed_down_mps},
        {"setpoint_accel_mps2", &setpoint_accel_mps2},
        {"vel_kp", &velocity_pid.kp},
        {"vel_ki", &velocity_pid.ki},
        {"vel_kd", &velocity_pid.kd},
        {"vel_d_cutoff_hz", &velocity_pid.derivative_cutoff_hz},
        {"vel_i_limit", &velocity_pid.integral_limit},
        {"vel_out_min", &velocity_pid.output_min},
        {"vel_out_max", &velocity_pid.output_max},
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
    } else if (const auto field = numeric_fields.find(key); field != numeric_fields.end()) {
        *field->second = parse_number(key, value);
    } else {
        throw std::runtime_error("config: unknown key '" + key + "'");
    }
}

Config Config::load(const std::string& path)
{
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("config: cannot open " + path);
    }
    Config config;
    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        line = trim(line.substr(0, line.find('#')));
        if (line.empty()) {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error("config: " + path + ":" + std::to_string(line_number) +
                                     ": expected 'key = value'");
        }
        config.set(trim(line.substr(0, separator)), trim(line.substr(separator + 1)));
    }
    return config;
}

}  // namespace altctl
