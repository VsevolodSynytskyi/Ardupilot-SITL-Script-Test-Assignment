#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace mavsdk {
class Mavsdk;
class System;
class Telemetry;
class Action;
class Param;
class MavlinkDirect;
}  // namespace mavsdk

namespace altctl {

// ArduCopter custom_mode values (HEARTBEAT.custom_mode)
enum class CopterMode : uint32_t { Guided = 4, Land = 9 };

// Everything that talks MAVLink. Owns the single NED -> altitude conversion.
//   Telemetry: altitude/climb (LOCAL_POSITION_NED), yaw, armed, in-air
//   Param:     GUID_OPTIONS, MOT_THST_HOVER
//   Action:    arm, land
//   MavlinkDirect: SET_ATTITUDE_TARGET (thrust), DO_SET_MODE, HEARTBEAT.custom_mode
class DroneInterface {
public:
    struct State {
        double alt_m = 0.0;     // up-positive, relative to EKF origin
        double climb_ms = 0.0;  // up-positive
        double yaw_rad = 0.0;
        bool armed = false;
        bool in_air = false;
        uint32_t custom_mode = 0;
        uint64_t position_updates = 0;  // counter, for rate checks
        std::chrono::steady_clock::time_point last_position_update{};
        std::chrono::steady_clock::time_point last_heartbeat{};
    };

    explicit DroneInterface(std::string connection_url);
    ~DroneInterface();

    bool connect(std::chrono::seconds timeout);
    State state() const;  // thread-safe snapshot

    bool set_param_int(const std::string& name, int32_t value);
    std::optional<int32_t> get_param_int(const std::string& name);
    std::optional<float> get_param_float(const std::string& name);

    // EKF position + home + pre-arm checks; `cancel` is polled (e.g. Ctrl+C).
    bool wait_ready(std::chrono::seconds timeout, const std::function<bool()>& cancel = {});
    bool set_mode(CopterMode mode, std::chrono::seconds timeout);
    // Measured LOCAL_POSITION_NED rate over `window`; re-requests the 50 Hz rate first if asked.
    double measure_position_rate(std::chrono::milliseconds window, bool request_rate = false);
    bool arm();
    bool land();

    // SET_ATTITUDE_TARGET: level attitude at yaw_rad, body rates ignored (type_mask = 7).
    bool send_thrust(double thrust, double yaw_rad);

    static double altitude_from_ned(double z_down_m) { return -z_down_m; }
    static double climb_from_ned(double vz_down_ms) { return -vz_down_ms; }

private:
    std::string url_;
    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Param> param_;
    std::unique_ptr<mavsdk::MavlinkDirect> direct_;

    mutable std::mutex mutex_;
    State state_;
};

}  // namespace altctl
