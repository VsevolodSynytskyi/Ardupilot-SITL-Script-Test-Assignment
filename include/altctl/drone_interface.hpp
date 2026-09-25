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

// GUID_OPTIONS bit 3: SET_ATTITUDE_TARGET.thrust is thrust, not a climb-rate request.
inline constexpr int32_t kGuidOptionThrustAsThrust = 8;

// Everything that talks MAVLink. Owns the single NED -> altitude conversion.
//   Telemetry: altitude/climb (LOCAL_POSITION_NED), yaw, armed, in-air
//   Param:     GUID_OPTIONS, MOT_THST_HOVER
//   Action:    arm, land
//   MavlinkDirect: SET_ATTITUDE_TARGET (thrust), DO_SET_MODE, HEARTBEAT.custom_mode
class DroneInterface {
public:
    using Clock = std::chrono::steady_clock;

    struct State {
        double alt_m = 0.0;      // up-positive, relative to EKF origin
        double climb_mps = 0.0;  // up-positive
        double yaw_rad = 0.0;
        bool armed = false;
        bool in_air = false;
        uint32_t custom_mode = 0;
        uint64_t position_updates = 0;  // counter, for rate checks
        Clock::time_point last_position_update{};
        Clock::time_point last_heartbeat{};

        [[nodiscard]] bool is_in(CopterMode mode) const
        {
            return custom_mode == static_cast<uint32_t>(mode);
        }
    };

    explicit DroneInterface(std::string connection_url);
    ~DroneInterface();
    DroneInterface(const DroneInterface&) = delete;
    DroneInterface& operator=(const DroneInterface&) = delete;

    [[nodiscard]] bool connect(std::chrono::seconds timeout);
    [[nodiscard]] State state() const;  // thread-safe snapshot

    [[nodiscard]] bool set_param_int(const std::string& name, int32_t value);
    [[nodiscard]] std::optional<int32_t> get_param_int(const std::string& name);
    [[nodiscard]] std::optional<float> get_param_float(const std::string& name);

    // EKF position + home + pre-arm checks; `cancel` is polled (e.g. Ctrl+C).
    [[nodiscard]] bool wait_ready(std::chrono::seconds timeout,
                                  const std::function<bool()>& cancel = {});
    [[nodiscard]] bool set_mode(CopterMode mode, std::chrono::seconds timeout);
    void request_position_rate();
    [[nodiscard]] double measure_position_rate(std::chrono::milliseconds window) const;
    [[nodiscard]] bool arm();
    [[nodiscard]] bool land();

    // SET_ATTITUDE_TARGET: level attitude at yaw_rad, body rates ignored (type_mask = 7).
    [[nodiscard]] bool send_thrust(double thrust, double yaw_rad);

    static double altitude_from_ned(double z_down_m) { return -z_down_m; }
    static double climb_from_ned(double vz_down_mps) { return -vz_down_mps; }

private:
    void request_telemetry_streams();
    void subscribe_telemetry();
    void wait_for_first_heartbeat(std::chrono::milliseconds timeout) const;

    std::string connection_url_;
    std::unique_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::unique_ptr<mavsdk::Telemetry> telemetry_;
    std::unique_ptr<mavsdk::Action> action_;
    std::unique_ptr<mavsdk::Param> param_;
    std::unique_ptr<mavsdk::MavlinkDirect> mavlink_direct_;

    mutable std::mutex state_mutex_;
    State state_;
};

}  // namespace altctl
