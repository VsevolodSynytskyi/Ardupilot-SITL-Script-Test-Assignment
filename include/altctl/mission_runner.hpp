#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include "altctl/config.hpp"

namespace altctl {

class DroneInterface;
class DataLogger;

enum class MissionState {
    Init,
    SetParams,
    SetGuided,
    Arm,
    ClimbToHigh,
    HoldHigh,
    DescendToLow,
    HoldLow,
    Land,
    Done,
    Error,
};

const char* to_string(MissionState state);

// Mission state machine + the fixed-rate control loop.
//
//   INIT -> SET_PARAMS -> SET_GUIDED -> ARM -> CLIMB_TO_HIGH -> HOLD_HIGH
//        -> DESCEND_TO_LOW -> HOLD_LOW -> LAND -> DONE        (any failure -> ERROR -> LAND)
//
// Exception: if someone else changes the flight mode (operator/GCS), the runner stops
// streaming and exits without commanding anything, leaving the vehicle to the operator.
//
// The flight states share one loop that computes and sends a fresh thrust every tick.
// There is no separate sender thread: a sender repeating a stale thrust is exactly the
// failure mode found in SITL (ArduPilot applies the last thrust until GUID_TIMEOUT).
class MissionRunner {
public:
    static constexpr int kExitDone = 0;
    static constexpr int kExitError = 1;
    static constexpr int kExitReleasedToOperator = 3;
    static constexpr int kExitInterrupted = 130;

    MissionRunner(const Config& config, DroneInterface& drone, DataLogger& logger,
                  const std::atomic<bool>& stop_requested);

    [[nodiscard]] int run();

private:
    using Clock = std::chrono::steady_clock;

    enum class FlightResult { Completed, Stopped, Failed, ReleasedToOperator };
    enum class StepResult { Continue, MissionComplete, TimedOut };

    bool prepare_vehicle();  // SET_PARAMS, SET_GUIDED, ARM
    bool configure_parameters();
    bool wait_for_telemetry();
    FlightResult fly_mission();  // CLIMB_TO_HIGH .. HOLD_LOW
    StepResult advance_mission_state(double alt_above_ground_m, Clock::time_point now);
    bool land_and_wait_disarmed();  // LAND (retried, e.g. after link loss), wait for disarm

    [[nodiscard]] double target_alt_m() const;
    void transition(MissionState next, const std::string& reason);
    void enter_flight_state(MissionState next, const std::string& reason, Clock::time_point now);
    bool fail(const std::string& reason);  // -> ERROR, returns false

    const Config& config_;
    DroneInterface& drone_;
    DataLogger& logger_;
    const std::atomic<bool>& stop_requested_;
    MissionState state_ = MissionState::Init;
    double hover_thrust_ = 0.0;
    bool armed_by_mission_ = false;
    Clock::time_point state_entered_{};
    Clock::time_point within_tolerance_since_{};  // default value: not within tolerance
};

}  // namespace altctl
