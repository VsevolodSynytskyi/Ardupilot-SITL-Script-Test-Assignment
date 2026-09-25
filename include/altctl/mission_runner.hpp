#pragma once

#include <atomic>
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

const char* to_string(MissionState s);

// Mission state machine + the fixed-rate control loop.
//
//   INIT -> SET_PARAMS -> SET_GUIDED -> ARM -> CLIMB_TO_HIGH -> HOLD_HIGH
//        -> DESCEND_TO_LOW -> HOLD_LOW -> LAND -> DONE        (any failure -> ERROR -> LAND)
//
// The flight states share one loop that computes and sends a fresh thrust every tick.
// There is no separate sender thread: a sender repeating a stale thrust is exactly the
// failure mode found in SITL (ArduPilot applies the last thrust until GUID_TIMEOUT).
class MissionRunner {
public:
    MissionRunner(const Config& cfg, DroneInterface& drone, DataLogger& logger,
                  const std::atomic<bool>& stop_requested);

    int run();  // returns process exit code: 0 done, 1 error, 130 interrupted

private:
    enum class FlightResult { Completed, Stopped, Failed };

    bool prepare();         // SET_PARAMS, SET_GUIDED, ARM
    FlightResult fly();     // CLIMB_TO_HIGH .. HOLD_LOW
    bool land();            // LAND: command LAND, stop streaming, wait for disarm
    void transition(MissionState next, const std::string& reason);
    bool fail(const std::string& reason);  // -> ERROR, returns false

    const Config& cfg_;
    DroneInterface& drone_;
    DataLogger& logger_;
    const std::atomic<bool>& stop_requested_;
    MissionState state_ = MissionState::Init;
    double hover_thrust_ = 0.0;
    bool armed_by_us_ = false;
};

}  // namespace altctl
