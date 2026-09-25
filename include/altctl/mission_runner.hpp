#pragma once

#include <atomic>

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

// Mission state machine + the fixed-rate control loop. The loop computes and sends a
// fresh thrust every tick (no separate sender thread: a sender repeating a stale thrust
// is exactly the failure mode found in Stage 2).
class MissionRunner {
public:
    MissionRunner(const Config& cfg, DroneInterface& drone, DataLogger& logger,
                  const std::atomic<bool>& stop_requested);

    int run();  // returns process exit code

private:
    const Config& cfg_;
    DroneInterface& drone_;
    DataLogger& logger_;
    const std::atomic<bool>& stop_requested_;
    MissionState state_ = MissionState::Init;
};

}  // namespace altctl
