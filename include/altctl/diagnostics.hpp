#pragma once

#include <atomic>

#include "altctl/config.hpp"

namespace altctl {

class DroneInterface;

// Bring-up checks used while building the controller (see README "Run").

// Prints altitude / climb / yaw / mode for a few seconds and the measured telemetry rate.
int run_telemetry_check(const Config& cfg, DroneInterface& drone);

// GUID_OPTIONS check, GUIDED, arm, then open-loop thrust (hover + 0.10) until 4 m, then LAND.
int run_open_loop_takeoff(const Config& cfg, DroneInterface& drone,
                          const std::atomic<bool>& stop_requested);

}  // namespace altctl
