#pragma once

#include <atomic>

#include "altctl/config.hpp"

namespace altctl {

class DroneInterface;

// Bring-up checks: `--run telemetry` and `--run open-loop`.

// Prints altitude / climb / yaw / mode for a few seconds and the measured telemetry rate.
int run_telemetry_check(const DroneInterface& drone);

// GUID_OPTIONS check, GUIDED, arm, then open-loop thrust (hover + 0.10) up to 4 m, then LAND.
int run_open_loop_takeoff(const Config& config, DroneInterface& drone,
                          const std::atomic<bool>& stop_requested);

}  // namespace altctl
