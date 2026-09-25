#pragma once

#include "altctl/config.hpp"
#include "altctl/pid_controller.hpp"

namespace altctl {

// Cascade altitude controller:
//   ramped altitude setpoint -> outer P -> climb-rate setpoint -> inner PID -> thrust correction
//   thrust = hover_thrust + correction, clamped to [thrust_min, thrust_max]
class AltitudeController {
public:
    struct Output {
        double alt_setpoint_m = 0.0;   // after ramp
        double climb_setpoint_ms = 0.0;
        double thrust = 0.0;
        PIDController::Terms vel_terms;
    };

    AltitudeController(const Config& cfg, double hover_thrust);

    // Start ramping from the current altitude; clears the integrator.
    void reset(double current_alt_m);

    Output update(double target_alt_m, double alt_m, double climb_ms, double dt);

private:
    Config cfg_;
    double hover_thrust_;
    double ramped_setpoint_m_ = 0.0;
    PIDController vel_pid_;
};

}  // namespace altctl
