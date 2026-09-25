#pragma once

#include "altctl/config.hpp"
#include "altctl/pid_controller.hpp"

namespace altctl {

// Cascade altitude controller:
//   target -> setpoint trajectory (trapezoidal: rate, acceleration and braking limited)
//   outer P:   climb_sp = ramp_rate + alt_kp * (setpoint - alt), clamped to climb/descent limits
//   inner PID: thrust correction from climb-rate error
//   thrust = hover_thrust + correction, clamped to [thrust_min, thrust_max]
// Take-off: until liftoff_alt_m above the ground, fixed thrust hover + takeoff_thrust_margin with
// the PID held in reset (no windup on the ground); then a bumpless hand-over to the cascade.
class AltitudeController {
public:
    struct Output {
        double alt_setpoint_m = 0.0;   // after ramp
        double climb_setpoint_ms = 0.0;
        double thrust = 0.0;
        bool integrator_frozen = false;
        PIDController::Terms vel_terms;
    };

    AltitudeController(const Config& cfg, double hover_thrust);

    // Call on the ground before take-off: sets the ground reference, clears all state.
    void reset(double ground_alt_m);

    Output update(double target_alt_m, double alt_m, double climb_ms, double dt);


private:
    Config cfg_;
    double hover_thrust_;
    double ramped_setpoint_m_ = 0.0;
    double ramp_rate_ms_ = 0.0;  // current setpoint velocity (fed forward)
    double ground_alt_m_ = 0.0;
    bool airborne_ = false;       // latched at liftoff
    PIDController vel_pid_;
};

}  // namespace altctl
