#pragma once

#include "altctl/config.hpp"
#include "altctl/pid_controller.hpp"

namespace altctl {

// Cascade altitude controller:
//   target -> setpoint trajectory (trapezoidal: speed, acceleration and braking limited)
//   outer P:   climb setpoint = trajectory speed + alt_kp * (setpoint - alt), clamped
//   inner PID: thrust correction from climb-rate error
//   thrust = hover_thrust + correction, clamped to [thrust_min, thrust_max]
// Take-off: until liftoff_alt_m above the ground, fixed thrust hover + takeoff_thrust_margin with
// the PID held in reset (no windup on the ground); then a bumpless hand-over to the cascade.
class AltitudeController {
public:
    struct Output {
        double setpoint_alt_m = 0.0;  // on the trajectory
        double climb_setpoint_mps = 0.0;
        double thrust = 0.0;
        bool integrator_frozen = false;
        PidController::Terms velocity_pid_terms;
    };

    AltitudeController(const Config& config, double hover_thrust);

    // Call on the ground before take-off: sets the ground reference, clears all state.
    void reset(double ground_alt_m);

    [[nodiscard]] Output update(double target_alt_m, double alt_m, double climb_mps, double dt_s);

private:
    Output takeoff_output(double alt_m);
    void advance_setpoint(double target_alt_m, double dt_s);

    Config config_;
    double hover_thrust_;
    double ground_alt_m_ = 0.0;
    double setpoint_alt_m_ = 0.0;
    double setpoint_speed_mps_ = 0.0;  // fed forward to the outer loop
    bool airborne_ = false;            // latched at liftoff
    PidController velocity_pid_;
};

}  // namespace altctl
