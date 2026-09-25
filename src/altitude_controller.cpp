#include "altctl/altitude_controller.hpp"

#include <algorithm>
#include <cmath>

namespace altctl {

AltitudeController::AltitudeController(const Config& config, double hover_thrust)
    : config_(config), hover_thrust_(hover_thrust), velocity_pid_(config.velocity_pid)
{
}

void AltitudeController::reset(double ground_alt_m)
{
    ground_alt_m_ = ground_alt_m;
    setpoint_alt_m_ = ground_alt_m;
    setpoint_speed_mps_ = 0.0;
    airborne_ = false;
    velocity_pid_.reset();
}

AltitudeController::Output AltitudeController::update(double target_alt_m, double alt_m,
                                                      double climb_mps, double dt_s)
{
    const double step_dt_s = dt_s > 0.0 ? dt_s : 0.0;

    if (!airborne_ && alt_m - ground_alt_m_ >= config_.liftoff_alt_m) {
        airborne_ = true;
        // bumpless hand-over: trajectory starts where the vehicle is, at its current speed
        setpoint_alt_m_ = alt_m;
        setpoint_speed_mps_ =
            std::clamp(climb_mps, -config_.setpoint_speed_down_mps, config_.setpoint_speed_up_mps);
    }
    if (!airborne_) {
        return takeoff_output(alt_m);
    }

    advance_setpoint(target_alt_m, step_dt_s);

    Output output;
    output.setpoint_alt_m = setpoint_alt_m_;
    // Outer loop: altitude -> climb rate
    output.climb_setpoint_mps =
        std::clamp(setpoint_speed_mps_ + config_.alt_kp * (setpoint_alt_m_ - alt_m),
                   -config_.max_descent_mps, config_.max_climb_mps);
    // Inner loop: climb rate -> thrust correction around hover
    const double correction = velocity_pid_.update(output.climb_setpoint_mps, climb_mps, step_dt_s);
    output.velocity_pid_terms = velocity_pid_.terms();
    output.thrust = std::clamp(hover_thrust_ + correction, config_.thrust_min, config_.thrust_max);
    return output;
}

// Take-off phase: a fixed hover + margin (the thrust-only take-off validated in SITL; ~3 s of it
// is motor spool-up). The PID is kept reset, so nothing winds up on the ground.
AltitudeController::Output AltitudeController::takeoff_output(double alt_m)
{
    velocity_pid_.reset();
    Output output;
    output.setpoint_alt_m = alt_m;
    output.integrator_frozen = true;
    output.thrust = std::clamp(hover_thrust_ + config_.takeoff_thrust_margin, config_.thrust_min,
                               config_.thrust_max);
    return output;
}

// Trapezoidal velocity profile: speed- and acceleration-limited, braking with
// v <= sqrt(2*a*distance) so the setpoint arrives at the target with zero speed.
void AltitudeController::advance_setpoint(double target_alt_m, double dt_s)
{
    const double remaining_m = target_alt_m - setpoint_alt_m_;
    const double speed_limit_mps =
        remaining_m >= 0.0 ? config_.setpoint_speed_up_mps : config_.setpoint_speed_down_mps;
    const double braking_speed_mps =
        std::sqrt(2.0 * config_.setpoint_accel_mps2 * std::abs(remaining_m));
    const double desired_speed_mps =
        std::copysign(std::min(speed_limit_mps, braking_speed_mps), remaining_m);
    const double max_speed_change_mps = config_.setpoint_accel_mps2 * dt_s;

    const bool speeding_up = std::abs(desired_speed_mps) > std::abs(setpoint_speed_mps_) &&
                             desired_speed_mps * setpoint_speed_mps_ >= 0.0;
    if (speeding_up) {
        setpoint_speed_mps_ = std::clamp(desired_speed_mps, setpoint_speed_mps_ - max_speed_change_mps,
                                         setpoint_speed_mps_ + max_speed_change_mps);
    } else {
        setpoint_speed_mps_ = desired_speed_mps;  // braking is never limited
    }

    const double step_m = setpoint_speed_mps_ * dt_s;
    if (std::abs(step_m) >= std::abs(remaining_m)) {  // do not pass the target
        setpoint_alt_m_ = target_alt_m;
        setpoint_speed_mps_ = 0.0;
    } else {
        setpoint_alt_m_ += step_m;
    }
}

}  // namespace altctl
