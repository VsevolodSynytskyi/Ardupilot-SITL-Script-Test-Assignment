#include "altctl/altitude_controller.hpp"

#include <algorithm>
#include <cmath>

namespace altctl {

AltitudeController::AltitudeController(const Config& cfg, double hover_thrust)
    : cfg_(cfg), hover_thrust_(hover_thrust), vel_pid_(cfg.vel)
{
}

void AltitudeController::reset(double current_alt_m)
{
    ramped_setpoint_m_ = current_alt_m;
    ramp_rate_ms_ = 0.0;
    vel_pid_.reset();
}

AltitudeController::Output AltitudeController::update(double target_alt_m, double alt_m,
                                                      double climb_ms, double dt)
{
    Output out;
    if (!(dt > 0.0)) {
        dt = 0.0;
    }

    // Setpoint trajectory: trapezoidal velocity profile (rate- and acceleration-limited, and
    // braking with v <= sqrt(2*a*distance) so it arrives at the target with zero velocity).
    // The profile velocity is fed forward so the outer loop does not lag the ramp.
    const double remaining = target_alt_m - ramped_setpoint_m_;
    const double rate_limit = remaining >= 0.0 ? cfg_.setpoint_rate_up_ms : cfg_.setpoint_rate_down_ms;
    const double v_brake = std::sqrt(2.0 * cfg_.setpoint_accel_mss * std::abs(remaining));
    const double v_wanted = std::copysign(std::min(rate_limit, v_brake), remaining);
    const double dv = cfg_.setpoint_accel_mss * dt;
    // accelerate at most `dv` per tick; braking toward v_wanted is always allowed
    if (std::abs(v_wanted) > std::abs(ramp_rate_ms_) && v_wanted * ramp_rate_ms_ >= 0.0) {
        ramp_rate_ms_ = std::clamp(v_wanted, ramp_rate_ms_ - dv, ramp_rate_ms_ + dv);
    } else {
        ramp_rate_ms_ = v_wanted;
    }
    double step = ramp_rate_ms_ * dt;
    if (std::abs(step) >= std::abs(remaining)) {  // do not pass the target
        step = remaining;
        ramp_rate_ms_ = 0.0;
    }
    ramped_setpoint_m_ += step;
    // On the ground (motors spooling, ~3 s in SITL) the vehicle cannot follow yet: keep the
    // setpoint just above it instead of letting it run metres ahead and cause a lunge at liftoff.
    if (alt_m < cfg_.liftoff_alt_m) {
        ramped_setpoint_m_ = std::min(ramped_setpoint_m_, alt_m + cfg_.liftoff_alt_m);
    }
    const double ramp_rate = ramp_rate_ms_;
    out.alt_setpoint_m = ramped_setpoint_m_;

    // Outer loop: altitude -> climb rate
    out.climb_setpoint_ms =
        std::clamp(ramp_rate + cfg_.alt_kp * (ramped_setpoint_m_ - alt_m), -cfg_.max_descent_ms,
                   cfg_.max_climb_ms);

    // Inner loop: climb rate -> thrust correction around hover
    out.integrator_frozen = alt_m < cfg_.liftoff_alt_m;
    vel_pid_.set_integrator_frozen(out.integrator_frozen);
    const double correction = vel_pid_.update(out.climb_setpoint_ms, climb_ms, dt);
    out.vel_terms = vel_pid_.terms();
    out.thrust = std::clamp(hover_thrust_ + correction, cfg_.thrust_min, cfg_.thrust_max);
    return out;
}

}  // namespace altctl
