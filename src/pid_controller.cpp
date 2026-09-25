#include "altctl/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace altctl {

PIDController::PIDController(const PidGains& gains) : gains_(gains) {}

void PIDController::reset()
{
    terms_ = {};
    integral_ = 0.0;
    prev_measurement_ = 0.0;
    d_filtered_ = 0.0;
    initialized_ = false;
}

double PIDController::update(double setpoint, double measurement, double dt)
{
    if (!(dt > 0.0) || !std::isfinite(setpoint) || !std::isfinite(measurement)) {
        return terms_.output;  // keep last output on a bad sample
    }
    const double error = setpoint - measurement;
    terms_.p = gains_.kp * error;

    // Derivative on measurement: no kick on setpoint steps. First-order low-pass on D.
    if (initialized_) {
        const double d_raw = -gains_.kd * (measurement - prev_measurement_) / dt;
        const double rc = 1.0 / (2.0 * M_PI * gains_.d_cutoff_hz);
        const double alpha = dt / (dt + rc);
        d_filtered_ += alpha * (d_raw - d_filtered_);
    }
    prev_measurement_ = measurement;
    initialized_ = true;
    terms_.d = d_filtered_;

    // Integrator with clamping anti-windup: skip integration if the output is already
    // saturated and the error would push it further into saturation.
    if (!integrator_frozen_) {
        const double candidate =
            std::clamp(integral_ + gains_.ki * error * dt, -gains_.i_limit, gains_.i_limit);
        const double unsat = terms_.p + candidate + terms_.d;
        const bool pushes_high = unsat > gains_.out_max && error > 0.0;
        const bool pushes_low = unsat < gains_.out_min && error < 0.0;
        if (!pushes_high && !pushes_low) {
            integral_ = candidate;
        }
    }
    terms_.i = integral_;

    terms_.output = std::clamp(terms_.p + terms_.i + terms_.d, gains_.out_min, gains_.out_max);
    return terms_.output;
}

}  // namespace altctl
