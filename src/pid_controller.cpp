#include "altctl/pid_controller.hpp"

#include <algorithm>
#include <cmath>

#include "altctl/units.hpp"

namespace altctl {

PidController::PidController(const PidGains& gains) : gains_(gains) {}

void PidController::reset()
{
    terms_ = {};
    integral_ = 0.0;
    previous_measurement_ = 0.0;
    filtered_derivative_ = 0.0;
    has_previous_measurement_ = false;
}

double PidController::update(double setpoint, double measurement, double dt_s)
{
    if (!(dt_s > 0.0) || !std::isfinite(setpoint) || !std::isfinite(measurement)) {
        return terms_.output;  // keep last output on a bad sample
    }
    const double error = setpoint - measurement;
    terms_.p = gains_.kp * error;

    // Derivative on measurement: no kick on setpoint steps. First-order low-pass on D.
    if (has_previous_measurement_) {
        const double raw_derivative = -gains_.kd * (measurement - previous_measurement_) / dt_s;
        const double time_constant_s = 1.0 / (2.0 * kPi * gains_.derivative_cutoff_hz);
        const double smoothing = dt_s / (dt_s + time_constant_s);
        filtered_derivative_ += smoothing * (raw_derivative - filtered_derivative_);
    }
    previous_measurement_ = measurement;
    has_previous_measurement_ = true;
    terms_.d = filtered_derivative_;

    // Integrator with clamping anti-windup: skip integration if the output is already
    // saturated and the error would push it further into saturation.
    if (!integrator_frozen_) {
        const double candidate_integral = std::clamp(integral_ + gains_.ki * error * dt_s,
                                                     -gains_.integral_limit, gains_.integral_limit);
        const double unclamped_output = terms_.p + candidate_integral + terms_.d;
        const bool pushes_above_max = unclamped_output > gains_.output_max && error > 0.0;
        const bool pushes_below_min = unclamped_output < gains_.output_min && error < 0.0;
        if (!pushes_above_max && !pushes_below_min) {
            integral_ = candidate_integral;
        }
    }
    terms_.i = integral_;

    terms_.output =
        std::clamp(terms_.p + terms_.i + terms_.d, gains_.output_min, gains_.output_max);
    return terms_.output;
}

}  // namespace altctl
