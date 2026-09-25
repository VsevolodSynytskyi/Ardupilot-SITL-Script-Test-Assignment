#pragma once

#include <algorithm>

namespace altctl::test {

// 1-D vertical copter model: m*a = T - m*g, with T = thrust * (m*g / true_hover_thrust),
// i.e. a = g * (effective_thrust / true_hover_thrust - 1). effective_thrust follows the command
// through a first-order lag (motor spool + ArduPilot throttle filtering). The ground stops descent.
struct VerticalModel {
    double true_hover_thrust = 0.36;
    double thrust_lag_s = 0.1;
    double gravity_mps2 = 9.81;
    double spool_delay_s = 0.0;  // no thrust for this long after start (ArduPilot spool-up)

    double time_s = 0.0;
    double alt_m = 0.0;
    double climb_mps = 0.0;
    double effective_thrust = 0.0;

    void step(double commanded_thrust, double dt_s)
    {
        time_s += dt_s;
        const double applied_thrust = time_s < spool_delay_s ? 0.0 : commanded_thrust;
        effective_thrust +=
            (applied_thrust - effective_thrust) * std::min(1.0, dt_s / thrust_lag_s);
        const double accel_mps2 = gravity_mps2 * (effective_thrust / true_hover_thrust - 1.0);
        climb_mps += accel_mps2 * dt_s;
        alt_m += climb_mps * dt_s;
        if (alt_m <= 0.0) {
            alt_m = 0.0;
            climb_mps = std::max(0.0, climb_mps);
        }
    }
};

}  // namespace altctl::test
