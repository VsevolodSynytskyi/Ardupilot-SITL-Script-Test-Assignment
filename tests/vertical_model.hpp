#pragma once

#include <algorithm>

namespace altctl::test {

// 1-D vertical copter model: m*a = T - m*g, with T = thrust * (m*g / hover_true),
// i.e. a = g * (thrust_eff / hover_true - 1). thrust_eff follows the command through a
// first-order lag (motor spool + ArduPilot throttle filtering). The ground stops descent.
struct VerticalModel {
    double hover_true = 0.36;
    double lag_tau_s = 0.1;
    double g = 9.81;

    double alt_m = 0.0;
    double climb_ms = 0.0;
    double thrust_eff = 0.0;

    void step(double thrust_cmd, double dt)
    {
        thrust_eff += (thrust_cmd - thrust_eff) * std::min(1.0, dt / lag_tau_s);
        const double accel = g * (thrust_eff / hover_true - 1.0);
        climb_ms += accel * dt;
        alt_m += climb_ms * dt;
        if (alt_m <= 0.0) {
            alt_m = 0.0;
            climb_ms = std::max(0.0, climb_ms);
        }
    }
};

}  // namespace altctl::test
