#pragma once

#include "altctl/config.hpp"

namespace altctl {

// PID with derivative on measurement, low-pass filtered D, clamped integrator
// and output limits. dt is supplied by the caller (measured loop period).
class PIDController {
public:
    struct Terms {
        double p = 0.0;
        double i = 0.0;
        double d = 0.0;
        double output = 0.0;
    };

    explicit PIDController(const PidGains& gains);

    double update(double setpoint, double measurement, double dt);
    void reset();

    // While frozen, the integrator keeps its value (used on the ground before liftoff).
    void set_integrator_frozen(bool frozen) { integrator_frozen_ = frozen; }

    const Terms& terms() const { return terms_; }
    const PidGains& gains() const { return gains_; }

private:
    PidGains gains_;
    Terms terms_;
    double integral_ = 0.0;         // already multiplied by ki
    double prev_measurement_ = 0.0;
    double d_filtered_ = 0.0;
    bool initialized_ = false;
    bool integrator_frozen_ = false;
};

}  // namespace altctl
