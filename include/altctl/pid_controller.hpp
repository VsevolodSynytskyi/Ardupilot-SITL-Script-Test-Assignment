#pragma once

#include "altctl/config.hpp"

namespace altctl {

// PID with derivative on measurement, low-pass filtered D, clamped integrator
// and output limits. dt is supplied by the caller (measured loop period).
class PidController {
public:
    struct Terms {
        double p = 0.0;
        double i = 0.0;
        double d = 0.0;
        double output = 0.0;
    };

    explicit PidController(const PidGains& gains);

    double update(double setpoint, double measurement, double dt_s);
    void reset();

    // While frozen, the integrator keeps its value.
    void set_integrator_frozen(bool frozen) { integrator_frozen_ = frozen; }

    [[nodiscard]] const Terms& terms() const { return terms_; }

private:
    PidGains gains_;
    Terms terms_;
    double integral_ = 0.0;  // already multiplied by ki
    double previous_measurement_ = 0.0;
    double filtered_derivative_ = 0.0;
    bool has_previous_measurement_ = false;
    bool integrator_frozen_ = false;
};

}  // namespace altctl
