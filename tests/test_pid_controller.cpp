#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "altctl/pid_controller.hpp"

using altctl::PidGains;
using altctl::PIDController;

namespace {

PidGains gains(double kp, double ki, double kd, double i_limit = 10.0, double out_lim = 100.0)
{
    PidGains g;
    g.kp = kp;
    g.ki = ki;
    g.kd = kd;
    g.d_cutoff_hz = 1000.0;  // effectively unfiltered unless a test says otherwise
    g.i_limit = i_limit;
    g.out_min = -out_lim;
    g.out_max = out_lim;
    return g;
}

constexpr double kDt = 0.02;

}  // namespace

TEST(PIDController, ProportionalOnly)
{
    PIDController pid(gains(2.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.update(1.5, 1.0, kDt), 1.0);
    EXPECT_DOUBLE_EQ(pid.terms().p, 1.0);
    EXPECT_DOUBLE_EQ(pid.terms().i, 0.0);
}

TEST(PIDController, IntegralAccumulatesWithDt)
{
    PIDController pid(gains(0.0, 0.5, 0.0));
    for (int k = 0; k < 100; ++k) {
        pid.update(1.0, 0.0, kDt);  // error 1 for 2 s
    }
    EXPECT_NEAR(pid.terms().i, 0.5 * 1.0 * 2.0, 1e-9);
}

TEST(PIDController, IntegralClampedToLimit)
{
    PIDController pid(gains(0.0, 1.0, 0.0, /*i_limit=*/0.3));
    for (int k = 0; k < 1000; ++k) {
        pid.update(1.0, 0.0, kDt);
    }
    EXPECT_DOUBLE_EQ(pid.terms().i, 0.3);
}

TEST(PIDController, AntiWindupStopsIntegratingWhenSaturated)
{
    // P alone saturates the output: the integrator must not grow in that direction...
    PIDController pid(gains(10.0, 1.0, 0.0, /*i_limit=*/5.0, /*out_lim=*/1.0));
    for (int k = 0; k < 500; ++k) {
        pid.update(1.0, 0.0, kDt);
    }
    EXPECT_DOUBLE_EQ(pid.terms().output, 1.0);
    EXPECT_NEAR(pid.terms().i, 0.0, 1e-12);
    // ...but it may still unwind when the error reverses.
    pid.update(0.0, 0.05, kDt);
    EXPECT_LT(pid.terms().i, 0.0);
}

TEST(PIDController, OutputLimits)
{
    PIDController pid(gains(100.0, 0.0, 0.0, 10.0, /*out_lim=*/0.3));
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, kDt), 0.3);
    EXPECT_DOUBLE_EQ(pid.update(-1.0, 0.0, kDt), -0.3);
}

TEST(PIDController, DerivativeOnMeasurementHasNoSetpointKick)
{
    PIDController pid(gains(0.0, 0.0, 1.0));
    pid.update(0.0, 0.0, kDt);
    pid.update(10.0, 0.0, kDt);  // setpoint step, measurement unchanged
    EXPECT_DOUBLE_EQ(pid.terms().d, 0.0);
}

TEST(PIDController, DerivativeOpposesMeasurementRate)
{
    PIDController pid(gains(0.0, 0.0, 0.5));
    double y = 0.0;
    for (int k = 0; k < 50; ++k) {
        pid.update(0.0, y, kDt);
        y += 2.0 * kDt;  // measurement rising at 2 units/s
    }
    EXPECT_NEAR(pid.terms().d, -0.5 * 2.0, 1e-9);
}

TEST(PIDController, DerivativeLowPassFiltersSteps)
{
    PidGains g = gains(0.0, 0.0, 1.0);
    g.d_cutoff_hz = 2.0;
    PIDController pid(g);
    pid.update(0.0, 0.0, kDt);
    pid.update(0.0, 1.0, kDt);  // one-sample jump: raw D = -50
    const double first = pid.terms().d;
    EXPECT_LT(first, 0.0);
    EXPECT_GT(first, -50.0 * 0.25);  // strongly attenuated
}

TEST(PIDController, FrozenIntegratorHoldsValue)
{
    PIDController pid(gains(0.0, 1.0, 0.0));
    pid.update(1.0, 0.0, kDt);
    const double held = pid.terms().i;
    pid.set_integrator_frozen(true);
    for (int k = 0; k < 100; ++k) {
        pid.update(1.0, 0.0, kDt);
    }
    EXPECT_DOUBLE_EQ(pid.terms().i, held);
}

TEST(PIDController, ResetClearsState)
{
    PIDController pid(gains(1.0, 1.0, 1.0));
    for (int k = 0; k < 10; ++k) {
        pid.update(1.0, k * 0.1, kDt);
    }
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.terms().i, 0.0);
    pid.update(1.0, 5.0, kDt);  // first sample after reset: no D from the old measurement
    EXPECT_DOUBLE_EQ(pid.terms().d, 0.0);
}

TEST(PIDController, BadSamplesKeepLastOutput)
{
    PIDController pid(gains(1.0, 0.0, 0.0));
    const double out = pid.update(1.0, 0.0, kDt);
    EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, 0.0), out);
    EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, -1.0), out);
    EXPECT_DOUBLE_EQ(pid.update(5.0, std::numeric_limits<double>::quiet_NaN(), kDt), out);
}
