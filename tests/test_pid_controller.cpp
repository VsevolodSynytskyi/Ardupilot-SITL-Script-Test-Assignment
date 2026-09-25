#include <gtest/gtest.h>

#include <limits>

#include "altctl/pid_controller.hpp"

using altctl::PidController;
using altctl::PidGains;

namespace {

constexpr double kDtS = 0.02;

PidGains make_gains(double kp, double ki, double kd, double integral_limit = 10.0,
                    double output_limit = 100.0)
{
    PidGains gains;
    gains.kp = kp;
    gains.ki = ki;
    gains.kd = kd;
    gains.derivative_cutoff_hz = 1000.0;  // effectively unfiltered unless a test says otherwise
    gains.integral_limit = integral_limit;
    gains.output_min = -output_limit;
    gains.output_max = output_limit;
    return gains;
}

}  // namespace

TEST(PidController, ProportionalOnly)
{
    PidController pid(make_gains(2.0, 0.0, 0.0));
    EXPECT_DOUBLE_EQ(pid.update(1.5, 1.0, kDtS), 1.0);
    EXPECT_DOUBLE_EQ(pid.terms().p, 1.0);
    EXPECT_DOUBLE_EQ(pid.terms().i, 0.0);
}

TEST(PidController, IntegralAccumulatesWithDt)
{
    PidController pid(make_gains(0.0, 0.5, 0.0));
    for (int tick = 0; tick < 100; ++tick) {
        pid.update(1.0, 0.0, kDtS);  // error 1 for 2 s
    }
    EXPECT_NEAR(pid.terms().i, 0.5 * 1.0 * 2.0, 1e-9);
}

TEST(PidController, IntegralClampedToLimit)
{
    PidController pid(make_gains(0.0, 1.0, 0.0, /*integral_limit=*/0.3));
    for (int tick = 0; tick < 1000; ++tick) {
        pid.update(1.0, 0.0, kDtS);
    }
    EXPECT_DOUBLE_EQ(pid.terms().i, 0.3);
}

TEST(PidController, AntiWindupStopsIntegratingWhenSaturated)
{
    // P alone saturates the output: the integrator must not grow in that direction...
    PidController pid(make_gains(10.0, 1.0, 0.0, /*integral_limit=*/5.0, /*output_limit=*/1.0));
    for (int tick = 0; tick < 500; ++tick) {
        pid.update(1.0, 0.0, kDtS);
    }
    EXPECT_DOUBLE_EQ(pid.terms().output, 1.0);
    EXPECT_NEAR(pid.terms().i, 0.0, 1e-12);
    // ...but it may still unwind when the error reverses.
    pid.update(0.0, 0.05, kDtS);
    EXPECT_LT(pid.terms().i, 0.0);
}

TEST(PidController, OutputLimits)
{
    PidController pid(make_gains(100.0, 0.0, 0.0, 10.0, /*output_limit=*/0.3));
    EXPECT_DOUBLE_EQ(pid.update(1.0, 0.0, kDtS), 0.3);
    EXPECT_DOUBLE_EQ(pid.update(-1.0, 0.0, kDtS), -0.3);
}

TEST(PidController, DerivativeOnMeasurementHasNoSetpointKick)
{
    PidController pid(make_gains(0.0, 0.0, 1.0));
    pid.update(0.0, 0.0, kDtS);
    pid.update(10.0, 0.0, kDtS);  // setpoint step, measurement unchanged
    EXPECT_DOUBLE_EQ(pid.terms().d, 0.0);
}

TEST(PidController, DerivativeOpposesMeasurementRate)
{
    PidController pid(make_gains(0.0, 0.0, 0.5));
    double measurement = 0.0;
    for (int tick = 0; tick < 50; ++tick) {
        pid.update(0.0, measurement, kDtS);
        measurement += 2.0 * kDtS;  // measurement rising at 2 units/s
    }
    EXPECT_NEAR(pid.terms().d, -0.5 * 2.0, 1e-9);
}

TEST(PidController, DerivativeLowPassFiltersSteps)
{
    PidGains gains = make_gains(0.0, 0.0, 1.0);
    gains.derivative_cutoff_hz = 2.0;
    PidController pid(gains);
    pid.update(0.0, 0.0, kDtS);
    pid.update(0.0, 1.0, kDtS);  // one-sample jump: raw D = -50
    const double first_derivative = pid.terms().d;
    EXPECT_LT(first_derivative, 0.0);
    EXPECT_GT(first_derivative, -50.0 * 0.25);  // strongly attenuated
}

TEST(PidController, FrozenIntegratorHoldsValue)
{
    PidController pid(make_gains(0.0, 1.0, 0.0));
    pid.update(1.0, 0.0, kDtS);
    const double held_integral = pid.terms().i;
    pid.set_integrator_frozen(true);
    for (int tick = 0; tick < 100; ++tick) {
        pid.update(1.0, 0.0, kDtS);
    }
    EXPECT_DOUBLE_EQ(pid.terms().i, held_integral);
}

TEST(PidController, ResetClearsState)
{
    PidController pid(make_gains(1.0, 1.0, 1.0));
    for (int tick = 0; tick < 10; ++tick) {
        pid.update(1.0, tick * 0.1, kDtS);
    }
    pid.reset();
    EXPECT_DOUBLE_EQ(pid.terms().i, 0.0);
    pid.update(1.0, 5.0, kDtS);  // first sample after reset: no D from the old measurement
    EXPECT_DOUBLE_EQ(pid.terms().d, 0.0);
}

TEST(PidController, BadSamplesKeepLastOutput)
{
    PidController pid(make_gains(1.0, 0.0, 0.0));
    const double last_output = pid.update(1.0, 0.0, kDtS);
    EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, 0.0), last_output);
    EXPECT_DOUBLE_EQ(pid.update(5.0, 0.0, -1.0), last_output);
    EXPECT_DOUBLE_EQ(pid.update(5.0, std::numeric_limits<double>::quiet_NaN(), kDtS), last_output);
}
