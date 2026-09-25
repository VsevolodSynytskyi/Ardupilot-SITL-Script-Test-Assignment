#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "altctl/altitude_controller.hpp"
#include "vertical_model.hpp"

using altctl::AltitudeController;
using altctl::Config;
using altctl::test::VerticalModel;

namespace {

constexpr double kDtS = 0.02;
constexpr double kHoverFeedforward = 0.39;  // MOT_THST_HOVER default

struct FlightTrace {
    std::vector<double> alt_m;
    std::vector<double> thrust;
};

// Fly the model toward `target_alt_m` for `duration_s`, continuing from its current state.
FlightTrace fly(AltitudeController& controller, VerticalModel& model, double target_alt_m,
                double duration_s)
{
    FlightTrace trace;
    for (double time_s = 0.0; time_s < duration_s; time_s += kDtS) {
        const auto output = controller.update(target_alt_m, model.alt_m, model.climb_mps, kDtS);
        model.step(output.thrust, kDtS);
        trace.alt_m.push_back(model.alt_m);
        trace.thrust.push_back(output.thrust);
    }
    return trace;
}

// First time after which |alt - target| stays within the tolerance.
double settling_time_s(const FlightTrace& trace, double target_alt_m, double tolerance_m)
{
    for (size_t sample = trace.alt_m.size(); sample-- > 0;) {
        if (std::abs(trace.alt_m[sample] - target_alt_m) > tolerance_m) {
            return (sample + 1) * kDtS;
        }
    }
    return 0.0;
}

// Number of sign changes of the error from `first_sample` on: counts oscillation.
int count_zero_crossings(const FlightTrace& trace, double target_alt_m, size_t first_sample)
{
    int crossings = 0;
    for (size_t sample = first_sample + 1; sample < trace.alt_m.size(); ++sample) {
        if ((trace.alt_m[sample - 1] - target_alt_m) * (trace.alt_m[sample] - target_alt_m) < 0.0) {
            ++crossings;
        }
    }
    return crossings;
}

}  // namespace

TEST(AltitudeController, SetpointTrajectoryRespectsSpeedAndAccel)
{
    Config config;
    config.liftoff_alt_m = -1.0;  // profile only: skip the take-off phase
    AltitudeController controller(config, kHoverFeedforward);
    controller.reset(0.0);
    double previous_setpoint_m = 0.0;
    double previous_speed_mps = 0.0;
    double max_speed_mps = 0.0;
    for (int tick = 0; tick < 2000; ++tick) {
        const auto output = controller.update(10.0, 0.0, 0.0, kDtS);  // plant ignored
        const double speed_mps = (output.setpoint_alt_m - previous_setpoint_m) / kDtS;
        EXPECT_LE(speed_mps, config.setpoint_speed_up_mps + 1e-9);
        EXPECT_LE(output.setpoint_alt_m, 10.0 + 1e-12) << "setpoint must not pass the target";
        if (speed_mps > previous_speed_mps) {
            EXPECT_LE(speed_mps - previous_speed_mps, config.setpoint_accel_mps2 * kDtS + 1e-9)
                << "acceleration limit";
        }
        max_speed_mps = std::max(max_speed_mps, speed_mps);
        previous_setpoint_m = output.setpoint_alt_m;
        previous_speed_mps = speed_mps;
    }
    EXPECT_DOUBLE_EQ(previous_setpoint_m, 10.0);
    EXPECT_NEAR(max_speed_mps, config.setpoint_speed_up_mps, 1e-9);

    controller.reset(10.0);  // downward uses the descent speed
    previous_setpoint_m = 10.0;
    for (int tick = 0; tick < 2000; ++tick) {
        const auto output = controller.update(5.0, 10.0, 0.0, kDtS);
        EXPECT_GE((output.setpoint_alt_m - previous_setpoint_m) / kDtS,
                  -config.setpoint_speed_down_mps - 1e-9);
        EXPECT_GE(output.setpoint_alt_m, 5.0 - 1e-12);
        previous_setpoint_m = output.setpoint_alt_m;
    }
    EXPECT_DOUBLE_EQ(previous_setpoint_m, 5.0);
}

TEST(AltitudeController, ClimbSetpointWithinLimits)
{
    Config config;
    AltitudeController controller(config, kHoverFeedforward);
    controller.reset(0.0);
    for (int tick = 0; tick < 100; ++tick) {
        const auto output = controller.update(100.0, 0.0, 0.0, kDtS);
        EXPECT_LE(output.climb_setpoint_mps, config.max_climb_mps + 1e-12);
    }
    controller.reset(100.0);
    for (int tick = 0; tick < 100; ++tick) {
        const auto output = controller.update(0.0, 100.0, 0.0, kDtS);
        EXPECT_GE(output.climb_setpoint_mps, -config.max_descent_mps - 1e-12);
    }
}

TEST(AltitudeController, ThrustWithinLimits)
{
    Config config;
    AltitudeController controller(config, kHoverFeedforward);
    controller.reset(50.0);
    for (int tick = 0; tick < 200; ++tick) {
        const auto output = controller.update(0.0, 50.0, 10.0, kDtS);  // far above, climbing fast
        EXPECT_GE(output.thrust, config.thrust_min);
        EXPECT_LE(output.thrust, config.thrust_max);
    }
}

TEST(AltitudeController, TakeoffPhaseOnGround)
{
    Config config;
    const double ground_alt_m = 0.2;  // EKF altitude of the ground is not necessarily 0
    AltitudeController controller(config, kHoverFeedforward);
    controller.reset(ground_alt_m);
    AltitudeController::Output output;
    for (int tick = 0; tick < 150; ++tick) {  // 3 s sitting on the ground (spool-up)
        output = controller.update(ground_alt_m + 10.0, ground_alt_m, 0.0, kDtS);
        EXPECT_DOUBLE_EQ(output.thrust, kHoverFeedforward + config.takeoff_thrust_margin);
    }
    EXPECT_TRUE(output.integrator_frozen);
    EXPECT_DOUBLE_EQ(output.velocity_pid_terms.i, 0.0);
    // just below the liftoff height (relative to the ground reference): still take-off phase
    output = controller.update(ground_alt_m + 10.0, ground_alt_m + config.liftoff_alt_m - 0.01,
                               0.5, kDtS);
    EXPECT_TRUE(output.integrator_frozen);
}

TEST(AltitudeController, BumplessHandoverAtLiftoff)
{
    Config config;
    AltitudeController controller(config, kHoverFeedforward);
    controller.reset(0.0);
    const double liftoff_alt_m = config.liftoff_alt_m + 0.01;
    const double liftoff_climb_mps = 1.0;
    const auto output = controller.update(10.0, liftoff_alt_m, liftoff_climb_mps, kDtS);
    EXPECT_FALSE(output.integrator_frozen);
    EXPECT_NEAR(output.setpoint_alt_m, liftoff_alt_m, 2 * liftoff_climb_mps * kDtS);
    EXPECT_NEAR(output.climb_setpoint_mps, liftoff_climb_mps, 0.1);
    EXPECT_NEAR(output.thrust, kHoverFeedforward, 0.05);
    // latched: dipping below the liftoff height does not re-enter the take-off phase
    EXPECT_FALSE(controller.update(10.0, 0.1, 0.0, kDtS).integrator_frozen);
}

// Full mission profile on the model: climb to 10 m, hold, descend to 5 m, hold.
// The controller's hover feedforward is deliberately wrong (as measured in SITL:
// MOT_THST_HOVER 0.39 vs. ~0.36 actual) so the integrator has to absorb the offset.
class CascadeOnModel : public ::testing::TestWithParam<double> {};

TEST_P(CascadeOnModel, ClimbHoldDescendHold)
{
    Config config;
    VerticalModel model;
    model.true_hover_thrust = GetParam();
    model.spool_delay_s = 3.0;  // as measured in SITL: liftoff ~3 s after arming
    AltitudeController controller(config, kHoverFeedforward);
    controller.reset(model.alt_m);

    const auto climb = fly(controller, model, config.alt_high_m, 20.0);
    const double peak_alt_m = *std::max_element(climb.alt_m.begin(), climb.alt_m.end());
    const double climb_settling_s =
        settling_time_s(climb, config.alt_high_m, config.settle_tolerance_m);
    EXPECT_LT(peak_alt_m - config.alt_high_m, 0.3) << "overshoot";
    EXPECT_LT(climb_settling_s, 15.0);  // includes the 3 s spool-up
    const size_t last_five_seconds = climb.alt_m.size() - static_cast<size_t>(5.0 / kDtS);
    EXPECT_LE(count_zero_crossings(climb, config.alt_high_m, last_five_seconds), 1)
        << "oscillating at hold";
    EXPECT_NEAR(climb.alt_m.back(), config.alt_high_m, 0.05);

    const auto descent = fly(controller, model, config.alt_low_m, 20.0);
    const double lowest_alt_m = *std::min_element(descent.alt_m.begin(), descent.alt_m.end());
    const double descent_settling_s =
        settling_time_s(descent, config.alt_low_m, config.settle_tolerance_m);
    EXPECT_LT(config.alt_low_m - lowest_alt_m, 0.3) << "undershoot";
    EXPECT_LT(descent_settling_s, 12.0);
    EXPECT_NEAR(descent.alt_m.back(), config.alt_low_m, 0.05);

    // In steady hover the thrust converges to the true hover value.
    EXPECT_NEAR(descent.thrust.back(), model.true_hover_thrust, 0.01);

    std::printf("    true hover %.2f: peak %.2f m, settle up %.1f s, lowest %.2f m, "
                "settle down %.1f s\n",
                model.true_hover_thrust, peak_alt_m, climb_settling_s, lowest_alt_m,
                descent_settling_s);
}

INSTANTIATE_TEST_SUITE_P(HoverMismatch, CascadeOnModel, ::testing::Values(0.33, 0.36, 0.39, 0.42));
