#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "altctl/altitude_controller.hpp"
#include "vertical_model.hpp"

using altctl::AltitudeController;
using altctl::Config;
using altctl::test::VerticalModel;

namespace {

constexpr double kDt = 0.02;

struct Trace {
    std::vector<double> alt;
    std::vector<double> thrust;
};

// Fly the model toward `target` for `duration` s, continuing from its current state.
Trace fly(AltitudeController& ctl, VerticalModel& model, double target, double duration)
{
    Trace tr;
    for (double t = 0.0; t < duration; t += kDt) {
        const auto out = ctl.update(target, model.alt_m, model.climb_ms, kDt);
        model.step(out.thrust, kDt);
        tr.alt.push_back(model.alt_m);
        tr.thrust.push_back(out.thrust);
    }
    return tr;
}

// First time after which |alt - target| stays within tol.
double settling_time(const Trace& tr, double target, double tol)
{
    for (size_t k = tr.alt.size(); k-- > 0;) {
        if (std::abs(tr.alt[k] - target) > tol) {
            return (k + 1) * kDt;
        }
    }
    return 0.0;
}

// Number of sign changes of the error over the tail of the trace: counts oscillation.
int zero_crossings(const Trace& tr, double target, size_t from)
{
    int n = 0;
    for (size_t k = from + 1; k < tr.alt.size(); ++k) {
        if ((tr.alt[k - 1] - target) * (tr.alt[k] - target) < 0.0) {
            ++n;
        }
    }
    return n;
}

}  // namespace

TEST(AltitudeController, SetpointTrajectoryRespectsRateAndAccel)
{
    Config cfg;
    AltitudeController ctl(cfg, 0.39);
    ctl.reset(0.0);
    double prev_sp = 0.0;
    double prev_v = 0.0;
    double max_v = 0.0;
    for (int k = 0; k < 2000; ++k) {
        const auto out = ctl.update(10.0, 0.0, 0.0, kDt);  // plant ignored: only the profile
        const double v = (out.alt_setpoint_m - prev_sp) / kDt;
        EXPECT_LE(v, cfg.setpoint_rate_up_ms + 1e-9);
        EXPECT_LE(out.alt_setpoint_m, 10.0 + 1e-12) << "setpoint must not pass the target";
        if (v > prev_v) {
            EXPECT_LE(v - prev_v, cfg.setpoint_accel_mss * kDt + 1e-9) << "acceleration limit";
        }
        max_v = std::max(max_v, v);
        prev_sp = out.alt_setpoint_m;
        prev_v = v;
    }
    EXPECT_DOUBLE_EQ(prev_sp, 10.0);
    EXPECT_NEAR(max_v, cfg.setpoint_rate_up_ms, 1e-9);

    ctl.reset(10.0);  // downward uses the descent rate
    prev_sp = 10.0;
    for (int k = 0; k < 2000; ++k) {
        const auto out = ctl.update(5.0, 10.0, 0.0, kDt);
        EXPECT_GE((out.alt_setpoint_m - prev_sp) / kDt, -cfg.setpoint_rate_down_ms - 1e-9);
        EXPECT_GE(out.alt_setpoint_m, 5.0 - 1e-12);
        prev_sp = out.alt_setpoint_m;
    }
    EXPECT_DOUBLE_EQ(prev_sp, 5.0);
}

TEST(AltitudeController, ClimbSetpointWithinLimits)
{
    Config cfg;
    AltitudeController ctl(cfg, 0.39);
    ctl.reset(0.0);
    for (int k = 0; k < 100; ++k) {
        const auto out = ctl.update(100.0, 0.0, 0.0, kDt);
        EXPECT_LE(out.climb_setpoint_ms, cfg.max_climb_ms + 1e-12);
    }
    ctl.reset(100.0);
    for (int k = 0; k < 100; ++k) {
        const auto out = ctl.update(0.0, 100.0, 0.0, kDt);
        EXPECT_GE(out.climb_setpoint_ms, -cfg.max_descent_ms - 1e-12);
    }
}

TEST(AltitudeController, ThrustWithinLimits)
{
    Config cfg;
    AltitudeController ctl(cfg, 0.39);
    ctl.reset(50.0);
    for (int k = 0; k < 200; ++k) {
        const auto out = ctl.update(0.0, 50.0, 10.0, kDt);  // far above, climbing fast
        EXPECT_GE(out.thrust, cfg.thrust_min);
        EXPECT_LE(out.thrust, cfg.thrust_max);
    }
}

TEST(AltitudeController, IntegratorFrozenOnGround)
{
    Config cfg;
    AltitudeController ctl(cfg, 0.39);
    ctl.reset(0.0);
    AltitudeController::Output out;
    for (int k = 0; k < 150; ++k) {  // 3 s sitting on the ground (spool-up), climb = 0
        out = ctl.update(10.0, 0.0, 0.0, kDt);
    }
    EXPECT_TRUE(out.integrator_frozen);
    EXPECT_DOUBLE_EQ(out.vel_terms.i, 0.0);
    out = ctl.update(10.0, cfg.liftoff_alt_m + 0.1, 1.0, kDt);
    EXPECT_FALSE(out.integrator_frozen);
}

// Full mission profile on the model: climb to 10 m, hold, descend to 5 m, hold.
// The controller's hover feedforward is deliberately wrong (as measured in SITL:
// MOT_THST_HOVER 0.39 vs. ~0.36 actual) so the integrator has to absorb the offset.
class CascadeOnModel : public ::testing::TestWithParam<double> {};

TEST_P(CascadeOnModel, ClimbHoldDescendHold)
{
    Config cfg;
    VerticalModel model;
    model.hover_true = GetParam();
    AltitudeController ctl(cfg, /*hover_feedforward=*/0.39);
    ctl.reset(model.alt_m);

    const auto climb = fly(ctl, model, cfg.alt_high_m, 20.0);
    const double peak = *std::max_element(climb.alt.begin(), climb.alt.end());
    const double t_settle_up = settling_time(climb, cfg.alt_high_m, cfg.settle_tolerance_m);
    EXPECT_LT(peak - cfg.alt_high_m, 0.3) << "overshoot";
    EXPECT_LT(t_settle_up, 12.0);
    const size_t tail = climb.alt.size() - static_cast<size_t>(5.0 / kDt);
    EXPECT_LE(zero_crossings(climb, cfg.alt_high_m, tail), 1) << "oscillating at hold";
    EXPECT_NEAR(climb.alt.back(), cfg.alt_high_m, 0.05);

    const auto descend = fly(ctl, model, cfg.alt_low_m, 20.0);
    const double trough = *std::min_element(descend.alt.begin(), descend.alt.end());
    const double t_settle_dn = settling_time(descend, cfg.alt_low_m, cfg.settle_tolerance_m);
    EXPECT_LT(cfg.alt_low_m - trough, 0.3) << "undershoot";
    EXPECT_LT(t_settle_dn, 12.0);
    EXPECT_NEAR(descend.alt.back(), cfg.alt_low_m, 0.05);

    // In steady hover the thrust converges to the true hover value.
    EXPECT_NEAR(descend.thrust.back(), model.hover_true, 0.01);

    std::printf("    hover_true=%.2f: peak=%.2f m, settle up %.1f s, trough=%.2f m, "
                "settle down %.1f s\n",
                model.hover_true, peak, t_settle_up, trough, t_settle_dn);
}

INSTANTIATE_TEST_SUITE_P(HoverMismatch, CascadeOnModel, ::testing::Values(0.33, 0.36, 0.39, 0.42));
