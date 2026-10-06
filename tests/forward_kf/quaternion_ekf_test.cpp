#include "pyro_ins_attitude.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace pyro;
static void check(bool passed, const char *message)
{
    if (!passed)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}
static void finite_output(const ins_attitude_t &filter)
{
    for (float value : filter.ekf.get_state())
        check(matrix::is_finite(value), "finite QEKF state");
    for (float value : filter.ekf.get_covariance())
        check(matrix::is_finite(value), "finite QEKF covariance");
    check(matrix::is_finite(filter.Roll) && matrix::is_finite(filter.Pitch) &&
              matrix::is_finite(filter.Yaw),
          "finite angles");
}
int main()
{
    ins_attitude_t filter, peer, fresh;
    check(filter.init(10, 0.001f, 1000000, 0.9996f, 0.01f) == PYRO_OK,
          "QEKF init");
    check(peer.init(10, 0.001f, 1000000, 0.9996f, 0.01f) == PYRO_OK,
          "independent init");
    const auto p0   = filter.ekf.get_covariance();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    for (int i = 0; i < 1000; ++i)
        filter.update(0.2f, -0.1f, 0.3f, 0, 0, 9.81f, 0.001f);
    check(peer.UpdateCount == 0 && peer.ekf.get_covariance() == p0,
          "interleaved instances have no global state");
    const auto state = filter.ekf.get_state();
    check(filter.update(nan, 0, 0, 0, 0, 9.81f, 0.001f) == PYRO_PARAM_ERROR &&
              filter.ekf.get_state() == state,
          "NaN gyro holds state");
    check(filter.update(0, 0, 0, 0, 0, 9.81f, 0) == PYRO_PARAM_ERROR &&
              filter.ekf.get_state() == state,
          "zero dt holds state");
    check(filter.update(0, 0, 0, 0, 0, 9.81f, inf) == PYRO_PARAM_ERROR,
          "infinite dt rejected");
    check(filter.init(10, 0.001f, 1000000, 0, 0.01f) == PYRO_PARAM_ERROR,
          "invalid fading rejected without clearing filter");
    check(filter.update(0, 0, 0, nan, 0, 9.81f, 0.001f) == PYRO_WARNING,
          "NaN accel predicts only");
    check(filter.ekf.get_diagnostics().result ==
              ins_attitude_t::filter_t::result_t::NO_MEASUREMENT,
          "missing accel diagnostics");
    finite_output(filter);
    filter.update(0, 0, 0, 0, 0, 9.81f, 0.001f);
    finite_output(filter);
    check(filter.get_status() != PYRO_PARAM_ERROR &&
              filter.get_status() != PYRO_ERROR,
          "valid accel recovers after NaN");
    check(filter.update(0, 0, 0, 0, 0, 0, 0.001f) == PYRO_WARNING,
          "zero raw accel cannot reuse a valid LP history as gravity");
    finite_output(filter);
    fresh.init(10, 0.001f, 1000000, 1, 0);
    check(fresh.update(0, 0, 0, 0, 0, 0, 0.001f) == PYRO_WARNING,
          "zero acceleration cannot define gravity direction");
    finite_output(fresh);
    fresh.update(0, 0, 0, 0, 0, 9.81f, 0.001f);
    finite_output(fresh);

    // Force an actually singular innovation through the new shared core.
    fresh.init(0, 0, 0, 1, 0);
    ins_attitude_t::filter_t::covariance_t zero{};
    fresh.ekf.reset({1, 0, 0, 0, 0, 0}, zero);
    check(fresh.update(0, 0, 0, 0, 0, 9.81f, 0.001f) == PYRO_ERROR &&
              fresh.ekf.get_diagnostics().math_status == ARM_MATH_SINGULAR,
          "QEKF inverse failure is reported");
    finite_output(fresh);
    fresh.R = 1;
    check(fresh.update(0, 0, 0, 0, 0, 9.81f, 0.001f) == PYRO_OK,
          "QEKF inverse failure recovers");

    // Re-init resets covariance, bias, angles, counters and LP history.
    filter.init(10, 0.001f, 1000000, 0.9996f, 0.01f);
    fresh.init(10, 0.001f, 1000000, 0.9996f, 0.01f);
    check(filter.UpdateCount == 0 && filter.GyroBias[0] == 0 &&
              filter.YawRoundCount == 0 && filter.ekf.get_covariance() == p0,
          "repeatable reset defaults");
    for (int i = 0; i < 500; ++i)
    {
        const float gyro = 0.1f * std::sin(i * 0.01f);
        filter.update(gyro, 0, 0, 0, 0, 9.81f, 0.001f);
        peer.update(-gyro, 0.1f, 0.2f, 1, 0, 9.81f, 0.001f);
        fresh.update(gyro, 0, 0, 0, 0, 9.81f, 0.001f);
        check(filter.ekf.get_state() == fresh.ekf.get_state() &&
                  filter.ekf.get_covariance() == fresh.ekf.get_covariance(),
              "instances remain independent under interleaving");
    }
    const auto f = filter.get_transition();
    const auto h = filter.get_observation();
    filter.update(0.4f, 0.2f, 0, 0, 0, 9.81f, 0.002f);
    check(filter.get_transition() != f && filter.get_observation() != h,
          "external F/H evolve every sample");
    finite_output(filter);
    ins_attitude_t ins;
    ins.init(10, 0.001f, 1000000, 1, 0);
    ins.update(0, 0, 0, 0, 0, 9.81f, 0.001f);
    check(ins.Initialized && ins.UpdateCount == 1,
          "INS-owned attitude adapter");
    std::puts("PASS: QEKF instances, reset, NaN/zero inputs, singular solve "
              "and recovery");
}
