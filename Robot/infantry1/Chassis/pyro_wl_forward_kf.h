#ifndef __PYRO_WL_FORWARD_KF_H__
#define __PYRO_WL_FORWARD_KF_H__

#include "kalman_filter.h"

namespace pyro
{

struct ForwardStateEstimate
{
    float x;
    float v;
    float a;
    float imu_acc_bias;
    float slip_score;
};

class wl_forward_kf_t
{
  public:
    wl_forward_kf_t() = default;
    wl_forward_kf_t(const wl_forward_kf_t &) = delete;
    wl_forward_kf_t &operator=(const wl_forward_kf_t &) = delete;

    void init(); // allocate once through the existing KF library
    ForwardStateEstimate reset(float x, float v_wheel, float a_imu,
                               bool wheel_valid = true);
    ForwardStateEstimate update(float a_imu, float a_model, float v_wheel,
                                bool wheel_valid = true);
    const KalmanFilter_t &get_filter() const { return _kf; }

  private:
    static void _adjust_wheel_r(KalmanFilter_t *kf);
    ForwardStateEstimate _get_estimate() const;

    KalmanFilter_t _kf{};
    float _previous_wheel_v = 0.0f;
    float _wheel_accel = 0.0f;
    float _slip_score = 0.0f;
    bool _wheel_valid = false;
    bool _wheel_was_valid = false;
};

} // namespace pyro
#endif // __PYRO_WL_FORWARD_KF_H__
