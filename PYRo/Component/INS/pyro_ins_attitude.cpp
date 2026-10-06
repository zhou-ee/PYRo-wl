/**
 * @file pyro_ins_attitude.cpp
 * @brief INS 层四元数模型、动态雅可比和观测门控实现。
 *        INS quaternion model, dynamic Jacobians and measurement gating.
 */
#include "pyro_ins_attitude.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace pyro
{
namespace
{

// 保留原快速平方根倒数近似；使用 memcpy 避免类型别名及 long 长度问题。
// Preserve inverse-square-root approximation without type-punning/long-size
// issues.
float inverse_sqrt(float value)
{
    const float half = 0.5f * value;
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    bits = 0x5f375a86U - (bits >> 1);
    float result;
    std::memcpy(&result, &bits, sizeof(result));
    return result * (1.5f - half * result * result);
}

// 初始 6×6 状态协方差 P0，顺序为 [qw,qx,qy,qz,bias_x,bias_y]。
// Initial 6×6 covariance P0 in [qw,qx,qy,qz,bias_x,bias_y] order.
constexpr ins_attitude_t::filter_t::covariance_t INITIAL_COVARIANCE = {
    100000, 0.1f, 0.1f, 0.1f,   0.1f, 0.1f,   0.1f, 100000, 0.1f,
    0.1f,   0.1f, 0.1f, 0.1f,   0.1f, 100000, 0.1f, 0.1f,   0.1f,
    0.1f,   0.1f, 0.1f, 100000, 0.1f, 0.1f,   0.1f, 0.1f,   0.1f,
    0.1f,   100,  0.1f, 0.1f,   0.1f, 0.1f,   0.1f, 0.1f,   100};

} // namespace

bool ins_attitude_t::_valid_config(float q_noise, float bias_noise,
                                   float r_noise, float fading, float accel_lpf)
{
    return matrix::is_finite(q_noise) && q_noise >= 0.0f &&
           matrix::is_finite(bias_noise) && bias_noise >= 0.0f &&
           matrix::is_finite(r_noise) && r_noise >= 0.0f &&
           matrix::is_finite(fading) && fading > 0.0f && fading <= 1.0f &&
           matrix::is_finite(accel_lpf) && accel_lpf >= 0.0f;
}

status_t ins_attitude_t::init(float q_noise, float bias_noise, float r_noise,
                              float fading, float accel_lpf)
{
    if (matrix::is_finite(fading) && fading > 1.0f)
        // 兼容原接口的上限截断。 / Preserve the legacy upper clamp.
        fading = 1.0f;
    if (!_valid_config(q_noise, bias_noise, r_noise, fading, accel_lpf))
        return _status = PYRO_PARAM_ERROR;

    const filter_t::state_t x0 = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    filter_t::options_t options;
    // 首阶段迁移使用原协方差公式和求逆路径，保持正常输入下的数值行为。
    // Preserve normal-input behavior with the original covariance/inverse
    // paths.
    options.covariance_update = filter_t::covariance_update_t::SIMPLIFIED;
    options.solver            = filter_t::solver_t::INVERSE;
    options.symmetrize        = false;
    _status = ekf.is_initialized() ? ekf.reset(x0, INITIAL_COVARIANCE)
                                   : ekf.init(x0, INITIAL_COVARIANCE, options);
    if (_status != PYRO_OK)
        return _status;

    Q1                     = q_noise;
    Q2                     = bias_noise;
    R                      = r_noise;
    lambda                 = fading;
    accLPFcoef             = accel_lpf;
    ChiSquareTestThreshold = 1.0e-8f;
    ConvergeFlag = StableFlag = 0;
    ErrorCount = UpdateCount = 0;
    AdaptiveGainScale        = 1.0f;
    gyro_norm = accl_norm = dt = 0.0f;
    Roll = Pitch = Yaw = YawTotalAngle = YawAngleLast = 0.0f;
    YawRoundCount                                     = 0;
    ChiSquare_Data[0]                                 = 0.0f;
    std::fill_n(GyroBias, 3, 0.0f);
    std::fill_n(Gyro, 3, 0.0f);
    std::fill_n(Accel, 3, 0.0f);
    std::fill_n(OrientationCosine, 3, 0.0f);
    std::copy_n(x0.data(), 4, q);
    F_.fill(0.0f);
    for (std::size_t i = 0; i < 6; ++i)
        F_[i * 6 + i] = 1.0f;
    H_.fill(0.0f);
    Q_d_.fill(0.0f);
    R_.fill(0.0f);
    z_.fill(0.0f);
    z_pred_.fill(0.0f);
    _accel_initialized = false;
    Initialized        = 1;
    return _status;
}

status_t ins_attitude_t::update(float gx, float gy, float gz, float ax,
                                float ay, float az, float dt_s)
{
    if (!Initialized && init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f) != PYRO_OK)
        return _status;
    if (!matrix::is_finite(dt_s) || dt_s <= 0.0f || !matrix::is_finite(gx) ||
        !matrix::is_finite(gy) || !matrix::is_finite(gz) ||
        !_valid_config(Q1, Q2, R, lambda, accLPFcoef))
        return _status = PYRO_PARAM_ERROR;

    dt      = dt_s;
    Gyro[0] = gx - GyroBias[0];
    Gyro[1] = gy - GyroBias[1];
    Gyro[2] = gz - GyroBias[2];
    const float gyro_squared =
        Gyro[0] * Gyro[0] + Gyro[1] * Gyro[1] + Gyro[2] * Gyro[2];
    if (!matrix::is_finite(gyro_squared))
        return _status = PYRO_PARAM_ERROR;
    gyro_norm                     = 1.0f / inverse_sqrt(gyro_squared);

    // 先检查原始加速度，避免异常样本污染低通历史。
    // Validate raw acceleration before it can contaminate the LPF history.
    float next_accel[3]           = {ax, ay, az};
    const float raw_accel_squared = ax * ax + ay * ay + az * az;
    bool accel_valid = matrix::is_finite(ax) && matrix::is_finite(ay) &&
                       matrix::is_finite(az) &&
                       matrix::is_finite(raw_accel_squared) &&
                       raw_accel_squared > 0.0f;
    if (accel_valid && _accel_initialized)
        for (std::size_t i = 0; i < 3; ++i)
            next_accel[i] = Accel[i] * accLPFcoef / (dt + accLPFcoef) +
                            next_accel[i] * dt / (dt + accLPFcoef);
    else if (accel_valid)
        // 保留原低通滤波首样本的运算顺序。
        // Preserve first-sample arithmetic of the original low-pass filter.
        for (std::size_t i = 0; i < 3; ++i)
            next_accel[i] = next_accel[i] * accLPFcoef / (dt + accLPFcoef) +
                            next_accel[i] * dt / (dt + accLPFcoef);
    const float accel_squared = next_accel[0] * next_accel[0] +
                                next_accel[1] * next_accel[1] +
                                next_accel[2] * next_accel[2];
    accel_valid =
        accel_valid && matrix::is_finite(accel_squared) && accel_squared > 0.0f;
    StableFlag = 0;
    if (accel_valid)
    {
        // 观测为加速度方向 z=a/|a|；模长仅用于稳定性判断。
        // Observe direction z=a/|a|; use acceleration norm only for stability.
        const float a_inv_norm = inverse_sqrt(accel_squared);
        std::copy_n(next_accel, 3, Accel);
        _accel_initialized = true;
        for (std::size_t i = 0; i < 3; ++i)
            z_[i] = Accel[i] * a_inv_norm;
        accl_norm  = 1.0f / a_inv_norm;
        StableFlag = gyro_norm < 0.3f && accl_norm > 9.8f - 0.5f &&
                     accl_norm < 9.8f + 0.5f;
    }
    else
        accl_norm = 0.0f;

    _status = _predict();
    if (_status != PYRO_OK)
        return _status;
    _status = accel_valid ? _correct() : PYRO_WARNING;
    if (!accel_valid)
    {
        // 全 false 掩码报告无观测，保留已完成的陀螺预测。
        // An all-false mask reports no measurement and retains gyro prediction.
        filter_t::measurement_mask_t mask{};
        ekf.correct(z_, z_pred_, H_, R_, &mask);
        ChiSquare_Data[0] = 0.0f;
    }
    _publish();
    ++UpdateCount;
    return _status;
}

status_t ins_attitude_t::_predict()
{
    // 先构造陀螺积分矩阵，按当前去零偏角速度和 dt 传播四元数。
    // Build the gyro integration matrix from bias-corrected rates and dt.
    F_.fill(0.0f);
    for (std::size_t i = 0; i < 6; ++i)
        F_[i * 6 + i] = 1.0f;
    const float half_dx = 0.5f * Gyro[0] * dt;
    const float half_dy = 0.5f * Gyro[1] * dt;
    const float half_dz = 0.5f * Gyro[2] * dt;
    F_[1]               = -half_dx;
    F_[2]               = -half_dy;
    F_[3]               = -half_dz;
    F_[6]               = half_dx;
    F_[8]               = half_dz;
    F_[9]               = -half_dy;
    F_[12]              = half_dy;
    F_[13]              = -half_dz;
    F_[15]              = half_dx;
    F_[18]              = half_dz;
    F_[19]              = half_dy;
    F_[20]              = -half_dx;

    filter_t::state_t x_pred{};
    arm_matrix_instance_f32 F_desc, x_desc, out_desc;
    arm_mat_init_f32(&F_desc, 6, 6, F_.data());
    arm_mat_init_f32(&x_desc, 6, 1,
                     const_cast<float *>(ekf.get_state().data()));
    arm_mat_init_f32(&out_desc, 6, 1, x_pred.data());
    if (arm_mat_mult_f32(&F_desc, &x_desc, &out_desc) != ARM_MATH_SUCCESS)
        return PYRO_ERROR;
    const float qw = x_pred[0], qx = x_pred[1];
    const float qy = x_pred[2], qz = x_pred[3];
    const float q_norm_sq = qw * qw + qx * qx + qy * qy + qz * qz;
    if (!matrix::is_finite(q_norm_sq) || q_norm_sq <= 0.0f)
        return PYRO_PARAM_ERROR;
    const float q_inv_norm = inverse_sqrt(q_norm_sq);
    for (std::size_t i = 0; i < 4; ++i)
        x_pred[i] *= q_inv_norm;

    // 补充零偏耦合项形成状态雅可比；沿用归一化前的线性化点。
    // Add bias coupling to form F, retaining pre-normalization linearization.
    F_[4]        = qx * dt / 2;
    F_[5]        = qy * dt / 2;
    F_[10]       = -qw * dt / 2;
    F_[11]       = qz * dt / 2;
    F_[16]       = -qz * dt / 2;
    F_[17]       = -qw * dt / 2;
    F_[22]       = qy * dt / 2;
    F_[23]       = -qx * dt / 2;
    // 仅对 XY 零偏方差施加遗忘因子，再传入本次预测。
    // Fade only XY bias variances and supply this covariance to prediction.
    auto P_faded = ekf.get_covariance();
    P_faded[28]  = std::min(P_faded[28] / lambda, 10000.0f);
    P_faded[35]  = std::min(P_faded[35] / lambda, 10000.0f);
    // 将噪声强度离散化：四元数 Q1·dt，XY 零偏 Q2·dt。
    // Discretize noise intensities: quaternion Q1·dt, XY bias Q2·dt.
    for (std::size_t i = 0; i < 4; ++i)
        Q_d_[i * 6 + i] = Q1 * dt;
    Q_d_[28] = Q_d_[35] = Q2 * dt;
    return ekf.predict(x_pred, F_, Q_d_, &P_faded);
}

status_t ins_attitude_t::_correct()
{
    const auto &state = ekf.get_state();
    const float qw = state[0], qx = state[1], qy = state[2], qz = state[3];
    const float qw2 = 2 * qw, qx2 = 2 * qx, qy2 = 2 * qy, qz2 = 2 * qz;
    // h(x) 为机体系中的预测重力方向；H=∂h/∂x，随预测四元数更新。
    // h(x) is predicted body-frame gravity; H=∂h/∂x varies with quaternion.
    H_      = {-qy2, qz2, -qw2, qx2, 0,    0,    qx2, qw2, qz2,
               qy2,  0,   0,    qw2, -qx2, -qy2, qz2, 0,   0};
    z_pred_ = {2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz),
               qw * qw - qx * qx - qy * qy + qz * qz};
    for (std::size_t i = 0; i < 3; ++i)
    {
        OrientationCosine[i] =
            std::acos(std::clamp(std::fabs(z_pred_[i]), 0.0f, 1.0f));
        R_[i * 3 + i] = R;
    }
    return ekf.correct(z_, z_pred_, H_, R_, nullptr,
                       [this](filter_t::correction_t &correction)
                       { return _gate(correction); });
}

status_t ins_attitude_t::_gate(filter_t::correction_t &correction)
{
    ChiSquare_Data[0] = correction.diagnostics.nis;
    // 先准备 y/S/K/NIS，再由模型决定拒绝、恢复或调整增益。
    // Prepare y/S/K/NIS first; the model then rejects, recovers or scales gain.
    if (ChiSquare_Data[0] < 0.5f * ChiSquareTestThreshold)
        ConvergeFlag = 1;
    if (ChiSquare_Data[0] > ChiSquareTestThreshold && ConvergeFlag)
    {
        ErrorCount = StableFlag ? ErrorCount + 1 : 0;
        if (ErrorCount > 50)
            ConvergeFlag = 0;
        else
            return PYRO_WARNING;
    }
    else
    {
        AdaptiveGainScale =
            ChiSquare_Data[0] > 0.1f * ChiSquareTestThreshold && ConvergeFlag
                ? (ChiSquareTestThreshold - ChiSquare_Data[0]) /
                      (0.9f * ChiSquareTestThreshold)
                : 1.0f;
        ErrorCount = 0;
    }

    // 保留原自适应增益与姿态可观测性相关的 XY 零偏行缩放。
    // Retain adaptive gain and orientation-dependent scaling of XY bias rows.
    auto &K = correction.K;
    for (float &value : K)
        value *= AdaptiveGainScale;
    for (std::size_t row = 4; row < 6; ++row)
        for (std::size_t col = 0; col < 3; ++col)
            K[row * 3 + col] *= OrientationCosine[row - 4] / 1.5707963f;
    auto &dx = correction.dx;
    arm_matrix_instance_f32 K_desc, y_desc, out_desc;
    arm_mat_init_f32(&K_desc, 6, 3, K.data());
    arm_mat_init_f32(&y_desc, 3, 1, const_cast<float *>(correction.y.data()));
    arm_mat_init_f32(&out_desc, 6, 1, dx.data());
    if (arm_mat_mult_f32(&K_desc, &y_desc, &out_desc) != ARM_MATH_SUCCESS)
    {
        return PYRO_ERROR;
    }
    if (ConvergeFlag)
        for (std::size_t i = 4; i < 6; ++i)
            dx[i] = std::clamp(dx[i], -1.0e-2f * dt, 1.0e-2f * dt);
    // 保留原模型禁止 qz 修正增量的策略。
    // Preserve the model policy that suppresses the qz correction increment.
    dx[3]             = 0.0f;
    correction.use_dx = true;
    return PYRO_OK;
}

void ins_attitude_t::_publish()
{
    // 发布四元数和零偏；欧拉角由 rad 转为度，并累计航向跨圈。
    // Publish quaternion/bias; convert Euler rad to degrees and unwrap yaw.
    const auto &state = ekf.get_state();
    std::copy_n(state.data(), 4, q);
    GyroBias[0] = state[4];
    GyroBias[1] = state[5];
    GyroBias[2] = 0.0f;
    Yaw         = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]),
                             2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) *
          57.295779513f;
    Roll = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                      2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) *
           57.295779513f;
    Pitch = std::asin(
                std::clamp(-2.0f * (q[1] * q[3] - q[0] * q[2]), -1.0f, 1.0f)) *
            57.295779513f;
    if (Yaw - YawAngleLast > 180.0f)
        --YawRoundCount;
    else if (Yaw - YawAngleLast < -180.0f)
        ++YawRoundCount;
    YawTotalAngle = 360.0f * YawRoundCount + Yaw;
    YawAngleLast  = Yaw;
}

} // namespace pyro
