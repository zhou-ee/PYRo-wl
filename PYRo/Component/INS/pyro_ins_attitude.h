/**
 * @file pyro_ins_attitude.h
 * @brief INS 应用层的四元数姿态模型，使用独立通用 EKF。
 *        INS application quaternion attitude model using the generic EKF.
 */
#ifndef PYRO_INS_ATTITUDE_H
#define PYRO_INS_ATTITUDE_H

#include "../../Algorithm/KF/pyro_algo_ekf.h"

namespace pyro
{

/**
 * @brief INS 姿态适配：IMU 预处理、状态/观测模型、门控与角度输出。
 *        INS attitude adapter: IMU preprocessing, models, gating and angles.
 * @details x = [qw, qx, qy, qz, bias_x, bias_y]，z 为归一化加速度方向。
 *          x = [qw, qx, qy, qz, bias_x, bias_y]; z is normalized acceleration.
 *          模型每步更新 F、H、Qd、R，再传给通用核心；不估计 Z 轴零偏。
 *          The model supplies F, H, Qd and R each step; Z bias is not
 * estimated.
 */
class ins_attitude_t
{
  public:
    /**
     * @brief 本模型使用的通用 EKF 类型：N=6、Z=3，模型在 INS 层提供。
     *        Generic EKF type: N=6, Z=3, model supplied by INS.
     * @details state_t: 6；covariance_t / transition_t: 6×6；
     *          observation_t: 3×6；measurement_noise_t: 3×3；gain_t: 6×3。
     *          All matrices are row-major; dimensions above are rows × columns.
     *          所有矩阵按行存储，上述矩阵维度依次为行数 × 列数。
     */
    using filter_t = ekf_t<6, 3>;

    /**
     * @brief 初始化或重新初始化状态、协方差、低通历史和计数器。
     *        Initialize or reinitialize state, covariance, LPF
     * history/counters.
     * @param q_noise 四元数过程噪声强度 Q1，Qd 第 0..3
     * 对角项为 Q1·dt。 Quaternion noise intensity Q1; Qd diagonal = Q1·dt. 单位
     * 1/s（四元数无量纲） / Units: 1/s (unitless q).
     * @param bias_noise XY 零偏噪声强度 Q2，Qd 第 4..5 对角项为
     * Q2·dt。 XY gyro bias noise intensity; diagonal = Q2·dt. 单位 (rad/s)²/s /
     * Units: (rad/s)²/s.
     * @param r_noise 归一化加速度观测的方差 R，各轴对角项相同。
     *                      Variance R of normalized acceleration, same per
     * axis.
     * @param fading 零偏协方差遗忘系数 λ；0<λ≤1，输入大于 1 时截为 1。
     *               Bias covariance fading factor λ; values above 1 clamp to 1.
     * @param accel_lpf 加速度低通时间常数，单位 s；0
     * 表示直接使用当前样本。 Acceleration LPF time constant in s; 0 uses raw
     * samples.
     * @return PYRO_OK 成功；非有限、负噪声或非法 λ 返回 PYRO_PARAM_ERROR。
     *         PYRO_OK on success; invalid/non-finite parameters return an
     * error.
     */
    status_t init(float q_noise, float bias_noise, float r_noise, float fading,
                  float accel_lpf);
    /**
     * @brief 由 IMU 样本推进一次预测和观测修正；未初始化时使用默认参数。
     *        Predict/correct from one IMU sample; auto-init uses default
     * tuning.
     * @param gx 机体系 X 角速度，rad/s，未去估计零偏。
     *           Body X angular rate in rad/s, before estimated bias removal.
     * @param gy 机体系 Y 角速度，rad/s。 / Body Y angular rate in rad/s.
     * @param gz 机体系 Z 角速度，rad/s。 / Body Z angular rate in rad/s.
     * @param ax 机体系 X 加速度，m/s²。 / Body X acceleration in m/s².
     * @param ay 机体系 Y 加速度，m/s²。 / Body Y acceleration in m/s².
     * @param az 机体系 Z 加速度，m/s²。 / Body Z acceleration in m/s².
     * @param dt_s 更新周期，单位 s，必须有限且大于零。
     *           Update period in s; must be finite and positive.
     * @details 非法陀螺/dt 保持上次姿态；非法或零加速度仅执行陀螺预测。
     *          Invalid gyro/dt holds attitude; invalid/zero accel predicts
     * only. 静止时输入加速度模长应接近 9.8 m/s²。 Stationary input acceleration
     * norm should be about 9.8 m/s².
     *          修正失败或被门限拒绝时保留预测结果，后续有效样本可恢复。
     *          Failed/rejected correction retains prediction; later data may
     * recover.
     * @return PYRO_OK 正常修正；PYRO_WARNING 无观测或门限拒绝；其他值为错误。
     *         PYRO_OK on correction; PYRO_WARNING on no/rejected data; else
     * error.
     */
    status_t update(float gx, float gy, float gz, float ax, float ay, float az,
                    float dt_s);
    /// 最近一次 init/update 的结果。 / Result of the last init/update call.
    [[nodiscard]] status_t get_status() const
    {
        return _status;
    }
    /// 最新 6×6 状态雅可比 F；预测时随陀螺、四元数和 dt 更新。
    /// Latest 6×6 state Jacobian F, updated with gyro, quaternion and dt.
    [[nodiscard]] const filter_t::transition_t &get_transition() const
    {
        return F_;
    }
    /// 最新 3×6 观测雅可比 H；修正时随预测四元数更新。
    /// Latest 3×6 observation Jacobian H, updated from the predicted
    /// quaternion.
    [[nodiscard]] const filter_t::observation_t &get_observation() const
    {
        return H_;
    }

    // 应用调参与输出字段，保持现有姿态行为；不属于算法核心。
    // Application tuning/output fields, preserving attitude behavior, not core
    // API.
    /// 初始化标记，1 表示就绪。 / Initialization flag: 1 means ready.
    uint8_t Initialized = 0;
    /// 独立持有 x、P、K 和诊断的核心。 / Core owning x, P, K and diagnostics.
    filter_t ekf;
    /// NIS 门限判断的收敛标记。 / Convergence flag determined by the NIS gate.
    uint8_t ConvergeFlag = 0;
    /// 低角速率、加速度接近重力时置 1。 / Low-rate, near-gravity stability
    /// flag.
    uint8_t StableFlag   = 0;
    /// 稳定状态下连续超过 NIS 门限的次数。 / Consecutive stable gate failures.
    uint64_t ErrorCount  = 0;
    /// 成功预测后发布输出的累计次数。 / Number of publications after
    /// prediction.
    uint64_t UpdateCount = 0;
    /// [qw,qx,qy,qz]，机体系到导航系旋转，无量纲。
    /// [qw,qx,qy,qz], body-to-navigation rotation, dimensionless.
    float q[4]           = {1.0f, 0.0f, 0.0f, 0.0f};
    /// [bias_x,bias_y,0]，单位 rad/s。 / Estimated gyro bias in rad/s; Z is
    /// zero.
    float GyroBias[3]{};
    /// 减去估计零偏后的机体系角速度，rad/s。 / Bias-corrected body rates,
    /// rad/s.
    float Gyro[3]{};
    /// 最近有效低通加速度，m/s²；并非归一化观测。
    /// Last valid low-pass acceleration in m/s²; not the normalized
    /// observation.
    float Accel[3]{};
    /// 兼容旧名：实际为 acos(|h_i(x)|)，单位 rad，并非余弦值。
    /// Legacy name: stores acos(|h_i(x)|) in rad, not cosine values.
    float OrientationCosine[3]{};
    /// 可调低通时间常数，s。 / Tunable LPF time constant in s.
    float accLPFcoef        = 0.0f;
    /// 去零偏角速度模长，rad/s。 / Bias-corrected angular-rate norm in rad/s.
    float gyro_norm         = 0.0f;
    /// 低通加速度模长，m/s²；无有效观测时为零。
    /// Low-pass acceleration norm in m/s²; zero when observation is invalid.
    float accl_norm         = 0.0f;
    /// 由 NIS 门限计算的增益缩放系数。 / Gain scale derived from the NIS gate.
    float AdaptiveGainScale = 1.0f;
    /// 横滚角，度。 / Roll angle in degrees.
    float Roll              = 0.0f;
    /// 俯仰角，度。 / Pitch angle in degrees.
    float Pitch             = 0.0f;
    /// 单圈航向角，度。 / Wrapped yaw angle in degrees.
    float Yaw               = 0.0f;
    /// 按跨圈计数展开的航向角，度。 / Unwrapped yaw angle in degrees.
    float YawTotalAngle     = 0.0f;
    /// 可调四元数过程噪声强度，每步乘 dt。 / Quaternion noise intensity × dt.
    float Q1                = 0.0f;
    /// 可调 XY 零偏过程噪声强度，每步乘 dt。 / XY bias noise intensity × dt.
    float Q2                = 0.0f;
    /// 可调归一化观测噪声方差。 / Tunable normalized measurement variance.
    float R                 = 0.0f;
    /// 最近接受的采样周期，s。 / Last accepted sample period in s.
    float dt                = 0.0f;
    /// 归一化观测残差平方（标准名称：归一化新息平方，NIS），无量纲。
    /// Normalized innovation squared (NIS) = yᵀS⁻¹y, dimensionless.
    float ChiSquare_Data[1]{};
    /// NIS 拒绝门限，保留原模型调参。 / NIS rejection threshold, legacy tuning.
    float ChiSquareTestThreshold = 1.0e-8f;
    /// 可调零偏协方差遗忘系数，0<λ≤1。 / Bias fading factor, 0<λ≤1.
    float lambda                 = 1.0f;
    /// 航向角累计跨圈数。 / Accumulated yaw wrap count.
    int16_t YawRoundCount        = 0;
    /// 用于跨圈判断的上次航向角，度。 / Previous wrapped yaw in degrees.
    float YawAngleLast           = 0.0f;

  private:
    /// 构造非线性预测及 F、Qd。 / Build nonlinear prediction, F and Qd.
    status_t _predict();
    /// 构造 h(x)、H、R 并执行 NIS 门控。 / Build h(x), H, R and apply NIS
    /// gating.
    status_t _correct();
    /// 将核心状态发布为兼容字段和欧拉角。 / Publish state and Euler angles.
    void _publish();
    /// INS 门控、增益调整及零偏限幅策略。 / INS gating, gain and bias-limit
    /// policy.
    status_t _gate(filter_t::correction_t &correction);
    /// 检查运行时可调参数。 / Validate runtime tuning parameters.
    static bool _valid_config(float q_noise, float bias_noise, float r_noise,
                              float fading, float accel_lpf);
    /// 最近调用结果。 / Last call result.
    status_t _status        = PYRO_NOT_FOUND;
    /// 加速度低通历史是否已有有效样本。 / Whether valid LPF history exists.
    bool _accel_initialized = false;
    /// F，6×6 状态雅可比。 / F, 6×6 state Jacobian.
    filter_t::transition_t F_{};
    /// H，3×6 重力方向观测雅可比。 / H, 3×6 gravity-direction Jacobian.
    filter_t::observation_t H_{};
    /// Qd，6×6 本周期离散过程噪声协方差。 / Qd, 6×6 discrete process
    /// covariance.
    filter_t::covariance_t Q_d_{};
    /// R，3×3 归一化加速度观测协方差。 / R, 3×3 measurement covariance.
    filter_t::measurement_noise_t R_{};
    /// z 和 h(x_pred)，各为三轴无量纲方向向量。
    /// z and h(x_pred), each a dimensionless three-axis direction.
    filter_t::measurement_t z_{}, z_pred_{};
};

} // namespace pyro
#endif // PYRO_INS_ATTITUDE_H
