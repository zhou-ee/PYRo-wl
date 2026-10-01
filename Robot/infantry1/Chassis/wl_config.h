#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__
#include "pyro_algo_common.h"
#include "lqr_coef.h"

#define RDOB_EN 1

namespace pyro
{
// Initial RDOB compensation envelope around the current static trim.
constexpr float RDOB_THETA_SCALE                  = 0.30f; // rad
constexpr float RDOB_PHI_SCALE                    = 0.18f; // rad
constexpr float RDOB_BODY_RATE_SCALE              = 2.50f; // rad/s
constexpr float RDOB_LEG_RATE_SCALE               = 0.35f; // m/s
constexpr float RDOB_BETA_SCALE                   = 0.50f; // rad
constexpr float RDOB_BETA_RATE_SCALE              = 3.00f; // rad/s
constexpr float RDOB_DEVIATION_FULL               = 0.60f;
constexpr float RDOB_DEVIATION_OFF                = 1.20f;
constexpr float RDOB_COMPENSATION_RISE_PER_STEP   = 0.001f; // 1 s at 1 kHz
// exp(-2*pi*0.05 Hz*0.001 s): leak the reconstructed disturbance before
// rebasing z. The RDOB coefficients also assume a 1 ms update interval.
constexpr float RDOB_DISTURBANCE_LEAK_POLE        = 0.998f;

// IMU 安装偏移与前向加速度带限微分，固定 1 ms 更新。
constexpr float IMU_DX                           = 0.24f;   // m，向前为正
constexpr float IMU_DZ                           = -0.034f; // m，向上为正
constexpr float IMU_ACCEL_TS                     = 0.001f;
constexpr float IMU_ACCEL_CUTOFF_HZ              = 15.0f;
constexpr float IMU_ACCEL_WC                     = 2.0f * PI * IMU_ACCEL_CUTOFF_HZ;
constexpr float IMU_ACCEL_DIFF_A                  =
    (2.0f - IMU_ACCEL_WC * IMU_ACCEL_TS) / (2.0f + IMU_ACCEL_WC * IMU_ACCEL_TS);
constexpr float IMU_ACCEL_DIFF_B                  =
    2.0f * IMU_ACCEL_WC / (2.0f + IMU_ACCEL_WC * IMU_ACCEL_TS);

// 四状态前向 KF，固定 1 kHz；Q/R/P 均为方差，以下为初始整定值。
constexpr float FORWARD_KF_TS                    = 0.001f;
constexpr float FORWARD_KF_ACCEL_VARIANCE        = 0.04f;   // (m/s^2)^2 / step
constexpr float FORWARD_KF_BIAS_VARIANCE         = 1.0e-7f; // (m/s^2)^2 / step
constexpr float FORWARD_KF_R_IMU                 = 0.25f;   // (m/s^2)^2
constexpr float FORWARD_KF_R_MODEL               = 1.0f;    // (m/s^2)^2
constexpr float FORWARD_KF_R_WHEEL               = 0.0025f; // (m/s)^2
constexpr float FORWARD_KF_P_INIT[4]             = {0.01f, 0.25f, 1.0f, 0.25f};
constexpr float FORWARD_KF_MIN_VARIANCE[4]       = {1.0e-8f, 1.0e-6f, 1.0e-5f, 1.0e-8f};
constexpr float FORWARD_KF_WHEEL_CUTOFF_HZ       = 15.0f;
constexpr float FORWARD_KF_WHEEL_WC              = 2.0f * PI * FORWARD_KF_WHEEL_CUTOFF_HZ;
constexpr float FORWARD_KF_WHEEL_DIFF_A          =
    (2.0f - FORWARD_KF_WHEEL_WC * FORWARD_KF_TS) /
    (2.0f + FORWARD_KF_WHEEL_WC * FORWARD_KF_TS);
constexpr float FORWARD_KF_WHEEL_DIFF_B          =
    2.0f * FORWARD_KF_WHEEL_WC / (2.0f + FORWARD_KF_WHEEL_WC * FORWARD_KF_TS);
constexpr float FORWARD_KF_SLIP_NIS_LOW          = 4.0f;
constexpr float FORWARD_KF_SLIP_NIS_HIGH         = 16.0f;
constexpr float FORWARD_KF_SLIP_ACCEL_LOW        = 1.0f; // m/s^2
constexpr float FORWARD_KF_SLIP_ACCEL_HIGH       = 4.0f; // m/s^2
constexpr float FORWARD_KF_REFERENCE_ACCEL_LOW   = 1.0f; // m/s^2
constexpr float FORWARD_KF_REFERENCE_ACCEL_HIGH  = 4.0f; // m/s^2
constexpr float FORWARD_KF_SLIP_RISE_TAU         = 0.003f; // s，打滑评分上升
constexpr float FORWARD_KF_SLIP_FALL_TAU         = 0.03f;  // s，轮速权重恢复
constexpr float FORWARD_KF_WHEEL_R_MULTIPLIER    = 500.0f;

// 前向模型加速度：L1/L2 总阶三次，来自上位机 model_acceleration_schedule.h。
// 行序：k_theta,k_phi,k_beta1,k_beta2,b_w1,b_w2,b_p1,b_p2,b_f1,b_f2。
// 列序：1,y,y^2,y^3,x,xy,xy^2,x^2,x^2y,x^3；腿长范围 [0.18,0.38] m。
constexpr uint32_t MODEL_ACCEL_STATE_DIM             = 4U;
constexpr float MODEL_ACCEL_LENGTH_CENTER          = 2.800000012e-01f;
constexpr float MODEL_ACCEL_LENGTH_INV_SCALE       = 1.000000095e+01f;
constexpr float MODEL_ACCEL_COEF[10][10] = {
    // k_theta
    {-2.895564539e-03f, 1.578039490e-02f, -8.613116108e-03f, 3.450470045e-03f, 1.578039490e-02f, -6.080517051e-05f, 3.500955791e-05f, -8.613116108e-03f, 3.500955791e-05f, 3.450470045e-03f},
    // k_phi
    {0.000000000e+00f, -3.264736831e-01f, 3.087833226e-01f, -1.142917573e-01f, 3.264736831e-01f, 0.000000000e+00f, -3.085462959e-05f, -3.087833226e-01f, 3.085462959e-05f, 1.142917573e-01f},
    // k_beta1
    {4.870008469e+01f, -5.145293102e-02f, 4.085071012e-02f, -1.707911678e-02f, 3.191469431e+00f, -8.676779456e-03f, 5.327824503e-03f, -1.822665453e+00f, 1.312274253e-03f, 7.751883268e-01f},
    // k_beta2
    {4.870008469e+01f, 3.191469431e+00f, -1.822665453e+00f, 7.751883268e-01f, -5.145293102e-02f, -8.676779456e-03f, 1.312274253e-03f, 4.085071012e-02f, 5.327824503e-03f, -1.707911678e-02f},
    // b_w1
    {9.673137665e+00f, 1.479585771e-03f, -2.238195011e-04f, 8.850350423e-05f, -2.854974382e-02f, -1.835711882e-04f, -8.320622146e-05f, -1.526049525e-01f, 3.119647154e-04f, 1.022374630e-01f},
    // b_w2
    {9.673137665e+00f, -2.854974382e-02f, -1.526049525e-01f, 1.022374630e-01f, 1.479585771e-03f, -1.835711882e-04f, 3.119647154e-04f, -2.238195011e-04f, -8.320622146e-05f, 8.850350423e-05f},
    // b_p1
    {1.704816818e+00f, 5.178009742e-04f, 3.019259893e-04f, -1.399408357e-04f, -4.866441190e-01f, 7.324374747e-04f, -4.861770722e-04f, 1.078990996e-01f, -1.774702105e-04f, -8.621597663e-03f},
    // b_p2
    {1.704816818e+00f, -4.866441190e-01f, 1.078990996e-01f, -8.621597663e-03f, 5.178009742e-04f, 7.324374747e-04f, -1.774702105e-04f, 3.019259893e-04f, -4.861770722e-04f, -1.399408357e-04f},
    // b_f1
    {-6.446957588e-03f, 1.844678809e-05f, 1.296279952e-04f, -4.841234477e-05f, 1.082370523e-03f, -3.353391776e-06f, -3.579941676e-07f, -1.372522791e-03f, 3.842601473e-06f, 3.117129381e-04f},
    // b_f2
    {-6.446957588e-03f, 1.082370523e-03f, -1.372522791e-03f, 3.117129381e-04f, 1.844678809e-05f, -3.353391776e-06f, 3.842601473e-06f, 1.296279952e-04f, -3.579941676e-07f, -4.841234477e-05f},
};

constexpr float loop_fp32_PI(float val)
{
    while (val > PI)
    {
        val -= 2 * PI;
    }
    while (val < -PI)
    {
        val += 2 * PI;
    }
    return val;
}
constexpr float OJ5                     = 0.0945f;
constexpr float J4J5                    = 0.1125f;
constexpr float OJ8                     = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float HIP_CALIBRATION_OFFSET  = -2.44899;
constexpr float KNEE_CALIBRATION_OFFSET = 2.62047f;
    constexpr float LEFT_HIP_OFFSET =
        -loop_fp32_PI(2.77928f + HIP_CALIBRATION_OFFSET);
    constexpr float LEFT_KNEE_OFFSET =
        -loop_fp32_PI(3.04188f + KNEE_CALIBRATION_OFFSET);
    constexpr float RIGHT_HIP_OFFSET =
        -loop_fp32_PI(-1.63108f + HIP_CALIBRATION_OFFSET);
    constexpr float RIGHT_KNEE_OFFSET =
        -loop_fp32_PI(-0.35833f + KNEE_CALIBRATION_OFFSET);

// constexpr float LEFT_HIP_OFFSET   =0;
// constexpr float LEFT_KNEE_OFFSET  =0;
// constexpr float RIGHT_HIP_OFFSET  =0;
// constexpr float RIGHT_KNEE_OFFSET =0;

constexpr float MAX_LEG_LENGTH        = 0.38f;
constexpr float MIN_LEG_LENGTH        = 0.18f;
constexpr float LEG_LENGTH_WALL_MARGIN = 0.02f;
constexpr float LEG_LENGTH_WALL_MAX    = MAX_LEG_LENGTH - LEG_LENGTH_WALL_MARGIN;
constexpr float LEG_LENGTH_WALL_MIN    = MIN_LEG_LENGTH + LEG_LENGTH_WALL_MARGIN;
constexpr float LEG_LENGTH_WALL_K      = 15000.0f; // N/m
constexpr float LEG_LENGTH_WALL_D      = 250.0f;   // N*s/m

constexpr float LEFT_LEG_DIRECTION    = -1.0f;
constexpr float RIGHT_LEG_DIRECTION   = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION  = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION = 1.0f;

// Single-leg gravity compensation parameters.
constexpr float SINGLE_LEG_BODY_MASS  = 6.0f;
constexpr float LEG_MASS              = 1.232f;
constexpr float GRAVITY_ACCELERATION  = 9.81f;
constexpr float K_t                                  = 0.21777611f;
constexpr float reduction_ratio                      = 13.94f;
constexpr float rec_reduction_ratio                  = 1 / reduction_ratio;
constexpr float MAX_F_L                              = 300.0f;
constexpr float MAX_T_P                              = 60.0f;
constexpr float MAX_CURRENT                          = 15.0f;
constexpr float MAX_T_W                              = K_t * MAX_CURRENT;
constexpr float SPIN_YAW_ACCEL                       = 10.0f; // rad/s^2 //最大的小陀螺加速时的加速度
constexpr float SPIN_YAW_DECEL                       = 10.0f; // rad/s^2 //最大的小陀螺减速是的加速度
constexpr float SPIN_EXIT_DECEL_TIME                 = 0.2f;  // s
constexpr float SPIN_RECOVERY_YAW_KP                 = 2.0f;

// Normal/Balance roll integral: positive trim adds to left and subtracts right.
constexpr float NORMAL_ROLL_INTEGRAL_KI              = 120.0f; // N/(rad*s)
constexpr float NORMAL_ROLL_INTEGRAL_LIMIT           = 15.0f;  // N per leg
constexpr float NORMAL_ROLL_INTEGRAL_DEADBAND        =
    0.2f * PI / 180.0f;

// 300 N 气弹簧广义力拟合，适用腿长 L 范围为 [0.18, 0.38] m。
// x = (L - 0.28) / 0.10，F_gas(L) = sum(c[k] * x^k)。
constexpr float GAS_SPRING_LENGTH_CENTER             = 0.28f;
constexpr float GAS_SPRING_LENGTH_SCALE              = 0.10f;
// Hardware tuning factor applied only to the motor-side cancellation.
constexpr float GAS_SPRING_COMPENSATION_SCALE        = 1.0f;
constexpr uint32_t GAS_SPRING_FORCE_POLY_DEGREE      = 3;
constexpr float GAS_SPRING_FORCE_POLY_COEF[
    GAS_SPRING_FORCE_POLY_DEGREE + 1] = {
    119.867552440332f,
    37.1404981279233f,
    -4.91246097241044f,
    2.70356625079196f,
};

// Airborne and landing detection defaults. Tune from logged support-force data.
constexpr float AIR_LENGTH_TARGET                   = 0.35f;//腿长目标值，低于上限虚拟墙
constexpr float NORMAL_LENGTH_TARGET                 = 0.20f;
constexpr float AIR_LENGTH_RATE                      = 0.60f;
constexpr float AIR_WHEEL_LOCK_K                     = 0.08f;
constexpr float AIR_TAKEOFF_FORCE_ON                 = 60.0f;//单腿支持力阈值，越小越易离地
constexpr float AIR_CONTACT_FORCE_OFF                = 35.0f;
constexpr float AIR_LANDING_ACC_ON                   = 3.0f;
constexpr float AIR_LANDING_COMPRESSION              = 0.03f;
constexpr float AIR_LENGTH_RECOVERY_RATE             = 0.20f;
constexpr float AIR_LENGTH_RECOVERY_EPSILON          = 0.005f;//腿长误差
constexpr float AIR_ACCEL_LPF_TAU                    = 0.02f;
constexpr float SUPPORT_FORCE_LPF_TAU                = 0.01f;
constexpr float SUPPORT_FORCE_EFFECTIVE_MASS         =
    SINGLE_LEG_BODY_MASS + LEG_MASS;
constexpr float SUPPORT_FORCE_BIAS[2]                = {0.0f, 0.0f};
constexpr uint16_t AIR_TAKEOFF_DEBOUNCE_TICKS        = 75;
constexpr uint16_t AIR_LANDING_DEBOUNCE_TICKS        = 8;


constexpr uint32_t L_WP_POLY_DEGREE                  = 3;

// L_wp(L) = 0.0581 + 0.3760*L + 0.7972*L^2 - 0.7381*L^3.
// Coefficients are in ascending-power order: c0, c1, c2, c3.
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {0.0581f, 0.3760f,
                                                        0.7972f, -0.7381f};

constexpr float WHEEL_RADIUS                         = 0.06f;

constexpr static float YAW_OFFSET = 0.95f;

#define Using_Gimbal_Cmd 1

namespace leg_def
{
enum : uint8_t
{
    L = 0, // LEFT
    R = 1  // RIGHT
};
}
namespace joint_def
{
enum : uint8_t
{
    HIP  = 0,
    KNEE = 1,
};
}
} // namespace pyro
#endif
