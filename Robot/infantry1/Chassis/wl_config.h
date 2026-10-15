#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__
#include "pyro_algo_common.h"
#include "lqr_coef.h"
#include "pyro_core_def.h"

namespace pyro
{
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
constexpr float OJ5                     = 0.0945f;//0.1134
constexpr float J4J5                    = 0.1125f;//0.1350
constexpr float OJ8                     = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float HIP_CALIBRATION_OFFSET  = -2.44899;
constexpr float KNEE_CALIBRATION_OFFSET = 2.62047f;

constexpr float LEFT_HIP_OFFSET =
    -loop_fp32_PI(2.77094f + HIP_CALIBRATION_OFFSET);
constexpr float LEFT_KNEE_OFFSET =
    -loop_fp32_PI(3.02693f + KNEE_CALIBRATION_OFFSET);
constexpr float RIGHT_HIP_OFFSET =
    -loop_fp32_PI(1.69349f + HIP_CALIBRATION_OFFSET);
constexpr float RIGHT_KNEE_OFFSET =
    -loop_fp32_PI(-0.35047f + KNEE_CALIBRATION_OFFSET);

// constexpr float LEFT_HIP_OFFSET   =0;
// constexpr float LEFT_KNEE_OFFSET  =0;
// constexpr float RIGHT_HIP_OFFSET  =0;
// constexpr float RIGHT_KNEE_OFFSET =0;

constexpr float MAX_LEG_LENGTH        = 0.38f;
constexpr float MIN_LEG_LENGTH        = 0.18f;
// Both margins are measured inward from the mechanical limit. Damping starts
// first; stiffness joins it only in the inner region nearest the stop.
constexpr float LEG_LENGTH_WALL_MARGIN = 0.005f; // stiffness + damping: 5 mm
constexpr float LEG_LENGTH_WALL_DAMPING_MARGIN = 0.020f; // damping only starts at 20 mm
constexpr float LEG_LENGTH_WALL_MAX    = MAX_LEG_LENGTH - LEG_LENGTH_WALL_MARGIN;
constexpr float LEG_LENGTH_WALL_MIN    = MIN_LEG_LENGTH + LEG_LENGTH_WALL_MARGIN;
constexpr float LEG_LENGTH_WALL_DAMPING_MAX = MAX_LEG_LENGTH - LEG_LENGTH_WALL_DAMPING_MARGIN;
constexpr float LEG_LENGTH_WALL_DAMPING_MIN = MIN_LEG_LENGTH + LEG_LENGTH_WALL_DAMPING_MARGIN;
constexpr float LEG_LENGTH_WALL_K      = 250000.0f; // N/m
constexpr float LEG_LENGTH_WALL_D      = 750.0f;    // N*s/m; softer damping across the wider region

constexpr float LEFT_LEG_DIRECTION    = -1.0f;
constexpr float RIGHT_LEG_DIRECTION   = 1.0f;

constexpr float LEFT_WHEEL_DIRECTION  = -1.0f;
constexpr float RIGHT_WHEEL_DIRECTION = 1.0f;

// Foot masses used by flight feedforward and contact inertia correction.
constexpr float LEG_MASS              = 1.232f;
constexpr float WHEEL_MASS            = 0.57f;
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
//过坎
constexpr float LEG_ACCEL_THRESHOLD = 30.0f;
constexpr float GETOVER_BETA_THRESHOLD = 8.0f * PI / 180.0f;
constexpr float GETOVER_SHORTEN_LENGTH = 0.05f;
constexpr float GETOVER_MIN_TARGET_LENGTH = 0.20f;
constexpr float GETOVER_FORWARD_RAD = 2.0f * PI / 180.0f;
constexpr float GETOVER_FORCE_SCALE = 2.0f;
constexpr float GETOVER_TIMEOUT = 0.12f;
constexpr float GETOVER_REARM_DELAY = 1.0f;
// Airborne and landing detection defaults. Tune from logged support-force data.
constexpr float AIR_LENGTH_TARGET                   = 0.35f;//腿长目标值，低于上限虚拟墙
constexpr float NORMAL_LENGTH_TARGET                 = 0.25f;
constexpr float AIR_LENGTH_RATE                      = 0.60f;
constexpr float AIR_WHEEL_LOCK_K                     = 0.08f;
constexpr float AIR_TAKEOFF_FORCE_ON                 = 40.0f;//单腿支持力阈值，越小越易离地
constexpr float AIR_CONTACT_FORCE_OFF                = 50.0f;
constexpr float AIR_LANDING_ACC_ON                   = 3.0f;
constexpr float AIR_LANDING_COMPRESSION              = 0.03f;
constexpr float AIR_LENGTH_RECOVERY_RATE             = 0.20f;
constexpr float AIR_LENGTH_RECOVERY_EPSILON          = 0.005f;//腿长误差
constexpr float AIR_ACCEL_LPF_TAU                    = 0.02f;
constexpr float SUPPORT_FORCE_LPF_TAU                = 0.01f;
constexpr float SUPPORT_FORCE_BIAS[2]                = {0.0f, 0.0f};
constexpr uint16_t AIR_TAKEOFF_DEBOUNCE_TICKS        = 75;
constexpr uint16_t AIR_LANDING_DEBOUNCE_TICKS        = 8;

// Active jump state machine. These values are separate from passive airborne
// detection so a commanded jump never depends on a takeoff-force threshold.
constexpr float JUMP_PRECOMPRESS_LENGTH               = 0.185f; // 预压缩目标腿长 (m)
constexpr float JUMP_AIR_LENGTH                       = 0.20f; // 收腿完成门限基准，与虚拟墙厚度独立
// Positive magnitudes per motor. Call sites choose extension (+F_L) or
// retraction (-F_L); equivalent F_L = 2 * signed_torque / abs(J_L).
constexpr float JUMP_PRECOMPRESS_MOTOR_TORQUE         = 5.0f;
constexpr float JUMP_PUSH_MOTOR_TORQUE                = 18.0f;
constexpr float JUMP_RECOVERY_MOTOR_TORQUE            = 4.0f;
constexpr float JUMP_AIR_MOTOR_TORQUE                 = 2.0f;
constexpr float JUMP_PRECOMPRESS_TIME                 = 0.2f; // 文档式预压缩超时 (s)
constexpr float JUMP_LENGTH_TOLERANCE                 = 0.003f;
constexpr float JUMP_SETTLED_LEG_SPEED                = 0.2f;
constexpr float JUMP_PUSH_LENGTH_MARGIN               = 0.01f;
constexpr float JUMP_PUSH_TIMEOUT                     = 0.5f;  // 蹬腿最长时间 (s)
constexpr float JUMP_RECOVERY_TIMEOUT                 = 0.4f;  // 收腿最长时间 (s)
constexpr float JUMP_RECOVERY_HOLD_TIME               = 0.1f;
constexpr float JUMP_TOTAL_TIMEOUT                    = 3.0f;  // 空中阶段最长时间 (s)
constexpr float JUMP_LANDING_REVERSE_SPEED            = -0.05f; // 判据 A：腿长收缩速度阈值 (m/s)
constexpr float JUMP_LANDING_LENGTH_MARGIN            = 0.03f; // 判据 B：腿长低于机械最大腿长的余量 (m)
constexpr float JUMP_LANDING_EXTENSION_SPEED_MIN      = 0.10f; // 启用减速判定所需的最小伸展速度峰值 (m/s)
constexpr float JUMP_LANDING_EXTENSION_SPEED_DROP     = 0.15f; // 伸展速度相对峰值的最小下降量 (m/s)
constexpr float JUMP_LANDING_CONFIRM_TIME             = 0.020f; // A 或 B 连续成立的触地确认时间 (s)
constexpr float JUMP_T_P_LIMIT = 8.0f;
constexpr bool JUMP_ENABLED                          = true;
constexpr float JUMP_STAND_CONFIRM_TIME              = 0.3f;
constexpr float JUMP_READY_ANGLE                     = 5.0f * PI / 180.0f;
constexpr float JUMP_READY_ANGULAR_SPEED             = 0.5f;
constexpr float JUMP_COOLDOWN_TIME                   = 1.0f;
constexpr float JUMP_POSITION_ERROR_LIMIT            = 0.8f;
constexpr float JUMP_LANDING_VELOCITY_SCALE          = 0.8f; // 退出预压缩时实测速度到落地参考的比例
constexpr float AIR_CONTACT_WALL_GUARD_MARGIN        = 0.03f; // 触地速度判据排除末端制动区
constexpr float AIR_CONTACT_MIN_TIME                = 0.025f;
constexpr float AIR_CONTACT_RELEASE_FORCE           = 20.0f;
constexpr float AIR_CONTACT_RELEASE_TIME            = 0.02f;
constexpr float LANDING_PITCH_REFERENCE_RATE         = 10.0f;
constexpr float LANDING_BETA_REFERENCE_RATE          = 5.0f;
constexpr float LANDING_VELOCITY_REFERENCE_RATE      = 5.0f; // m/s^2; 落地速度参考向当前遥控速度的变化率

constexpr uint32_t L_WP_POLY_DEGREE                  = 3;

// L_wp(L) = 0.0581 + 0.3760*L + 0.7972*L^2 - 0.7381*L^3.
// Coefficients are in ascending-power order: c0, c1, c2, c3.
constexpr float L_WP_POLY_COEF[L_WP_POLY_DEGREE + 1] = {0.0581f, 0.3760f,
                                                        0.7972f, -0.7381f};

constexpr float WHEEL_RADIUS                         = 0.06f;

constexpr static float YAW_OFFSET = -1.152f;



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



//各种开关
#define Using_Gimbal_Cmd 0
} // namespace pyro
#endif
