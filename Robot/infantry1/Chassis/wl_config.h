#ifndef __WL_CONFIG_H__
#define __WL_CONFIG_H__
#include "pyro_algo_common.h"
#include "lqr_coef.h"
#include "leso_coef.h"

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
constexpr float OJ5                     = 0.0945f;
constexpr float J4J5                    = 0.1125f;
constexpr float OJ8                     = 0.2100f;
// constexpr float MIN_LEG_LENGTH = 0.18f;
// constexpr float MAX_LEG_LENGTH = 2.5f;
constexpr float HIP_CALIBRATION_OFFSET  = -2.44899;
constexpr float KNEE_CALIBRATION_OFFSET = 2.62047f;

constexpr float LEFT_HIP_OFFSET =
    -loop_fp32_PI(3.07832f + HIP_CALIBRATION_OFFSET);
constexpr float LEFT_KNEE_OFFSET =
    -loop_fp32_PI(0.25690f + KNEE_CALIBRATION_OFFSET);
constexpr float RIGHT_HIP_OFFSET = 
    -loop_fp32_PI(-0.99730f + HIP_CALIBRATION_OFFSET);
constexpr float RIGHT_KNEE_OFFSET = 
    -loop_fp32_PI(2.71408f + KNEE_CALIBRATION_OFFSET);

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

constexpr static float YAW_OFFSET = -2.2f;

#define Using_Gimbal_Cmd 0


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
