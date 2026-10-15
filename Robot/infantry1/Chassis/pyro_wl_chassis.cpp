#include "pyro_wl_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"
#include "rdob_coef.h"
#include "wl_config.h"
#include "pyro_board_drv.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

namespace
{
constexpr float RDOB_DISTURBANCE_LIMIT[INPUT_DIM] = {
    0.1f * MAX_T_W, 0.1f * MAX_T_W,
    0.1f * MAX_T_P, 0.1f * MAX_T_P,
    0.1f * MAX_F_L, 0.1f * MAX_F_L,
};

}



wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis", 512, 512)
{
}

status_t wl_chassis_t::_init()
{
    _ctx                                      = {};
    _ctx.motor                                = _module_deps.motor;
    _ctx.pid                                  = _module_deps.pid;

    _current_cmd.delta_leg_length[leg_def::L] = 0.0f;
    _current_cmd.delta_leg_length[leg_def::R] = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::L]    = 0.0f;
    _current_cmd.delta_leg_rad[leg_def::R]    = 0.0f;

    _ctx.data.leg[leg_def::L].direction       = LEFT_LEG_DIRECTION;
    _ctx.data.leg[leg_def::R].direction       = RIGHT_LEG_DIRECTION;

    _ctx.data.wheel[leg_def::L].direction     = LEFT_WHEEL_DIRECTION;
    _ctx.data.wheel[leg_def::R].direction     = RIGHT_WHEEL_DIRECTION;

    _forward_kf.init();
    return PYRO_OK;
}

void wl_chassis_t::_update_feedback()
{
    // 1. Update timing.
    static uint32_t dwt_cnt = 0;
    const float measured_dt = dwt_drv_t::get_delta_t(&dwt_cnt);
    _ctx.data._dt =
        (measured_dt > 1.0e-5f && measured_dt < 0.1f) ? measured_dt : 0.001f;

    // 2. Update all motor feedback before reading cached values.
    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        for (uint8_t joint = 0; joint < 2; ++joint)
        {
            _ctx.motor.joint[leg][joint]->update_feedback();
        }
        _ctx.motor.wheel[leg]->update_feedback();
    }
    _ctx.motor.yaw->update_feedback();

    // 3. Read and organize all leg-related feedback.
    constexpr float JOINT_POSITION_OFFSET[2][2] = {
        {LEFT_HIP_OFFSET, LEFT_KNEE_OFFSET},
        {RIGHT_HIP_OFFSET, RIGHT_KNEE_OFFSET},
    };

    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        leg_ctx_t &leg_ctx     = _ctx.data.leg[leg];
        wheel_ctx_t &wheel_ctx = _ctx.data.wheel[leg];

        for (uint8_t joint = 0; joint < 2; ++joint)
        {
            motor_base_t *motor = _ctx.motor.joint[leg][joint];
            const float raw_rad =
                motor->get_current_position() +
                leg_ctx.direction * JOINT_POSITION_OFFSET[leg][joint];

            leg_ctx.current_joint_rad[joint] =
                leg_ctx.direction * loop_fp32_constrain(raw_rad, -PI, PI);
            leg_ctx.current_joint_torque[joint] =
                leg_ctx.direction * motor->get_current_torque();
            leg_ctx.current_joint_radps[joint] =
                leg_ctx.direction * motor->get_current_rotate();
        }

        motor_base_t *wheel_motor = _ctx.motor.wheel[leg];
        wheel_ctx.current_radps   = wheel_motor->get_current_rotate() *
                                  wheel_ctx.direction * rec_reduction_ratio;
        wheel_ctx.current_T_w = wheel_motor->get_current_torque() *
                                wheel_ctx.direction * rec_reduction_ratio;
    }

    _ctx.data.yaw.pos = loop_fp32_constrain(_ctx.motor.yaw->get_current_position() - YAW_OFFSET,-PI, PI) ;
    _ctx.data.yaw.rot = _ctx.motor.yaw->get_current_rotate();

    // 4. Convert joint-space feedback to virtual-mechanism feedback.
    _vmc_trans_j2v();

    // 5. Update wheel odometry.
    _ctx.data.odom.wheel_dot_x[1] = _ctx.data.odom.wheel_dot_x[0];
    _ctx.data.odom.wheel_dot_x[0] = 0.5f * WHEEL_RADIUS *
                                   (_ctx.data.wheel[leg_def::L].current_radps +
                                    _ctx.data.wheel[leg_def::R].current_radps);
    _ctx.data.odom.wheel_x +=
        0.5f * (_ctx.data.odom.wheel_dot_x[0] + _ctx.data.odom.wheel_dot_x[1]) *
        _ctx.data._dt;

    // 6. Read IMU feedback.
    auto ins = ins_drv_t::get_instance();
    ins->get_rads_n(&_ctx.data.ins.euler_rad[0], &_ctx.data.ins.euler_rad[1],
                    &_ctx.data.ins.euler_rad[2]);
    ins->get_gyro_b(&_ctx.data.ins.gyro[0], &_ctx.data.ins.gyro[1],
                    &_ctx.data.ins.gyro[2]);
    ins->get_accel_without_g_b(&_ctx.data.ins.accel[0],
                               &_ctx.data.ins.accel[1],
                               &_ctx.data.ins.accel[2]);

    // 7. State vector feedback.
    state_vec_t &state = _ctx.data.measured_state;

    // Retain the KF feedback; balance/spin refresh it again before RDOB/LQR.
    state.x            = _ctx.data.odom.X[0];
    state.dot_x        = _ctx.data.odom.X[1];

    #if Using_Gimbal_Cmd
    //更新yaw的反馈
    const auto& board_ptr = &pyro::board_drv_t::get_instance(pyro::board_drv_t::role_t::CHASSIS,pyro::bsp_can::can3);
    const auto& g2c_data = board_ptr->get_g2c_rx_data();

    state.psi          = -_ctx.data.yaw.pos;
    state.dot_psi = _ctx.data.ins.gyro[0] - static_cast<float>(g2c_data.imu_yaw_radps_100) * 0.01f;
    #else
    state.psi          = _ctx.data.ins.euler_rad[0];
    state.dot_psi      = _ctx.data.ins.gyro[0];
    #endif
    

    state.theta        = _ctx.data.ins.euler_rad[1];
    state.dot_theta    = _ctx.data.ins.gyro[1];
    state.phi          = _ctx.data.ins.euler_rad[2];
    state.dot_phi      = _ctx.data.ins.gyro[2];
    state.beta1        = _ctx.data.leg[leg_def::L].current_leg_rad - PI / 2 -
                  _ctx.data.ins.euler_rad[1];
    state.dot_beta1 =
        _ctx.data.leg[leg_def::L].current_leg_radps - _ctx.data.ins.gyro[1];
    state.beta2 = _ctx.data.leg[leg_def::R].current_leg_rad - PI / 2 -
                  _ctx.data.ins.euler_rad[1];
    state.dot_beta2 =
        _ctx.data.leg[leg_def::R].current_leg_radps - _ctx.data.ins.gyro[1];

    state.L =
        0.5f * (_ctx.data.leg[leg_def::L].current_leg_length +
                _ctx.data.leg[leg_def::R].current_leg_length);
    state.dot_L =
        0.5f * (_ctx.data.leg[leg_def::L].current_leg_speed  +
                _ctx.data.leg[leg_def::R].current_leg_speed );


    const float accel_alpha =
        _ctx.data._dt / (0.01f + _ctx.data._dt);
    for (auto &leg : _ctx.data.leg)
    {
        const float raw_leg_accel =
            std::clamp((leg.current_leg_speed - leg.previous_leg_speed) /
                           _ctx.data._dt,
                       -100.0f, 100.0f);
        leg.current_leg_accel +=
            accel_alpha * (raw_leg_accel - leg.current_leg_accel);
        leg.previous_leg_speed = leg.current_leg_speed;
        const float beta_dot = leg.current_leg_radps - _ctx.data.ins.gyro[1];
        const float raw_beta_accel =
            std::clamp((beta_dot - leg.previous_leg_radps) /
                           _ctx.data._dt,
                       -100.0f, 100.0f);
        leg.current_leg_rad_accel +=
            accel_alpha * (raw_beta_accel - leg.current_leg_rad_accel);
        leg.previous_leg_radps = beta_dot;
    }

    _calc_ddot_x_imu();
    _update_accel_heading_frame();
    // Refresh with the current body pitch before support-force estimation.
    // The takeoff detector must account for force carried by the gas spring,
    // otherwise compensation lowers motor torque and looks like loss of contact.
    _calc_support_force();

}


void wl_chassis_t::_fsm_execute()
{
    static pyro::chassis_function_state_t last_cmd_state = pyro::chassis_function_state_t::NONE;
    if( _current_cmd.cmd_function_state == pyro::chassis_function_state_t::RESTART &&
        last_cmd_state != _current_cmd.cmd_function_state)
    {
        _ctx.data.flag.leg_is_should_restart = false;
    }
    last_cmd_state = _current_cmd.cmd_function_state;

    if (_current_cmd.mode == cmd_base_t::mode_t::ACTIVE && (!_ctx.data.flag.leg_is_should_restart))
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    _main_fsm.execute(this);
}

void wl_chassis_t::_vmc_trans_j2v()
{
    for (auto &leg : _ctx.data.leg)
    {
        const float raw_2_theta = leg.current_joint_rad[joint_def::KNEE] -
                                  leg.current_joint_rad[joint_def::HIP];
        const float theta     = loop_fp32_constrain(raw_2_theta, -PI, PI) / 2;
        const float dot_theta = (leg.current_joint_radps[joint_def::KNEE] -
                                 leg.current_joint_radps[joint_def::HIP]) /
                                2;
        const float sin_theta  = arm_sin_f32(theta);
        const float cos_theta  = arm_cos_f32(theta);
        float OH               = OJ5 * cos_theta;
        float HJ5              = OJ5 * sin_theta;
        float HJ4              = sqrt(J4J5 * J4J5 - HJ5 * HJ5);
        float OJ4              = OH + HJ4;
        leg.J_L                = -OJ8 * sin_theta * (OJ4 / HJ4);
        leg.current_leg_length = OJ4 * OJ8 / OJ5;
        leg.L_wp               = evaluate_polynomial_ascending(
            leg.current_leg_length, L_WP_POLY_COEF, L_WP_POLY_DEGREE);
        leg.current_leg_speed = dot_theta * leg.J_L;
        leg.gas_spring_force   = _calc_gas_spring_force(
            leg.current_leg_length);
        leg.virtual_wall_force = _calc_leg_length_wall_force(leg);

        const float raw_beta  = leg.current_joint_rad[joint_def::HIP] + theta;
        const float beta      = loop_fp32_constrain(raw_beta, -PI, PI);
        leg.current_leg_rad   = beta;
        leg.current_leg_radps = (leg.current_joint_radps[joint_def::KNEE] +
                                 leg.current_joint_radps[joint_def::HIP]) /
                                2;
        leg.current_F_L = (leg.current_joint_torque[joint_def::KNEE] -
                           leg.current_joint_torque[joint_def::HIP]) /
                          leg.J_L +  leg.gas_spring_force;
        leg.current_T_p = leg.current_joint_torque[joint_def::KNEE] +
                          leg.current_joint_torque[joint_def::HIP];
    }
}

void wl_chassis_t::_manual_control()
{
    _ctx.data.leg[leg_def::L].out_F_L =
        _ctx.pid.leg_length[leg_def::L]->calculate(
            _ctx.data.leg[leg_def::L].target_leg_length,
            _ctx.data.leg[leg_def::L].current_leg_length,
            _ctx.data.leg[leg_def::L].current_leg_speed)
    -_ctx.data.leg[leg_def::L].gas_spring_force;

    _ctx.data.leg[leg_def::R].out_F_L =
        _ctx.pid.leg_length[leg_def::R]->calculate(
            _ctx.data.leg[leg_def::R].target_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_speed)
    -_ctx.data.leg[leg_def::R].gas_spring_force;


    float ll_target_radps;
    ll_target_radps = _ctx.pid.leg_control_rad[leg_def::L]->calculate(
        0.0f, _ctx.data.leg[leg_def::L].error_leg_rad);
    _ctx.data.leg[leg_def::L].out_T_p = _ctx.pid.leg_control_radps[leg_def::L]->calculate
        (ll_target_radps, _ctx.data.leg[leg_def::L].current_leg_radps);

    float rl_target_radps;
    rl_target_radps = _ctx.pid.leg_control_rad[leg_def::R]->calculate(
    0.0f, _ctx.data.leg[leg_def::R].error_leg_rad);
    _ctx.data.leg[leg_def::R].out_T_p = _ctx.pid.leg_control_radps[leg_def::R]->calculate
    (rl_target_radps, _ctx.data.leg[leg_def::R].current_leg_radps);

    // _ctx.data.leg[leg_def::L].out_T_p = _ctx.pid.leg_rad[leg_def::L]->calculate(
    //     0.0f, _ctx.data.leg[leg_def::L].error_leg_rad,
    //     _ctx.data.leg[leg_def::L].current_leg_radps);

    // _ctx.data.leg[leg_def::R].out_T_p = _ctx.pid.leg_rad[leg_def::R]->calculate(
    //     0.0f, _ctx.data.leg[leg_def::R].error_leg_rad,
    //     _ctx.data.leg[leg_def::R].current_leg_radps);
}

float wl_chassis_t::_calc_leg_length_wall_force(const leg_ctx_t &leg) const
{
    if (leg.current_leg_length > LEG_LENGTH_WALL_MAX)
    {
        const float penetration = leg.current_leg_length - LEG_LENGTH_WALL_MAX;
        const float damping_force =
            LEG_LENGTH_WALL_D * std::max(leg.current_leg_speed, 0.0f);
        return -LEG_LENGTH_WALL_K * penetration - damping_force;
    }

    if (leg.current_leg_length < LEG_LENGTH_WALL_MIN)
    {
        const float penetration = LEG_LENGTH_WALL_MIN - leg.current_leg_length;
        const float damping_force =
            LEG_LENGTH_WALL_D * std::max(-leg.current_leg_speed, 0.0f);
        return LEG_LENGTH_WALL_K * penetration + damping_force;
    }

    return 0.0f;
}

float wl_chassis_t::_calc_gas_spring_force(const float leg_length) const
{
    const float normalized_length = std::clamp(
        (leg_length - GAS_SPRING_LENGTH_CENTER) /
            GAS_SPRING_LENGTH_SCALE,
        -1.0f, 1.0f);
    return evaluate_polynomial_ascending(
        normalized_length, GAS_SPRING_FORCE_POLY_COEF,
        GAS_SPRING_FORCE_POLY_DEGREE);
}
__attribute__((optimize("O3")))
void wl_chassis_t::_gain_calculate()
{
    // const float norm_delta_L =
    //     std::clamp((_ctx.data.leg[leg_def::L].current_leg_length -
    //                 _ctx.data.leg[leg_def::R].current_leg_length) *
    //                    (1.0f / 0.2f),
    //                -1.0f, 1.0f);
    const float norm_L1 =
        std::clamp((_ctx.data.leg[leg_def::L].current_leg_length -
                    0.5f * (MAX_LEG_LENGTH + MIN_LEG_LENGTH)) *
                       (1.0f / (0.5f * (MAX_LEG_LENGTH - MIN_LEG_LENGTH))),
                   -1.0f, 1.0f);
    const float norm_L2 =
        std::clamp((_ctx.data.leg[leg_def::R].current_leg_length -
                    0.5f * (MAX_LEG_LENGTH + MIN_LEG_LENGTH)) *
                       (1.0f / (0.5f * (MAX_LEG_LENGTH - MIN_LEG_LENGTH))),
                   -1.0f, 1.0f);

    for (uint32_t input = 0; input < INPUT_DIM; ++input)
    {
        for (uint32_t state = 0; state < STATE_DIM; ++state)
        {
            float p_terms[K_POLY_DEGREE + 1];
            for (uint32_t p = 0; p <= K_POLY_DEGREE; ++p)
            {
                p_terms[p] = evaluate_polynomial_ascending(
                    norm_L2, K_POLY_COEF[input][state][p], K_POLY_DEGREE);
            }
            _ctx.data.K[input][state] =
                evaluate_polynomial_ascending(norm_L1, p_terms, K_POLY_DEGREE);
        }
    }

    _ctx.data.U0[lqr_input_def::F_L1] =
        evaluate_polynomial_ascending(norm_L1, FL_U0_POLY_COEF, U0_POLY_DEGREE);
    _ctx.data.U0[lqr_input_def::F_L2] =
        evaluate_polynomial_ascending(norm_L2, FL_U0_POLY_COEF, U0_POLY_DEGREE);
    _ctx.data.target_state.beta1 = evaluate_polynomial_ascending(
        _ctx.data.leg[leg_def::L].current_leg_length, BETA_TRIM_POLY_COEF,
        BETA_TRIM_POLY_DEGREE);
    _ctx.data.target_state.beta2 = evaluate_polynomial_ascending(
        _ctx.data.leg[leg_def::R].current_leg_length, BETA_TRIM_POLY_COEF,
        BETA_TRIM_POLY_DEGREE);
}

void wl_chassis_t::_balance_control()
{

    float error[STATE_DIM];
    for (uint8_t state = 0; state < STATE_DIM; ++state)
    {
        error[state] = _ctx.data.target_state.data[state] -
                       _ctx.data.measured_state.data[state];
    }
    error[lqr_state_def::PSI] =
        loop_fp32_constrain(error[lqr_state_def::PSI], -PI, PI);

    // calculate control data
#if RDOB_EN
    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        _ctx.data.control.data[input] = _ctx.data.U0[input] -
            _ctx.data.rdob.ratio * dist_gain[input] *
                _ctx.data.rdob.dist.data[input];
        for (uint8_t state = 0; state < STATE_DIM; ++state)
        {
            _ctx.data.control.data[input] +=
                _ctx.data.K[input][state] * error[state];
        }
    }
#else
    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        _ctx.data.control.data[input] = _ctx.data.U0[input];
        for (uint8_t state = 0; state < STATE_DIM; ++state)
        {
            _ctx.data.control.data[input] +=
                _ctx.data.K[input][state] * error[state];
        }
    }
#endif

    _ctx.data.control.T_w1 =
        std::clamp(_ctx.data.control.T_w1, -MAX_T_W, MAX_T_W);
    _ctx.data.control.T_w2 =
        std::clamp(_ctx.data.control.T_w2, -MAX_T_W, MAX_T_W);
    _ctx.data.control.T_p1 =
        std::clamp(_ctx.data.control.T_p1, -MAX_T_P, MAX_T_P);
    _ctx.data.control.T_p2 =
        std::clamp(_ctx.data.control.T_p2, -MAX_T_P, MAX_T_P);
    _ctx.data.control.F_l1 =
        std::clamp(_ctx.data.control.F_l1, -MAX_F_L, MAX_F_L);
    _ctx.data.control.F_l2 =
        std::clamp(_ctx.data.control.F_l2, -MAX_F_L, MAX_F_L);

    _ctx.data.leg[leg_def::L].out_T_p   = _ctx.data.control.T_p1;
    _ctx.data.leg[leg_def::R].out_T_p   = _ctx.data.control.T_p2;
    _ctx.data.leg[leg_def::L].out_F_L   = _ctx.data.control.F_l1;
    _ctx.data.leg[leg_def::R].out_F_L   = _ctx.data.control.F_l2;

    _ctx.data.wheel[leg_def::L].out_T_w = _ctx.data.control.T_w1;
    _ctx.data.wheel[leg_def::R].out_T_w = _ctx.data.control.T_w2;
    for (auto &wheel : _ctx.data.wheel)
    {
        wheel.out_current =
            std::clamp(wheel.out_T_w * (1 / K_t), -MAX_CURRENT, MAX_CURRENT);
    }
}

__attribute__((optimize("O3")))
void wl_chassis_t::_rdob_coef_fit()
{
    const float norm_L1 =
        std::clamp((_ctx.data.leg[leg_def::L].current_leg_length -
                    0.5f * (MAX_LEG_LENGTH + MIN_LEG_LENGTH)) *
                       (1.0f / (0.5f * (MAX_LEG_LENGTH - MIN_LEG_LENGTH))),
                   -1.0f, 1.0f);
    const float norm_L2 =
        std::clamp((_ctx.data.leg[leg_def::R].current_leg_length -
                    0.5f * (MAX_LEG_LENGTH + MIN_LEG_LENGTH)) *
                       (1.0f / (0.5f * (MAX_LEG_LENGTH - MIN_LEG_LENGTH))),
                   -1.0f, 1.0f);

    rdob_data_t &rdob = _ctx.data.rdob;
    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        for (uint8_t state = 0; state < GENERAL_STATE_DIM; ++state)
        {
            const float (&coefficient)[4][4] =
                kGammaCoefficients[
                    input * GENERAL_STATE_DIM + state];
            const float x0 =
                ((coefficient[0][3] * norm_L2 + coefficient[0][2]) * norm_L2 +
                 coefficient[0][1]) * norm_L2 + coefficient[0][0];
            const float x1 =
                ((coefficient[1][3] * norm_L2 + coefficient[1][2]) * norm_L2 +
                 coefficient[1][1]) * norm_L2 + coefficient[1][0];
            const float x2 =
                ((coefficient[2][3] * norm_L2 + coefficient[2][2]) * norm_L2 +
                 coefficient[2][1]) * norm_L2 + coefficient[2][0];
            const float x3 =
                ((coefficient[3][3] * norm_L2 + coefficient[3][2]) * norm_L2 +
                 coefficient[3][1]) * norm_L2 + coefficient[3][0];
            rdob.gamma[input][state] =
                ((x3 * norm_L1 + x2) * norm_L1 + x1) * norm_L1 + x0;
        }

        for (uint8_t state = 2U; state < GENERAL_STATE_DIM; ++state)
        {
            const float *coefficient =
                kPositionGainCoefficients[
                    input * (GENERAL_STATE_DIM - 2U) + state - 2U];
            const float x0 =
                ((coefficient[3] * norm_L2 + coefficient[2]) * norm_L2 +
                 coefficient[1]) * norm_L2 + coefficient[0];
            const float x1 =
                (coefficient[6] * norm_L2 + coefficient[5]) * norm_L2 + coefficient[4];
            const float x2 = coefficient[8] * norm_L2 + coefficient[7];
            rdob.bq[input][state - 2U] =
                ((coefficient[9] * norm_L1 + x2) * norm_L1 + x1) * norm_L1 + x0;
        }
    }
}

__attribute__((optimize("O3")))
void wl_chassis_t::_rdob_init()
{
    _ctx.data.rdob = rdob_data_t{};
    rdob_data_t &rdob = _ctx.data.rdob;
    _rdob_coef_fit();

    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        float gamma_v = 0.0f;
        for (uint8_t state = 0; state < GENERAL_STATE_DIM; ++state)
        {
            gamma_v += rdob.gamma[input][state] *
                       (_ctx.data.measured_state.data[2U * state + 1U] -
                        _ctx.data.target_state.data[2U * state + 1U]);
        }
        rdob.z[input] = -gamma_v;
        rdob.dist.data[input] = 0.0f;
        rdob.prev_U0.data[input] = _ctx.data.U0[input];
    }

    // Freeze the start of the first complete observation interval.
    for (uint8_t state = 0; state < GENERAL_STATE_DIM; ++state)
    {
        rdob.prev_q[state] =
            _ctx.data.measured_state.data[2U * state];
        rdob.prev_dot_q[state] =
            _ctx.data.measured_state.data[2U * state + 1U];
        rdob.prev_trim_q[state] =
            _ctx.data.target_state.data[2U * state];
        rdob.prev_trim_dot_q[state] =
            _ctx.data.target_state.data[2U * state + 1U];
    }
    rdob.initialized = true;
}

__attribute__((optimize("O3")))
void wl_chassis_t::_rdob_update()
{
    rdob_data_t &rdob = _ctx.data.rdob;

    if (!rdob.initialized)
    {
        _rdob_init();
        return;
    }
    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        float q_sum = 0.0f;
        for (uint8_t state = 2U; state < GENERAL_STATE_DIM; ++state)
        {
            q_sum += rdob.bq[input][state - 2U] *
                     (rdob.prev_q[state] - rdob.prev_trim_q[state] +
                      _ctx.data.measured_state.data[2U * state] -
                      rdob.prev_trim_q[state]);
        }

        float gamma_v_sum = 0.0f;
        for (uint8_t state = 0; state < GENERAL_STATE_DIM; ++state)
        {
            gamma_v_sum += rdob.gamma[input][state] *
                (rdob.prev_dot_q[state] - rdob.prev_trim_dot_q[state] +
                 _ctx.data.measured_state.data[2U * state + 1U] -
                 rdob.prev_trim_dot_q[state]);
        }

        rdob.z[input] =
            kStepPoles[input] * rdob.z[input] +
            kStepEndpointWeight[input] * q_sum +
            kStepVelocityScale[input] * gamma_v_sum +
            kStepInputGain[input] *
                (_ctx.data.output.data[input] - rdob.prev_U0.data[input]);

        // Reconstruct and limit with the old Gamma before changing schedule.
        float estimate = rdob.z[input];
        for (uint8_t state = 0; state < GENERAL_STATE_DIM; ++state)
        {
            estimate += rdob.gamma[input][state] *
                (_ctx.data.measured_state.data[2U * state + 1U] -
                 rdob.prev_trim_dot_q[state]);
        }
        rdob.dist.data[input] = std::clamp(
            RDOB_DISTURBANCE_LEAK_POLE * estimate,
            -RDOB_DISTURBANCE_LIMIT[input],
            RDOB_DISTURBANCE_LIMIT[input]);
    }

    _rdob_coef_fit();

    // Rebase z so the limited physical estimate is continuous after Gamma or
    // moving-trim changes.
    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        float gamma_v = 0.0f;
        for (uint8_t state = 0; state < GENERAL_STATE_DIM; ++state)
        {
            gamma_v += rdob.gamma[input][state] *
                       (_ctx.data.measured_state.data[2U * state + 1U] -
                        _ctx.data.target_state.data[2U * state + 1U]);
        }
        rdob.z[input] = rdob.dist.data[input] - gamma_v;
        rdob.prev_U0.data[input] = _ctx.data.U0[input];
    }

    // Advance the cached interval start only after the old interval is closed.
    for (uint8_t state = 0; state < GENERAL_STATE_DIM; ++state)
    {
        rdob.prev_q[state] =
            _ctx.data.measured_state.data[2U * state];
        rdob.prev_dot_q[state] =
            _ctx.data.measured_state.data[2U * state + 1U];
        rdob.prev_trim_q[state] =
            _ctx.data.target_state.data[2U * state];
        rdob.prev_trim_dot_q[state] =
            _ctx.data.target_state.data[2U * state + 1U];
    }
}

void wl_chassis_t::_calc_state_deviation()
{
    rdob_data_t &rdob = _ctx.data.rdob;
    const state_vec_t &state = _ctx.data.measured_state;

    // L_bar itself is the scheduled static trim, so its position error is
    // zero. The available leg-position range check uses measured L_bar.
    const float normalized[] = {
        state.theta / RDOB_THETA_SCALE,
        state.dot_theta / RDOB_BODY_RATE_SCALE,
        state.phi / RDOB_PHI_SCALE,
        state.dot_phi / RDOB_BODY_RATE_SCALE,
        state.dot_L / RDOB_LEG_RATE_SCALE,
        (state.beta1 - _ctx.data.target_state.beta1) / RDOB_BETA_SCALE,
        state.dot_beta1 / RDOB_BETA_RATE_SCALE,
        (state.beta2 - _ctx.data.target_state.beta2) / RDOB_BETA_SCALE,
        state.dot_beta2 / RDOB_BETA_RATE_SCALE,
    };
    float deviation_sq = 0.0f;
    for (float value : normalized)
    {
        deviation_sq += value * value;
    }
    rdob.state_deviation = std::sqrt(deviation_sq);

    const float t = std::clamp(
        (rdob.state_deviation - RDOB_DEVIATION_FULL) /
            (RDOB_DEVIATION_OFF - RDOB_DEVIATION_FULL),
        0.0f, 1.0f);
    const float state_ratio = 1.0f - t * t * (3.0f - 2.0f * t);

    // Drop compensation immediately when confidence falls; restore it over
    // about one second without changing the observer state or its inputs.
    rdob.ratio = state_ratio;
}

void wl_chassis_t::_calc_ddot_x_imu()
{
    auto &odom = _ctx.data.odom;
    auto &ins = _ctx.data.ins;
    const auto &state = _ctx.data.measured_state;
    const auto &left = _ctx.data.leg[leg_def::L];
    const auto &right = _ctx.data.leg[leg_def::R];
    const float wx = ins.gyro[2];
    const float wy = ins.gyro[1];
    const float wz = ins.gyro[0];
    const float st = arm_sin_f32(state.theta);
    const float ct = arm_cos_f32(state.theta);
    const float sp = arm_sin_f32(state.phi);
    const float cp = arm_cos_f32(state.phi);
    const float s1 = arm_sin_f32(state.beta1);
    const float c1 = arm_cos_f32(state.beta1);
    const float s2 = arm_sin_f32(state.beta2);
    const float c2 = arm_cos_f32(state.beta2);

    // beta_i = current_leg_rad_i - pi/2 - theta；倾斜时 dot_theta != wy。
    const float dot_theta = cp * wy - sp * wz;
    const float s_bar = 0.5f * (left.current_leg_length * s1
                               + right.current_leg_length * s2);
    const float dot_s_bar = 0.5f * (
        left.current_leg_speed * s1
        + left.current_leg_length * c1 * (left.current_leg_radps - dot_theta)
        + right.current_leg_speed * s2
        + right.current_leg_length * c2 * (right.current_leg_radps - dot_theta));

    static float previous_gyro[3] = {ins.gyro[0],ins.gyro[1],ins.gyro[2]};
    static float previous_dot_s_bar = dot_s_bar;
    static float angular_accel[3] = {};
    static float ddot_s_bar = 0.0f;

    for (uint8_t i = 0; i < 3; ++i)
    {
        angular_accel[i] = IMU_ACCEL_DIFF_A * angular_accel[i]
            + IMU_ACCEL_DIFF_B * (ins.gyro[i] - previous_gyro[i]);
        previous_gyro[i] = ins.gyro[i];
    }
    ddot_s_bar = IMU_ACCEL_DIFF_A * ddot_s_bar
        + IMU_ACCEL_DIFF_B * (dot_s_bar - previous_dot_s_bar);
    previous_dot_s_bar = dot_s_bar;

    // I -> H：减去 alpha x r 和 omega x (omega x r)，重力已由 INS 去除。
    const float a_hx = ins.accel[0] - angular_accel[1] * IMU_DZ
        - (wx * wz * IMU_DZ - (wy * wy + wz * wz) * IMU_DX);
    const float a_hy = ins.accel[1]
        - (angular_accel[0] * IMU_DX - angular_accel[2] * IMU_DZ)
        - (wx * wy * IMU_DX + wy * wz * IMU_DZ);
    const float a_hz = ins.accel[2] + angular_accel[1] * IMU_DX
        - (wx * wz * IMU_DX - (wx * wx + wy * wy) * IMU_DZ);

    // H -> 航向系 -> C：这里使用实际航向角速度，不能使用云台相对转速。
    const float dot_psi = (sp * wy + cp * wz) / ct;
    odom.ddot_x_imu = ct * a_hx + st * (sp * a_hy + cp * a_hz)
        + ddot_s_bar - dot_psi * dot_psi * s_bar;
}


void wl_chassis_t::_calc_ddot_x_model()
{
    const float norm_L1 = std::clamp(
        (_ctx.data.leg[leg_def::L].current_leg_length - MODEL_ACCEL_LENGTH_CENTER) *
            MODEL_ACCEL_LENGTH_INV_SCALE,
        -1.0f, 1.0f);
    const float norm_L2 = std::clamp(
        (_ctx.data.leg[leg_def::R].current_leg_length - MODEL_ACCEL_LENGTH_CENTER) *
            MODEL_ACCEL_LENGTH_INV_SCALE,
        -1.0f, 1.0f);

    for (uint8_t i = 0; i < MODEL_ACCEL_STATE_DIM + INPUT_DIM; ++i)
    {
        const float *coefficient = MODEL_ACCEL_COEF[i];
        const float x0 =
            ((coefficient[3] * norm_L2 + coefficient[2]) * norm_L2 +
             coefficient[1]) * norm_L2 + coefficient[0];
        const float x1 =
            (coefficient[6] * norm_L2 + coefficient[5]) * norm_L2 + coefficient[4];
        const float x2 = coefficient[8] * norm_L2 + coefficient[7];
        const float value =
            ((coefficient[9] * norm_L1 + x2) * norm_L1 + x1) * norm_L1 + x0;
        if (i < MODEL_ACCEL_STATE_DIM)
        {
            _ctx.data.Hxi[i] = value;
        }
        else
        {
            _ctx.data.Bx[i - MODEL_ACCEL_STATE_DIM] = value;
        }
    }

    const state_vec_t &state = _ctx.data.measured_state;
    // _gain_calculate() has refreshed beta0 and active U0 for these lengths.
    // Measured-length scheduling makes delta_L_bar=0; no length-deviation term.
    const float delta_q[MODEL_ACCEL_STATE_DIM] = {
        state.theta, state.phi,
        state.beta1 - _ctx.data.target_state.beta1,
        state.beta2 - _ctx.data.target_state.beta2,
    };
    auto &odom = _ctx.data.odom;
    odom.ddot_x_model = 0.0f;
    for (uint8_t i = 0; i < MODEL_ACCEL_STATE_DIM; ++i)
    {
        odom.ddot_x_model += _ctx.data.Hxi[i] * delta_q[i];
    }
    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        // Both output and U0 contain active leg forces. The same scheduled
        // passive gas force cancels in u_total - u0_total.
        const float delta_u = _ctx.data.output.data[input] - _ctx.data.U0[input];
        odom.ddot_x_model += _ctx.data.Bx[input] * delta_u;
    }
}

void wl_chassis_t::_reset_forward_kf()
{
    const bool wheel_valid = _ctx.motor.wheel[leg_def::L]->is_online() &&
                             _ctx.motor.wheel[leg_def::R]->is_online();
    const auto estimate = _forward_kf.reset(
        0.0f, _ctx.data.odom.wheel_dot_x[0], _ctx.data.odom.ddot_x_imu, wheel_valid);
    _ctx.data.odom.X[0] = estimate.x;
    _ctx.data.odom.X[1] = estimate.v;
    _ctx.data.odom.X[2] = estimate.a;
    _ctx.data.odom.X[3] = estimate.imu_acc_bias;
    _ctx.data.odom.slip_score = estimate.slip_score;
    _ctx.data.measured_state.x = estimate.x;
    _ctx.data.measured_state.dot_x = estimate.v;
}

void wl_chassis_t::_update_forward_kf()
{
    auto &odom = _ctx.data.odom;
    const bool wheel_valid = _ctx.motor.wheel[leg_def::L]->is_online() &&
                             _ctx.motor.wheel[leg_def::R]->is_online();
    const auto estimate = _forward_kf.update(
        odom.ddot_x_imu, odom.wheel_dot_x[0], wheel_valid);
    odom.X[0] = estimate.x;
    odom.X[1] = estimate.v;
    odom.X[2] = estimate.a;
    odom.X[3] = estimate.imu_acc_bias;
    odom.slip_score = estimate.slip_score;
    // Only the first two entries of the 14-state feedback are replaced.
    _ctx.data.measured_state.x = estimate.x;
    _ctx.data.measured_state.dot_x = estimate.v;
}

void wl_chassis_t::_actual_output_update()
{
    _ctx.data.output.data[lqr_input_def::F_L1] = _ctx.data.leg[leg_def::L].actual_out_F_L;
    _ctx.data.output.data[lqr_input_def::F_L2] = _ctx.data.leg[leg_def::R].actual_out_F_L;
    _ctx.data.output.data[lqr_input_def::T_P1] = _ctx.data.leg[leg_def::L].actual_out_T_p;
    _ctx.data.output.data[lqr_input_def::T_P2] = _ctx.data.leg[leg_def::R].actual_out_T_p;
    _ctx.data.output.data[lqr_input_def::T_W1] = _ctx.data.wheel[leg_def::L].out_T_w;
    _ctx.data.output.data[lqr_input_def::T_W2] = _ctx.data.wheel[leg_def::R].out_T_w;
}

void wl_chassis_t::_vmc_trans_v2j()
{
    constexpr float MAX_MOTOR_TORQUE = 40.0f;
    // out_F_L is the active motor-side generalized force.  The passive gas
    // spring acts mechanically and therefore is not added to motor torque here.
    for (auto &leg : _ctx.data.leg)
    {
        float F_L = std::clamp(leg.out_F_L + leg.virtual_wall_force,-MAX_F_L,MAX_F_L);
        float T_p = std::clamp(leg.out_T_p , -MAX_T_P, MAX_T_P);
        float tau_sum                        = T_p;
        float tau_diff                       = F_L * leg.J_L;
        leg.out_joint_torque[joint_def::HIP] = std::clamp(
            (tau_sum - tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        leg.out_joint_torque[joint_def::KNEE] = std::clamp(
            (tau_sum + tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        if (std::isfinite(leg.J_L) && std::fabs(leg.J_L) > 1.0e-6f)
        {
            leg.actual_out_F_L =
                (leg.out_joint_torque[joint_def::KNEE] -
                 leg.out_joint_torque[joint_def::HIP]) /
                leg.J_L;
        }
        else
        {
            leg.actual_out_F_L = 0.0f;
        }
        leg.actual_out_T_p = leg.out_joint_torque[joint_def::KNEE] +
                          leg.out_joint_torque[joint_def::HIP];
    }
}

void wl_chassis_t::_send_joint_torque() const
{
    _ctx.motor.joint[leg_def::L][joint_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[joint_def::HIP]);
    _ctx.motor.joint[leg_def::L][joint_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::L].direction *
        _ctx.data.leg[leg_def::L].out_joint_torque[joint_def::KNEE]);
    _ctx.motor.joint[leg_def::R][joint_def::HIP]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[joint_def::HIP]);
    _ctx.motor.joint[leg_def::R][joint_def::KNEE]->send_torque(
        _ctx.data.leg[leg_def::R].direction *
        _ctx.data.leg[leg_def::R].out_joint_torque[joint_def::KNEE]);

    // _ctx.motor.joint[leg_def::L][joint_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::L][joint_def::KNEE]->send_torque(0);
    // _ctx.motor.joint[leg_def::R][joint_def::HIP]->send_torque(0);
    // _ctx.motor.joint[leg_def::R][joint_def::KNEE]->send_torque(0);
}

void wl_chassis_t::_send_wheel_torque() const
{
    _ctx.motor.wheel[leg_def::L]->send_torque(
        _ctx.data.wheel[leg_def::L].direction *
        _ctx.data.wheel[leg_def::L].out_current);
    _ctx.motor.wheel[leg_def::R]->send_torque(
        _ctx.data.wheel[leg_def::R].direction *
        _ctx.data.wheel[leg_def::R].out_current);

    // _ctx.motor.wheel[leg_def::L]->send_torque(0);
    // _ctx.motor.wheel[leg_def::R]->send_torque(0);
}

} // namespace pyro
