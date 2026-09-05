#include "pyro_wl_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "dsp/fast_math_functions.h"
#include "gas_spring_compensation_table.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

namespace
{
struct gas_spring_lookup_t
{
    float f_l = 0.0f;
    float t_p = 0.0f;
    bool valid = false;
};

struct lookup_interval_t
{
    uint16_t index = 0;
    float alpha = 0.0f;
};

lookup_interval_t find_lookup_interval(const float *axis, uint16_t count,
                                       float value)
{
    if (axis == nullptr || count == 0u || !std::isfinite(value))
    {
        return {};
    }
    if (count == 1u || value <= axis[0])
    {
        return {0u, 0.0f};
    }
    if (value >= axis[count - 1u])
    {
        return {static_cast<uint16_t>(count - 2u), 1.0f};
    }
    for (uint16_t index = 0u; index + 1u < count; ++index)
    {
        if (value <= axis[index + 1u])
        {
            const float span = axis[index + 1u] - axis[index];
            const float alpha = (span > 0.0f)
                                    ? (value - axis[index]) / span
                                    : 0.0f;
            return {index, std::clamp(alpha, 0.0f, 1.0f)};
        }
    }
    return {static_cast<uint16_t>(count - 2u), 1.0f};
}

float bilinear_value(const float *values, uint16_t length_count,
                     uint16_t beta_count, lookup_interval_t length_interval,
                     lookup_interval_t beta_interval)
{
    if (values == nullptr || length_count == 0u || beta_count == 0u)
    {
        return 0.0f;
    }
    const uint16_t i0 = length_interval.index;
    const uint16_t j0 = beta_interval.index;
    const uint16_t i1 = (length_count > 1u) ? static_cast<uint16_t>(i0 + 1u) : i0;
    const uint16_t j1 = (beta_count > 1u) ? static_cast<uint16_t>(j0 + 1u) : j0;
    const uint32_t row0 = static_cast<uint32_t>(i0) * beta_count;
    const uint32_t row1 = static_cast<uint32_t>(i1) * beta_count;
    const float v00 = values[row0 + j0];
    const float v01 = values[row0 + j1];
    const float v10 = values[row1 + j0];
    const float v11 = values[row1 + j1];
    const float along_beta_0 = v00 + beta_interval.alpha * (v01 - v00);
    const float along_beta_1 = v10 + beta_interval.alpha * (v11 - v10);
    return along_beta_0 + length_interval.alpha * (along_beta_1 - along_beta_0);
}

gas_spring_lookup_t lookup_gas_spring(const gas_spring_table::table_t &table,
                                      float leg_length, float beta)
{
    if (table.lengths == nullptr || table.betas == nullptr ||
        table.f_gas_l == nullptr || table.t_gas_beta == nullptr ||
        table.length_count == 0u || table.beta_count == 0u)
    {
        return {};
    }
    const lookup_interval_t length_interval =
        find_lookup_interval(table.lengths, table.length_count, leg_length);
    const lookup_interval_t beta_interval =
        find_lookup_interval(table.betas, table.beta_count, beta);
    return {
        bilinear_value(table.f_gas_l, table.length_count, table.beta_count,
                       length_interval, beta_interval),
        bilinear_value(table.t_gas_beta, table.length_count, table.beta_count,
                       length_interval, beta_interval),
        true};
}
} // namespace

wl_chassis_t::wl_chassis_t() : module_base_t("wl_chassis")
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

    // 4. Convert joint-space feedback to virtual-mechanism feedback.
    _vmc_trans_j2v();

    // 5. Update wheel odometry.
    _ctx.data.odom.real_dot_x[1] = _ctx.data.odom.real_dot_x[0];
    _ctx.data.odom.real_dot_x[0] = 0.5f * WHEEL_RADIUS *
                                   (_ctx.data.wheel[leg_def::L].current_radps +
                                    _ctx.data.wheel[leg_def::R].current_radps);
    _ctx.data.odom.real_x +=
        0.5f * (_ctx.data.odom.real_dot_x[0] + _ctx.data.odom.real_dot_x[1]) *
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
    state_vec_t &state = _ctx.data.current_state;

    state.x            = _ctx.data.odom.real_x;
    state.dot_x        = (_ctx.data.odom.real_dot_x[0] + _ctx.data.odom.real_dot_x[1]) / 2;
    state.psi          = _ctx.data.ins.euler_rad[0];
    state.dot_psi      = _ctx.data.ins.gyro[0];
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

    _update_accel_heading_frame();
    // Refresh with the current body pitch before support-force estimation.
    // The takeoff detector must account for force carried by the gas spring,
    // otherwise compensation lowers motor torque and looks like loss of contact.
    _apply_gas_spring_compensation();
    _calc_support_force();

    _ctx.motor.yaw->update_feedback();
    _ctx.data.yaw.pos = _module_deps.motor.yaw->get_current_position();
}


void wl_chassis_t::_fsm_execute()
{
    static pyro::chassis_function_state_t last_cmd_state = pyro::chassis_function_state_t::NONE;
    if( _current_cmd.cmd_function_state == pyro::chassis_function_state_t::RESTART &&
        last_cmd_state != _current_cmd.cmd_function_state)
    {
        _ctx.data.flag.leg_is_should_restart = false;
    }

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
        leg.gas_spring_force   = _calc_gas_spring_force(
            leg.current_leg_length);
        leg.current_leg_speed = dot_theta * leg.J_L;

        const float raw_beta  = leg.current_joint_rad[joint_def::HIP] + theta;
        const float beta      = loop_fp32_constrain(raw_beta, -PI, PI);
        leg.current_leg_rad   = beta;
        leg.current_leg_radps = (leg.current_joint_radps[joint_def::KNEE] +
                                 leg.current_joint_radps[joint_def::HIP]) /
                                2;
        leg.current_F_L = (leg.current_joint_torque[joint_def::KNEE] -
                           leg.current_joint_torque[joint_def::HIP]) /
                          leg.J_L;
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
    - _ctx.data.leg[leg_def::L].gas_spring_force;
    
    _ctx.data.leg[leg_def::R].out_F_L =
        _ctx.pid.leg_length[leg_def::R]->calculate(
            _ctx.data.leg[leg_def::R].target_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_length,
            _ctx.data.leg[leg_def::R].current_leg_speed)
    - _ctx.data.leg[leg_def::R].gas_spring_force;


    float ll_target_radps;
    ll_target_radps = _ctx.pid.leg_control_rad[leg_def::L]->calculate(
        0.0f, _ctx.data.leg[leg_def::L].error_leg_rad);
    _ctx.data.leg[leg_def::L].out_T_p = _ctx.pid.leg_control_radps[leg_def::L]->calculate
        (ll_target_radps, _ctx.data.leg[leg_def::L].current_leg_radps);
    
    float rl_target_radps;
    rl_target_radps = _ctx.pid.leg_control_rad[leg_def::R]->calculate(
    0.0f, _ctx.data.leg[leg_def::R].error_leg_rad);
    _ctx.data.leg[leg_def::R].out_T_p = _ctx.pid.leg_control_radps[leg_def::L]->calculate
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
    return GAS_SPRING_COMPENSATION_SCALE *
           evaluate_polynomial_ascending(
               normalized_length, GAS_SPRING_FORCE_POLY_COEF,
               GAS_SPRING_FORCE_POLY_DEGREE);
}

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
                       _ctx.data.current_state.data[state];
    }
    error[lqr_state_def::PSI] =
        loop_fp32_constrain(error[lqr_state_def::PSI], -PI, PI);

    // calculate control data

    for (uint8_t input = 0; input < INPUT_DIM; ++input)
    {
        _ctx.data.control.data[input] = _ctx.data.U0[input];
        for (uint8_t state = 0; state < STATE_DIM; ++state)
        {
            _ctx.data.control.data[input] +=
                _ctx.data.K[input][state] * error[state];
        }
    }

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

void wl_chassis_t::_apply_gas_spring_compensation()
{
    _ctx.data.gas_spring_compensation_active = false;
    for (auto &leg : _ctx.data.leg)
    {
        leg.gas_f_l = 0.0f;
        leg.gas_t_p = 0.0f;
    }

    if constexpr (!GAS_SPRING_COMPENSATION_ENABLE)
    {
        return;
    }

    constexpr float HALF_PI = PI / 2.0f;
    const gas_spring_table::table_t *tables[2] = {
        &gas_spring_table::LEFT, &gas_spring_table::RIGHT};
    bool any_valid = false;
    for (uint8_t side = 0u; side < 2u; ++side)
    {
        leg_ctx_t &leg = _ctx.data.leg[side];
        // The SolidWorks table is expressed in the body-fixed GS_BASE frame.
        // current_leg_rad is world-referenced, so remove body pitch just as
        // the LQR state construction does before looking up beta.
        const float beta = loop_fp32_constrain(
            leg.current_leg_rad - HALF_PI - _ctx.data.ins.euler_rad[1], -PI,
            PI);
        const gas_spring_lookup_t lookup =
            lookup_gas_spring(*tables[side], leg.current_leg_length, beta);
        if (!lookup.valid)
        {
            continue;
        }
        leg.gas_f_l = GAS_SPRING_COMPENSATION_SCALE * lookup.f_l;
        leg.gas_t_p = GAS_SPRING_COMPENSATION_SCALE * lookup.t_p;
        any_valid = true;
    }
    _ctx.data.gas_spring_compensation_active = any_valid;
}


void wl_chassis_t::_vmc_trans_v2j()
{
    constexpr float MAX_MOTOR_TORQUE = 40.0f;

    // Every active mode eventually reaches this VMC-to-joint conversion.
    // Apply the passive gas-spring contribution here so ALIGN, MANUAL, AIR,
    // STEP and BALANCE all use the same motor-side compensation.
    _apply_gas_spring_compensation();

    for (auto &leg : _ctx.data.leg)
    {
        leg.virtual_wall_force = _calc_leg_length_wall_force(leg);
<<<<<<< HEAD
        const float f_l_before_wall =
            leg.out_F_L - (_ctx.data.gas_spring_compensation_active ? leg.gas_f_l : 0.0f);
        const float t_p_motor =
            leg.out_T_p - (_ctx.data.gas_spring_compensation_active ? leg.gas_t_p : 0.0f);
        leg.motor_f_l = std::clamp(f_l_before_wall + leg.virtual_wall_force,
                                   -MAX_F_L, MAX_F_L);
        leg.motor_t_p = t_p_motor;
=======
        leg.out_F_L = std::clamp(
            leg.out_F_L,
            -MAX_F_L, MAX_F_L);
>>>>>>> upstream/double_model

        float tau_sum                        = leg.motor_t_p;
        float tau_diff                       = leg.motor_f_l * leg.J_L;
        leg.out_joint_torque[joint_def::HIP] = std::clamp(
            (tau_sum - tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
        leg.out_joint_torque[joint_def::KNEE] = std::clamp(
            (tau_sum + tau_diff) / 2, -MAX_MOTOR_TORQUE, MAX_MOTOR_TORQUE);
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
