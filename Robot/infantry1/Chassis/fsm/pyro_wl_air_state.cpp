#include "pyro_wl_chassis.h"

#include "dsp/fast_math_functions.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::_update_accel_heading_frame()
{
    const float theta = _ctx.data.ins.euler_rad[1];
    const float phi   = _ctx.data.ins.euler_rad[2];
    const float sin_theta = arm_sin_f32(theta);
    const float cos_theta = arm_cos_f32(theta);
    const float sin_phi = arm_sin_f32(phi);
    const float cos_phi = arm_cos_f32(phi);
    const float ax_b = _ctx.data.ins.accel[0];
    const float ay_b = _ctx.data.ins.accel[1];
    const float az_b = _ctx.data.ins.accel[2];

    _ctx.data.airborne.accel_z_y =
        -sin_theta * ax_b
        + cos_theta * sin_phi * ay_b
        + cos_theta * cos_phi * az_b;

    const float tau = AIR_ACCEL_LPF_TAU;
    const float alpha = _ctx.data._dt / (tau + _ctx.data._dt);
    _ctx.data.airborne.accel_z_y_lpf +=
        alpha * (_ctx.data.airborne.accel_z_y -
                 _ctx.data.airborne.accel_z_y_lpf);
}

void wl_chassis_t::_calc_support_force()
{
    const float support_alpha =
        _ctx.data._dt / (SUPPORT_FORCE_LPF_TAU + _ctx.data._dt);
    for (uint8_t i = 0; i < 2; ++i)
    {
        const leg_ctx_t &leg = _ctx.data.leg[i];
        const float beta =
            leg.current_leg_rad - PI / 2.0f - _ctx.data.ins.euler_rad[1];
        const float length = std::max(leg.current_leg_length, 0.05f);
        const float sin_beta = arm_sin_f32(beta);
        const float cos_beta = arm_cos_f32(beta);
        const float beta_dot =
            leg.current_leg_radps - _ctx.data.ins.gyro[1];
        const float leg_endpoint_accel =
            _ctx.data.airborne.accel_z_y
            - leg.current_leg_accel * cos_beta
            + 2.0f * leg.current_leg_speed *
                  beta_dot * sin_beta
            + length * leg.current_leg_rad_accel * sin_beta
            + length * beta_dot * beta_dot *
                  cos_beta;

        const float support_force_raw =
            (leg.current_F_L) * cos_beta;
            // + leg.current_T_p / length * sin_beta
            // + SUPPORT_FORCE_EFFECTIVE_MASS * leg_endpoint_accel;
        _ctx.data.airborne.support_force[i] +=
            support_alpha *
                (support_force_raw + SUPPORT_FORCE_BIAS[i] -
                 _ctx.data.airborne.support_force[i]);
    }
    _ctx.data.airborne.support_force_sum =
        _ctx.data.airborne.support_force[0] +
        _ctx.data.airborne.support_force[1];
}

bool wl_chassis_t::_detect_takeoff()
{
    const bool low_support =
        _ctx.data.airborne.support_force[leg_def::L] <
            AIR_TAKEOFF_FORCE_ON &&
        _ctx.data.airborne.support_force[leg_def::R] <
            AIR_TAKEOFF_FORCE_ON;
    if (low_support)
    {
        if (_ctx.data.airborne.takeoff_counter <
            AIR_TAKEOFF_DEBOUNCE_TICKS)
        {
            ++_ctx.data.airborne.takeoff_counter;
        }
    }
    else
    {
        _ctx.data.airborne.takeoff_counter = 0;
    }
    return _ctx.data.airborne.takeoff_counter >=
           AIR_TAKEOFF_DEBOUNCE_TICKS;
}

bool wl_chassis_t::_detect_landing()
{
    const bool impact =
        _ctx.data.airborne.accel_z_y_lpf > AIR_LANDING_ACC_ON;
    const bool compressed =
        _ctx.data.leg[leg_def::L].current_leg_length <
            _ctx.data.airborne.L_air_ref[leg_def::L] -
                AIR_LANDING_COMPRESSION ||
        _ctx.data.leg[leg_def::R].current_leg_length <
            _ctx.data.airborne.L_air_ref[leg_def::R] -
                AIR_LANDING_COMPRESSION;
    if (impact && compressed)
    {
        if (_ctx.data.airborne.landing_counter <
            AIR_LANDING_DEBOUNCE_TICKS)
        {
            ++_ctx.data.airborne.landing_counter;
        }
    }
    else
    {
        _ctx.data.airborne.landing_counter = 0;
    }
    return _ctx.data.airborne.landing_counter >=
           AIR_LANDING_DEBOUNCE_TICKS;
}

void wl_chassis_t::_execute_landing_recovery()
{
    if (!_ctx.data.airborne.landing_recovery)
    {
        return;
    }

    const float error =
        NORMAL_LENGTH_TARGET - _ctx.data.vector.target_state[state_def::L];
    const float max_step = AIR_LENGTH_RECOVERY_RATE * _ctx.data._dt;
    const float step = std::clamp(error, -max_step, max_step);
    _ctx.data.vector.target_state[state_def::L] += step;
    _ctx.data.vector.target_state[state_def::DOT_L] =
        (_ctx.data._dt > 1.0e-5f) ? step / _ctx.data._dt : 0.0f;

    if (std::fabs(error) <= AIR_LENGTH_RECOVERY_EPSILON &&
        _ctx.data.airborne.support_force_sum > AIR_CONTACT_FORCE_OFF)
    {
        _ctx.data.vector.target_state[state_def::L] = NORMAL_LENGTH_TARGET;
        _ctx.data.vector.target_state[state_def::DOT_L] = 0.0f;
        _ctx.data.airborne.landing_recovery = false;
    }
}

void wl_chassis_t::_execute_air_control()
{
    _fit_params();
    for (uint8_t i = 0; i < 2; ++i)
    {
        leg_ctx_t &leg = _ctx.data.leg[i];
        const float length_error =
            AIR_LENGTH_TARGET - _ctx.data.airborne.L_air_ref[i];
        const float max_step = AIR_LENGTH_RATE * _ctx.data._dt;
        _ctx.data.airborne.L_air_ref[i] +=
            std::clamp(length_error, -max_step, max_step);
        leg.target_leg_length = _ctx.data.airborne.L_air_ref[i];
        leg.out_F_L = _ctx.pid.leg_length[i]->calculate(
            leg.target_leg_length, leg.current_leg_length,
            leg.current_leg_speed);

        const uint8_t beta_index =
            (i == leg_def::L) ? state_def::BETA_1 :
                                state_def::BETA_2;
        const uint8_t dot_beta_index =
            (i == leg_def::L) ? state_def::DOT_BETA_1 :
                                state_def::DOT_BETA_2;
        const uint8_t input_index =
            (i == leg_def::L) ? input_def::T_P1 :
                                input_def::T_P2;
        const float beta_error = -_ctx.data.vector.measured_state[beta_index];
        const float dot_beta_error =
            -_ctx.data.vector.measured_state[dot_beta_index];
        leg.out_T_p = std::clamp(
            _ctx.data.matrix.K(input_index, beta_index) * beta_error +
                _ctx.data.matrix.K(input_index, dot_beta_index) *
                    dot_beta_error,
            -MAX_T_P, MAX_T_P);

        _ctx.data.wheel[i].out_T_w = std::clamp(
            -AIR_WHEEL_LOCK_K * _ctx.data.wheel[i].current_radps,
            -MAX_T_W, MAX_T_W);
        _ctx.data.wheel[i].out_current =
            std::clamp(_ctx.data.wheel[i].out_T_w / K_t,
                       -MAX_CURRENT, MAX_CURRENT);
    }
}

void wl_chassis_t::fsm_active_t::state_air_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.airborne.state = chassis_state_t::AIR;
    owner->_ctx.data.airborne.landing_counter = 0;
    owner->_ctx.data.airborne.L_ref =
        0.5f * (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                owner->_ctx.data.leg[leg_def::R].current_leg_length);
    for (uint8_t i = 0; i < 2; ++i)
    {
        owner->_ctx.data.airborne.L_air_ref[i] =
            owner->_ctx.data.leg[i].current_leg_length;
        owner->_ctx.data.leg[i].target_leg_length =
            owner->_ctx.data.airborne.L_air_ref[i];
        owner->_ctx.data.leg[i].out_F_L = 0.0f;
        owner->_ctx.data.leg[i].out_T_p = 0.0f;
        owner->_ctx.data.wheel[i].out_T_w = 0.0f;
    }
}

void wl_chassis_t::fsm_active_t::state_air_t::execute(wl_chassis_t *owner)
{
    if (owner->_detect_landing())
    {
        owner->_ctx.data.airborne.state = chassis_state_t::NORMAL;
        owner->_ctx.data.airborne.landing_recovery = true;
        owner->_ctx.data.airborne.L_ref =
            0.5f * (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                    owner->_ctx.data.leg[leg_def::R].current_leg_length);
        owner->_ctx.data.vector.target_state[state_def::L] =
            owner->_ctx.data.airborne.L_ref;
        owner->_ctx.data.vector.target_state[state_def::DOT_L] = 0.0f;
        owner->_ctx.data.airborne.takeoff_counter = 0;
        owner->_ctx.data.airborne.landing_counter = 0;
        owner->_state_active.request_normal();
        return;
    }

    owner->_execute_air_control();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_air_t::exit(wl_chassis_t *owner)
{
    for (auto &wheel : owner->_ctx.data.wheel)
    {
        wheel.out_T_w = 0.0f;
        wheel.out_current = 0.0f;
    }
}

} // namespace pyro
