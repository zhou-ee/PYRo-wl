#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_pre_compress_t::enter(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase = jump_phase_t::PRE_COMPRESS;
    data.jump_phase_time = 0.0f;
    data.jump_heading_ref = data.measured_state.psi;
    data.jump_forward_velocity_ref = data.measured_state.dot_x;
    data.jump_vertical_velocity_integral = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::L] = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::R] = 0.0f;
    data.jump_landing_confirm_time = 0.0f;
    data.jump_push_accel_hold_ticks = 0;
    data.airborne.takeoff_counter = 0;
    data.airborne.landing_counter = 0;
    data.airborne.landing_recovery = false;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    data.target_state.x = data.measured_state.x;
    data.target_state.dot_x = data.jump_forward_velocity_ref;
    data.target_state.theta = 0.0f;
    data.target_state.dot_theta = 0.0f;
    data.target_state.phi = 0.0f;
    data.target_state.dot_phi = 0.0f;
    data.target_state.beta1 = 0.0f;
    data.target_state.dot_beta1 = 0.0f;
    data.target_state.beta2 = 0.0f;
    data.target_state.dot_beta2 = 0.0f;
    data.target_state.dot_L = 0.0f;
    data.jump_recovery_hold_time = 0.0f;
    data.airborne.attitude_recovery = false;
    data.airborne.velocity_recovery = false;
    data.jump_precompress_exit_velocity = 0.0f;
    owner->_reset_air_contacts();
    data.airborne.forward_velocity_ref = data.jump_forward_velocity_ref;
    for (uint8_t i = 0; i < 2; ++i)
    {
        owner->_ctx.pid.leg_length[i]->clear();
        owner->_ctx.pid.leg_control_rad[i]->clear();
        owner->_ctx.pid.leg_control_radps[i]->clear();
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_pre_compress_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.target_state.L = JUMP_PRECOMPRESS_LENGTH;
    data.target_state.dot_L = 0.0f;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    // 保持进入跳跃时的前进速度，避免预压缩阶段突然刹车产生前翻力矩。
    data.target_state.x += data.jump_forward_velocity_ref * data._dt;
    data.target_state.dot_x = data.jump_forward_velocity_ref;
    owner->_gain_calculate();
    owner->_balance_control();
    for (auto &leg : data.leg)
    {
        // Keep ground balance and add a fixed per-motor retraction effort.
        leg.out_F_L += owner->_motor_torque_to_leg_force(
            leg, -JUMP_PRECOMPRESS_MOTOR_TORQUE);
    }
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    if ((std::fabs(data.leg[leg_def::L].current_leg_length - JUMP_PRECOMPRESS_LENGTH) < JUMP_LENGTH_TOLERANCE &&
         std::fabs(data.leg[leg_def::R].current_leg_length - JUMP_PRECOMPRESS_LENGTH) < JUMP_LENGTH_TOLERANCE &&
         std::fabs(data.leg[leg_def::L].current_leg_speed) < JUMP_SETTLED_LEG_SPEED &&
         std::fabs(data.leg[leg_def::R].current_leg_speed) < JUMP_SETTLED_LEG_SPEED && std::fabs(data.leg[leg_def::L].current_leg_length -data.leg[leg_def::R].current_leg_length) < 0.02f) ||
        data.jump_phase_time >= JUMP_PRECOMPRESS_TIME)
    {
        // Sample while the wheels are still grounded, at the actual handover
        // to PUSH. Keep the existing ground/PUSH reference separate.
        data.jump_precompress_exit_velocity = data.measured_state.dot_x;
        data.airborne.forward_velocity_ref =
            JUMP_LANDING_VELOCITY_SCALE * data.jump_precompress_exit_velocity;
        auto &jump = owner->_state_active._state_normal._state_jump;
        jump._transition_phase(owner, &jump._state_push);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_pre_compress_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
