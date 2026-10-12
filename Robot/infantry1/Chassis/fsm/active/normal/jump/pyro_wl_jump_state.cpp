#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::on_enter(wl_chassis_t *owner)
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
    data.jump_virtual_wall_bypass = true;
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
    _active_state = &_state_pre_compress;
    _target_state = nullptr;
    _last_state = nullptr;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::on_execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase_time += data._dt;
    // Preserve the original per-tick integration before the active phase runs.
    data.jump_vertical_velocity_integral +=
        data.airborne.accel_z_y_lpf * data._dt;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::_transition_phase(
    wl_chassis_t *owner, state_t<wl_chassis_t> *next)
{
    // The old switch changed phase after sending this tick's outputs. Enter the
    // next child now, but execute it on the next tick. Using change_state() here
    // would insert a transition-only tick in fsm_t::execute().
    if (_active_state)
    {
        _active_state->exit(owner);
    }
    _last_state = _active_state;
    _active_state = next;
    _target_state = nullptr;
    _active_state->enter(owner);
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::_abort_jump(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_virtual_wall_bypass = false;
    data.airborne.landing_recovery = false;
    for (uint8_t i = 0; i < 2; ++i)
    {
        data.leg[i].out_F_L = 0.0f;
        data.leg[i].out_T_p = 0.0f;
        data.leg[i].out_joint_torque[joint_def::HIP] = 0.0f;
        data.leg[i].out_joint_torque[joint_def::KNEE] = 0.0f;
        data.wheel[i].out_T_w = 0.0f;
        data.wheel[i].out_current = 0.0f;
    }
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    // Keep the existing NORMAL-level exit request, including its destination.
    // Routing this through the child would add a level and change behavior.
    request_switch(&owner->_state_passive);
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::_request_balance(wl_chassis_t *owner)
{
    request_switch(&owner->_state_active._state_normal._state_balance);
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::on_exit(wl_chassis_t *owner)
{
    owner->_ctx.data.jump_virtual_wall_bypass = false;
}

} // namespace pyro
