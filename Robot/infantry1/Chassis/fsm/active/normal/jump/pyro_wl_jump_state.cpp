#include "pyro_wl_chassis.h"
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::on_enter(wl_chassis_t *owner)
{
    _active_state = &_state_pre_compress;
    _target_state = nullptr;
    _last_state = nullptr;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::on_execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    if (!std::isfinite(data._dt) || data._dt <= 0.0f)
    {
        _abort_jump(owner);
        _active_state = nullptr;
        return;
    }
    for (const float value : data.measured_state.data)
    {
        if (!std::isfinite(value))
        {
            _abort_jump(owner);
            _active_state = nullptr;
            return;
        }
    }
    data.jump_phase_time += data._dt;
    // Preserve the original per-tick integration before the active phase runs.
    data.jump_vertical_velocity_integral +=
        data.airborne.accel_z_y_lpf * data._dt;
    for (const auto &leg : data.leg)
    {
        if (!std::isfinite(leg.current_leg_length) || !std::isfinite(leg.current_leg_speed) ||
            !std::isfinite(leg.gas_spring_force) || !std::isfinite(leg.J_L) ||
            std::fabs(leg.J_L) < 1.0e-4f)
        {
            _abort_jump(owner);
            _active_state = nullptr;
            return;
        }
    }
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
    data.jump_cooldown_remaining = JUMP_COOLDOWN_TIME;
    data.flag.leg_is_should_restart = true;
    // Faults leave at the main FSM level and require the existing RESTART command.
    owner->_main_fsm.change_state(&owner->_state_passive);
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::_request_balance(wl_chassis_t *owner)
{
    request_switch(&owner->_state_active._state_normal._state_balance);
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::on_exit(wl_chassis_t *owner)
{
    owner->_ctx.data.jump_cooldown_remaining = JUMP_COOLDOWN_TIME;
}

} // namespace pyro
