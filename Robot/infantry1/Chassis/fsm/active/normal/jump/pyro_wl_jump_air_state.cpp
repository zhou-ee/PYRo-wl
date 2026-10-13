#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_air_t::enter(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.airborne.L_air_ref[leg_def::L] = data.leg[leg_def::L].current_leg_length;
    data.airborne.L_air_ref[leg_def::R] = data.leg[leg_def::R].current_leg_length;
    data.jump_phase = jump_phase_t::AIR;
    data.jump_phase_time = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::L] = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::R] = 0.0f;
    data.jump_landing_confirm_time = 0.0f;
    owner->_reset_air_contacts();
    for (uint8_t i = 0; i < 2; ++i)
    {
        owner->_ctx.pid.leg_length[i]->clear();
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_air_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_virtual_wall_bypass = false;
    const bool landed = owner->_detect_landing();
    owner->_execute_air_control();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    if (landed)
    {
        // Handover directly after this tick's contact-aware outputs. Balance
        // retains the existing length and attitude recovery on its next entry.
        data.airborne.L_ref = 0.5f * (data.leg[leg_def::L].current_leg_length +
                                     data.leg[leg_def::R].current_leg_length);
        data.target_state.L = data.airborne.L_ref;
        data.target_state.dot_L = 0.0f;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        data.airborne.landing_recovery = true;
        data.jump_phase = jump_phase_t::COMPLETE;
        data.jump_phase_time = 0.0f;
        owner->_state_active._state_normal._state_jump._request_balance(owner);
    }
    else if (data.jump_phase_time >= JUMP_TOTAL_TIMEOUT)
    {
        owner->_state_active._state_normal._state_jump._abort_jump(owner);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_air_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
