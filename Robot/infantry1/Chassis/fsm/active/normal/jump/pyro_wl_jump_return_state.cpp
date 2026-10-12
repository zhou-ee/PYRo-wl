#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_return_t::enter(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase = jump_phase_t::RETURN;
    data.jump_phase_time = 0.0f;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_return_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.airborne.L_ref = 0.5f * (data.leg[leg_def::L].current_leg_length + data.leg[leg_def::R].current_leg_length);
    data.target_state.L = data.airborne.L_ref;
    data.target_state.dot_L = 0.0f;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    data.airborne.landing_recovery = true;
    data.jump_virtual_wall_bypass = false;
    owner->_state_active._state_normal._state_jump._request_balance(owner);
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_return_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
