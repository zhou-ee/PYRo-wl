#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{
void wl_chassis_t::fsm_active_t::state_normal_t::state_getover_t::enter(owner *owner)
{
    owner->_getover_target_length[leg_def::L] = std::clamp(
        owner->_ctx.data.leg[leg_def::L].current_leg_length -
            GETOVER_SHORTEN_LENGTH,
        MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    owner->_getover_target_length[leg_def::R] = std::clamp(
        owner->_ctx.data.leg[leg_def::R].current_leg_length -
            GETOVER_SHORTEN_LENGTH,
        MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    owner->_ctx.data.current_function = chassis_function_state_t::GETOVER;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_getover_t::execute(owner *owner)
{
    if (owner->_current_cmd.cmd_function_state != chassis_function_state_t::NONE)
    {
        owner->_getover_active = false;
        request_switch(&owner->_state_active._state_normal._state_balance);
        return;
    }

    owner->_ctx.data.target_state.theta = GETOVER_FORWARD_RAD;
    owner->_ctx.data.target_state.dot_theta = 0.0f;

    /* Keep normal LQR for body, wheel and swing control. */
    owner->_state_active._state_normal._state_balance.execute(owner);

    /* Replace only the leg-length force with a faster direct PD command. */
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = owner->_ctx.data.leg[i];
        leg.target_leg_length = owner->_getover_target_length[i];
        leg.out_F_L = std::clamp(
            GETOVER_FORCE_SCALE * owner->_ctx.pid.leg_length[i]->calculate(
                leg.target_leg_length, leg.current_leg_length,
                leg.current_leg_speed) -
                leg.gas_spring_force,
            -MAX_F_L, MAX_F_L);
    }
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_getover_t::exit(owner *owner)
{
    owner->_getover_active = false;
}
}
