#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_recovery_t::enter(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase = jump_phase_t::RECOVERY;
    data.jump_phase_time = 0.0f;
    data.jump_recovery_hold_time = 0.0f;
    owner->_prepare_air_length_control();
    for (uint8_t i = 0; i < 2; ++i)
    {
        data.airborne.L_air_ref[i] = data.leg[i].current_leg_length;
        owner->_ctx.pid.leg_length[i]->clear();
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_recovery_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_virtual_wall_bypass = false;
    owner->_execute_airborne_lqr();
    bool settled = true;
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = data.leg[i];
        float &reference = data.airborne.L_air_ref[i];
        const float step = JUMP_RECOVERY_LENGTH_RATE * data._dt;
        reference += std::clamp(JUMP_AIR_LENGTH - reference, -step, step);
        leg.target_leg_length = reference;
        // Eq.139 is a total virtual force; convert once to active motor force.
        const float total_force = std::clamp(owner->_ctx.pid.leg_length[i]->calculate(
            reference, leg.current_leg_length, leg.current_leg_speed),
            -JUMP_RECOVERY_FORCE_MAX, JUMP_RECOVERY_FORCE_MAX)
            - (LEG_MASS + WHEEL_MASS) * GRAVITY_ACCELERATION;
        leg.out_F_L = total_force - leg.gas_spring_force;
        data.wheel[i].out_T_w = 0.0f;
        data.wheel[i].out_current = 0.0f;
        // A slightly shorter leg has completed retraction as well. The existing
        // zero-integral PD plus negative foot feedforward has a static offset.
        settled &= leg.current_leg_length < JUMP_AIR_LENGTH + JUMP_LENGTH_TOLERANCE &&
                   std::fabs(leg.current_leg_speed) < JUMP_SETTLED_LEG_SPEED;
    }
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    data.jump_recovery_hold_time = settled ? data.jump_recovery_hold_time + data._dt : 0.0f;
    if (data.jump_recovery_hold_time >= JUMP_RECOVERY_HOLD_TIME ||
        data.jump_phase_time >= JUMP_RECOVERY_TIMEOUT)
    {
        // On timeout, keep attitude control and prepare landing rather than
        // continuing to retract indefinitely.
        auto &jump = owner->_state_active._state_normal._state_jump;
        jump._transition_phase(owner, &jump._state_air);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_recovery_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
