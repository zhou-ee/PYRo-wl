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
    for (uint8_t i = 0; i < 2; ++i)
    {
        data.airborne.L_air_ref[i] = JUMP_AIR_LENGTH;
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_recovery_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    owner->_execute_airborne_lqr();
    bool settled = true;
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = data.leg[i];
        // The reference is for phase completion/telemetry, not a length loop.
        leg.target_leg_length = JUMP_AIR_LENGTH;
        leg.out_F_L = owner->_motor_torque_to_leg_force(leg, -JUMP_RECOVERY_MOTOR_TORQUE)
            - (LEG_MASS + WHEEL_MASS) * GRAVITY_ACCELERATION
            - leg.gas_spring_force;
        data.wheel[i].out_T_w = 0.0f;
        data.wheel[i].out_current = 0.0f;
        // Retraction effort continues until handover; the lower virtual wall
        // provides the opposing force when the leg reaches its short end.
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
