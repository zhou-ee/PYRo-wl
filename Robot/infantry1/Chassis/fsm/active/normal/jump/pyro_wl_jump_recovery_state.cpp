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
    data.jump_push_accel_hold_ticks = 0;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_recovery_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_virtual_wall_bypass = true;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = data.leg[i];
        leg.target_leg_length = JUMP_AIR_LENGTH;
        // 与 JUMP 的 2 * JUMP_TAU / J_L 一致，正号恢复力矩用于反向收腿。
        const float recovery_force = 2.0f * JUMP_RECOVERY_TAU / leg.J_L;
        leg.out_F_L = recovery_force;
        data.leg[i].out_T_p = 0.0f;
        data.wheel[i].out_T_w = 0.0f;
        data.wheel[i].out_current = 0.0f;
    }
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    const bool falling_fast_enough =
        data.jump_vertical_velocity_integral <= -JUMP_RECOVERY_FALL_SPEED;
    if (falling_fast_enough ||
        data.jump_phase_time >= JUMP_RECOVERY_TIMEOUT)
    {
        auto &jump = owner->_state_active._state_normal._state_jump;
        jump._transition_phase(owner, &jump._state_air);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_recovery_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
