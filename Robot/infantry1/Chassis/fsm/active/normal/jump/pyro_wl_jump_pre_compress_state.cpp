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
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_pre_compress_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_virtual_wall_bypass = true;
    data.target_state.L = JUMP_PRECOMPRESS_LENGTH;
    data.target_state.dot_L = 0.0f;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    // 保持进入跳跃时的前进速度，避免预压缩阶段突然刹车产生前翻力矩。
    data.target_state.x += data.jump_forward_velocity_ref * data._dt;
    data.target_state.dot_x = data.jump_forward_velocity_ref;
    // data.target_state.theta = 20.0f/180.0f*PI;
    owner->_gain_calculate();
    owner->_balance_control();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    if ((std::fabs(data.leg[leg_def::L].current_leg_length - JUMP_PRECOMPRESS_LENGTH) < 0.015f &&
         std::fabs(data.leg[leg_def::R].current_leg_length - JUMP_PRECOMPRESS_LENGTH) < 0.015f) ||
        data.jump_phase_time >= JUMP_PRECOMPRESS_TIME)
    {
        auto &jump = owner->_state_active._state_normal._state_jump;
        jump._transition_phase(owner, &jump._state_push);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_pre_compress_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
