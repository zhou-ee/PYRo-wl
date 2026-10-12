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
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_air_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_virtual_wall_bypass = true;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    bool leg_speed_reversed = false;
    bool leg_extension_slowed = false;
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = data.leg[i];
        data.jump_leg_extension_speed_peak[i] = std::max(
            data.jump_leg_extension_speed_peak[i],
            leg.current_leg_speed);
        const float extension_speed_peak =
            data.jump_leg_extension_speed_peak[i];

        // 正速度表示伸腿，负速度表示收腿。
        leg_speed_reversed |=
            leg.current_leg_speed < JUMP_LANDING_REVERSE_SPEED;
        leg_extension_slowed |=
            leg.current_leg_length <
                MAX_LEG_LENGTH - JUMP_LANDING_LENGTH_MARGIN &&
            extension_speed_peak > JUMP_LANDING_EXTENSION_SPEED_MIN &&
            extension_speed_peak - leg.current_leg_speed >
                JUMP_LANDING_EXTENSION_SPEED_DROP;

        const float air_force = 2.0f * JUMP_AIR_TAU / leg.J_L;
        leg.out_F_L = std::clamp(air_force, -500.0f, 500.0f);
        data.leg[i].out_T_p = 0.0f;
        data.wheel[i].out_T_w = 0.0f;
        data.wheel[i].out_current = 0.0f;
    }
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    const bool landing_candidate = leg_speed_reversed ||
                                   leg_extension_slowed;
    data.jump_landing_confirm_time = landing_candidate
                                         ? data.jump_landing_confirm_time + data._dt
                                         : 0.0f;
    if (data.jump_landing_confirm_time >= JUMP_LANDING_CONFIRM_TIME)
    {
        auto &jump = owner->_state_active._state_normal._state_jump;
        jump._transition_phase(owner, &jump._state_return);
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
