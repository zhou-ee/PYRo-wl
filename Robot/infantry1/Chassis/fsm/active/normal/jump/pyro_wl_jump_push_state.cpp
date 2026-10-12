#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_push_t::enter(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase = jump_phase_t::JUMP;
    data.jump_phase_time = 0.0f;
    data.jump_vertical_velocity_integral = 0.0f;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_push_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_virtual_wall_bypass = true;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    data.target_state.x += data.jump_forward_velocity_ref * data._dt;
    data.target_state.dot_x = data.jump_forward_velocity_ref;
    // 腿长力稍后由跳跃输出覆盖；此处消除腿长误差，避免它干扰轮子 LQR。
    data.target_state.L = data.measured_state.L;
    data.target_state.dot_L = data.measured_state.dot_L;
    owner->_gain_calculate();
    const auto jump_lqr_output = [&](const uint8_t input) {
        float output = data.U0[input];
        for (uint8_t state = 0; state < STATE_DIM; ++state)
        {
            // 保留 THETA、PHI 以及两条腿的 BETA、DOT_BETA 状态。
            if (state == lqr_state_def::X ||
                state == lqr_state_def::DOT_X ||
                state == lqr_state_def::PSI ||
                state == lqr_state_def::DOT_PSI ||
                state == lqr_state_def::L_BAR ||
                state == lqr_state_def::DOT_L_BAR)
            {
                continue;
            }
            output += data.K[input][state] *
                      (data.target_state.data[state] -
                       data.measured_state.data[state]);
        }
        return output;
    };
    for (uint8_t i = 0; i < 2; ++i)
    {
        auto &leg = data.leg[i];
        const uint8_t wheel_input = i == leg_def::L
                                        ? lqr_input_def::T_W1
                                        : lqr_input_def::T_W2;
        const uint8_t leg_swing_input = i == leg_def::L
                                            ? lqr_input_def::T_P1
                                            : lqr_input_def::T_P2;
        const uint8_t force_input = i == leg_def::L
                                        ? lqr_input_def::F_L1
                                        : lqr_input_def::F_L2;
        leg.out_T_p = std::clamp(jump_lqr_output(leg_swing_input),
                                 -JUMP_T_P_LIMIT, JUMP_T_P_LIMIT);
        const float open_loop_force = 2.0f * JUMP_TAU / leg.J_L;
        leg.out_F_L = std::clamp(
            open_loop_force + jump_lqr_output(force_input),
                                 -500.0f, 500.0f);
        data.wheel[i].out_T_w = std::clamp(
            jump_lqr_output(wheel_input), -MAX_T_W, MAX_T_W);
        data.wheel[i].out_current = std::clamp(
            data.wheel[i].out_T_w / K_t, -MAX_CURRENT, MAX_CURRENT);
    }
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    // accel_z_y_lpf 是去重力加速度；失重后竖直分量趋近 -g，
    // 表示机体不再受到地面推力，进入收腿阶段。
    const bool accel_near_gravity =
        std::fabs(data.airborne.accel_z_y_lpf + GRAVITY_ACCELERATION) <=
        JUMP_PUSH_ACCEL_EPSILON;
    if (data.jump_phase_time >= JUMP_PUSH_MIN_TIME &&
        accel_near_gravity)
    {
        if (data.jump_push_accel_hold_ticks <
            JUMP_PUSH_ACCEL_HOLD_TICKS)
        {
            ++data.jump_push_accel_hold_ticks;
        }
    }
    else
    {
        data.jump_push_accel_hold_ticks = 0;
    }

    if (data.jump_push_accel_hold_ticks >= JUMP_PUSH_ACCEL_HOLD_TICKS)
    {
        auto &jump = owner->_state_active._state_normal._state_jump;
        jump._transition_phase(owner, &jump._state_recovery);
    }
    else if (data.jump_phase_time >= JUMP_PUSH_TIMEOUT)
    {
        owner->_state_active._state_normal._state_jump._abort_jump(owner);
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::state_push_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
