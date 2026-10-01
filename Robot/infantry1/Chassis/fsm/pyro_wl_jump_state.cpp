#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

float wl_chassis_t::_jump_beta_torque(uint8_t leg_index, float limit) const
{
    const uint8_t beta_index = leg_index == leg_def::L ? lqr_state_def::BETA_1 : lqr_state_def::BETA_2;
    const uint8_t dot_beta_index = leg_index == leg_def::L ? lqr_state_def::DOT_BETA_1 : lqr_state_def::DOT_BETA_2;
    const uint8_t input_index = leg_index == leg_def::L ? lqr_input_def::T_P1 : lqr_input_def::T_P2;
    const float beta_error = -_ctx.data.measured_state.data[beta_index];
    const float dot_beta_error = -_ctx.data.measured_state.data[dot_beta_index];
    return std::clamp(_ctx.data.K[input_index][beta_index] * beta_error +
                          _ctx.data.K[input_index][dot_beta_index] * dot_beta_error,
                      -limit, limit);
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::enter(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase = jump_phase_t::PRE_COMPRESS;
    data.jump_phase_time = 0.0f;
    data.jump_heading_ref = data.measured_state.psi;
    data.jump_vertical_velocity_integral = 0.0f;
    data.jump_push_accel_hold_ticks = 0;
    data.jump_landing_counter = 0;
    data.jump_virtual_wall_bypass = true;
    data.airborne.takeoff_counter = 0;
    data.airborne.landing_counter = 0;
    data.airborne.landing_recovery = false;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    data.target_state.x = data.measured_state.x;
    data.target_state.dot_x = 0.0f;
    data.target_state.dot_L = 0.0f;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::execute(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase_time += data._dt;
    // 仅用于观测：对滤波后的竖直去重力加速度积分，记录本次跳跃的速度变化。
    data.jump_vertical_velocity_integral +=
        data.airborne.accel_z_y_lpf * data._dt;
    const auto abort_jump = [&]() {
        data.jump_virtual_wall_bypass = false;
        data.airborne.landing_recovery = false;
        // 异常退出时直接清零腿长力、腿摆力和轮子输出，随后下发零力矩。
        for (uint8_t i = 0; i < 2; ++i)
        {
            owner->_ctx.data.leg[i].out_F_L = 0.0f;
            owner->_ctx.data.leg[i].out_T_p = 0.0f;
            owner->_ctx.data.leg[i].out_joint_torque[joint_def::HIP] = 0.0f;
            owner->_ctx.data.leg[i].out_joint_torque[joint_def::KNEE] = 0.0f;
            owner->_ctx.data.wheel[i].out_T_w = 0.0f;
            owner->_ctx.data.wheel[i].out_current = 0.0f;
        }
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        // 跳跃阶段超时后回到平衡态，避免一次跳跃失败直接进入被动态。
        request_switch(&owner->_state_active._state_normal._state_balance);
    };

    switch (data.jump_phase)
    {
    // 预压缩：先把双腿压到较短长度，锁存的航向保持不变；关闭弹性墙。
    // 进入条件：左右腿都在目标长度 +/- 0.015 m 内，或预压缩超过 0.08 s。
    case jump_phase_t::PRE_COMPRESS:
        data.jump_virtual_wall_bypass = true;
        data.target_state.L = JUMP_PRECOMPRESS_LENGTH;
        data.target_state.dot_L = 0.0f;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        data.target_state.x = data.measured_state.x;
        data.target_state.dot_x = 0.0f;
        owner->_gain_calculate();
        owner->_balance_control();
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        if ((std::fabs(data.leg[leg_def::L].current_leg_length - JUMP_PRECOMPRESS_LENGTH) < 0.015f &&
             std::fabs(data.leg[leg_def::R].current_leg_length - JUMP_PRECOMPRESS_LENGTH) < 0.015f) ||
            data.jump_phase_time >= JUMP_PRECOMPRESS_TIME)
        {
            data.jump_phase = jump_phase_t::JUMP;
            data.jump_phase_time = 0.0f;
            // 从真正开始蹬腿时积分，避免预压缩阶段的加速度影响速度估计。
            data.jump_vertical_velocity_integral = 0.0f;
        }
        break;

    // 起跳蹬腿：关闭弹性墙，轮子力矩为零，腿长方向输出开环推力叠加 LQR 修正。
    // 结束条件：蹬腿至少保持 0.05 s 后，竖直总加速度连续 3 个周期接近重力；
    // 超过 0.5 s 仍未满足时，走超时安全退出。
    case jump_phase_t::JUMP:
    {
        data.jump_virtual_wall_bypass = true;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        owner->_gain_calculate();
        for (uint8_t i = 0; i < 2; ++i)
        {
            auto &leg = data.leg[i];
            leg.out_T_p = owner->_jump_beta_torque(i, JUMP_T_P_LIMIT);

            const uint8_t force_input = i == leg_def::L
                                            ? lqr_input_def::F_L1
                                            : lqr_input_def::F_L2;
            float lqr_force = data.U0[force_input];
            for (uint8_t state = 0; state < STATE_DIM; ++state)
            {
                // 蹬腿阶段不使用 X、DOT_X、PSI、DOT_PSI、L_BAR、DOT_L_BAR，
                // 避免位置、速度、航向和腿长误差干扰起跳。
                if (state == lqr_state_def::X ||
                    state == lqr_state_def::DOT_X ||
                    state == lqr_state_def::PSI ||
                    state == lqr_state_def::DOT_PSI ||
                    state == lqr_state_def::L_BAR ||
                    state == lqr_state_def::DOT_L_BAR)
                {
                    continue;
                }

                const float error = data.target_state.data[state] -
                                    data.measured_state.data[state];
                lqr_force += data.K[force_input][state] * error;
            }

            // 开环起跳力 + 仅使用俯仰、滚转和腿摆状态的 LQR 修正力。
            const float open_loop_force = 2.0f * JUMP_TAU / leg.J_L;
            leg.out_F_L = std::clamp(open_loop_force + lqr_force,
                                     -500.0f, 500.0f);
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
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
            data.jump_phase = jump_phase_t::RECOVERY;
            data.jump_phase_time = 0.0f;
            data.jump_push_accel_hold_ticks = 0;
        }
        else if (data.jump_phase_time >= JUMP_PUSH_TIMEOUT)
        {
            abort_jump();
        }
        break;
    }

    // 收腿：重新启用弹性墙，使用与起跳相同的开环关节力矩换算直接收腿。
    // 进入空中阶段：收腿阶段达到配置的超时时间。
    case jump_phase_t::RECOVERY:
    {
        data.jump_virtual_wall_bypass = false;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        for (uint8_t i = 0; i < 2; ++i)
        {
            auto &leg = data.leg[i];
            leg.target_leg_length = JUMP_AIR_LENGTH;
            // 与 JUMP 的 2 * JUMP_TAU / J_L 一致，正号恢复力矩用于反向收腿。
            const float recovery_force = 2.0f * JUMP_RECOVERY_TAU / leg.J_L;
            leg.out_F_L = recovery_force;
            leg.out_T_p = owner->_jump_beta_torque(i, MAX_T_P);
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
        }
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        if (data.jump_phase_time >= JUMP_RECOVERY_TIMEOUT)
        {
            data.airborne.L_air_ref[leg_def::L] = data.leg[leg_def::L].current_leg_length;
            data.airborne.L_air_ref[leg_def::R] = data.leg[leg_def::R].current_leg_length;
            data.jump_phase = jump_phase_t::AIR;
            data.jump_phase_time = 0.0f;
            data.jump_landing_counter = 0;
        }
        break;
    }

    // 空中：使用与起跳相同的开环关节力矩直接伸腿，保留腿摆角姿态控制；
    // 弹性墙保持开启，轮子力矩为零。
    // 落地候选：空中至少保持 0.06 s，且满足原落地判据或支持力和超过
    // 2 * 35 N；连续确认 8 个周期后进入 RETURN，空中超过 2.0 s 则退出。
    case jump_phase_t::AIR:
    {
        data.jump_virtual_wall_bypass = false;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        for (uint8_t i = 0; i < 2; ++i)
        {
            auto &leg = data.leg[i];
            const float air_force = 2.0f * JUMP_AIR_TAU / leg.J_L;
            leg.out_F_L = std::clamp(air_force, -500.0f, 500.0f);
            data.leg[i].out_T_p = owner->_jump_beta_torque(i, MAX_T_P);
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
        }
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        const bool support_contact = data.airborne.support_force_sum > 2.0f * AIR_CONTACT_FORCE_OFF;
        const bool landing_candidate = data.jump_phase_time >= JUMP_AIR_MIN_TIME &&
                                       (support_contact);
        data.jump_landing_counter = landing_candidate
                                        ? static_cast<uint16_t>(std::min<uint32_t>(data.jump_landing_counter + 1, JUMP_LANDING_TICKS))
                                        : 0;
        if (data.jump_landing_counter >= JUMP_LANDING_TICKS)
        {
            data.jump_phase = jump_phase_t::RETURN;
            data.jump_phase_time = 0.0f;
        }
        else if (data.jump_phase_time >= JUMP_TOTAL_TIMEOUT)
        {
            abort_jump();
        }
        break;
    }

    // 返回：锁存落地瞬间腿长，交给 BALANCE 的 landing_recovery 逐步恢复到正常腿长。
    case jump_phase_t::RETURN:
        data.airborne.L_ref = 0.5f * (data.leg[leg_def::L].current_leg_length + data.leg[leg_def::R].current_leg_length);
        data.target_state.L = data.airborne.L_ref;
        data.target_state.dot_L = 0.0f;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        data.airborne.landing_recovery = true;
        data.jump_virtual_wall_bypass = false;
        request_switch(&owner->_state_active._state_normal._state_balance);
        break;

    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::exit(wl_chassis_t *owner)
{
    owner->_ctx.data.jump_virtual_wall_bypass = false;
}

} // namespace pyro
