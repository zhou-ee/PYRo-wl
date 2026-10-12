#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::enter(wl_chassis_t *owner)
{
    auto &data = owner->_ctx.data;
    data.jump_phase = jump_phase_t::PRE_COMPRESS;
    data.jump_phase_time = 0.0f;
    data.jump_heading_ref = data.measured_state.psi;
    data.jump_forward_velocity_ref = data.measured_state.dot_x;
    data.jump_vertical_velocity_integral = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::L] = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::R] = 0.0f;
    data.jump_landing_confirm_time = 0.0f;
    data.jump_push_accel_hold_ticks = 0;
    data.jump_virtual_wall_bypass = true;
    data.airborne.takeoff_counter = 0;
    data.airborne.landing_counter = 0;
    data.airborne.landing_recovery = false;
    data.target_state.psi = data.jump_heading_ref;
    data.target_state.dot_psi = 0.0f;
    data.target_state.x = data.measured_state.x;
    data.target_state.dot_x = data.jump_forward_velocity_ref;
    data.target_state.theta = 0.0f;
    data.target_state.dot_theta = 0.0f;
    data.target_state.phi = 0.0f;
    data.target_state.dot_phi = 0.0f;
    data.target_state.beta1 = 0.0f;
    data.target_state.dot_beta1 = 0.0f;
    data.target_state.beta2 = 0.0f;
    data.target_state.dot_beta2 = 0.0f;
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
        request_switch(&owner->_state_passive);
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
            data.jump_phase = jump_phase_t::JUMP;
            data.jump_phase_time = 0.0f;
            // 从真正开始蹬腿时积分，避免预压缩阶段的加速度影响速度估计。
            data.jump_vertical_velocity_integral = 0.0f;
        }
        break;

    // 起跳蹬腿：关闭弹性墙。轮子、腿摆和腿长均使用排除
    // X、DOT_X、PSI、DOT_PSI、L_BAR、DOT_L_BAR 后的 LQR；
    // 腿长方向再叠加开环起跳力。
    // 结束条件：蹬腿至少保持 0.05 s 后，竖直总加速度连续 3 个周期接近重力；
    // 超过 0.5 s 仍未满足时，走超时安全退出。
    case jump_phase_t::JUMP:
    {
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
    // 竖直速度向上为正；检测到下落速度达到 0.5 m/s 后进入 AIR，超时作为兜底。
    case jump_phase_t::RECOVERY:
    {
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
            data.airborne.L_air_ref[leg_def::L] = data.leg[leg_def::L].current_leg_length;
            data.airborne.L_air_ref[leg_def::R] = data.leg[leg_def::R].current_leg_length;
            data.jump_phase = jump_phase_t::AIR;
            data.jump_phase_time = 0.0f;
            data.jump_leg_extension_speed_peak[leg_def::L] = 0.0f;
            data.jump_leg_extension_speed_peak[leg_def::R] = 0.0f;
            data.jump_landing_confirm_time = 0.0f;
        }
        break;
    }

    // 空中：使用与起跳相同的开环关节力矩直接伸腿，保留腿摆角姿态控制；
    // 弹性墙保持开启，轮子力矩为零。
    // 落地候选（任意一条腿满足即可）：
    // A. 腿长速度小于 -0.05 m/s；或
    // B. 腿长低于机械最大值 3 cm、伸展速度峰值大于 0.1 m/s，且
    //    当前速度相对峰值下降超过 0.15 m/s。
    // A 或 B 连续成立 10 ms 后进入 RETURN。
    case jump_phase_t::AIR:
    {
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
