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

    // 锁存起跳瞬间的航向与前进速度；整个跳跃过程继续沿该航向和速度参考运动，
    // 落地后 Balance 状态也会沿用这里保存的前进速度参考做连续恢复。
    data.jump_heading_ref = data.measured_state.psi;
    data.jump_forward_velocity_ref = data.measured_state.dot_x;
    data.jump_precompress_exit_velocity = 0.0f;
    data.jump_vertical_velocity_integral = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::L] = 0.0f;
    data.jump_leg_extension_speed_peak[leg_def::R] = 0.0f;
    data.jump_landing_confirm_time = 0.0f;
    data.jump_push_accel_hold_ticks = 0;
    data.airborne.takeoff_counter = 0;
    data.airborne.landing_counter = 0;
    data.airborne.landing_recovery = false;
    data.airborne.attitude_recovery = false;
    data.airborne.velocity_recovery = false;
    data.airborne.forward_velocity_ref = data.jump_forward_velocity_ref;

    // 初始化跳跃期间使用的目标状态
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

    // 对滤波后的竖直加速度积分，得到从当前阶段起累计的竖直速度变化量。。
    data.jump_vertical_velocity_integral +=
        data.airborne.accel_z_y_lpf * data._dt;

    // 任一阶段超时或状态异常时，立即清零腿、关节和轮毂输出，发送零力矩，
    // 随后退出到 Passive，避免跳跃控制继续保持上一拍的执行器指令。
    const auto abort_jump = [&]() {
        data.airborne.landing_recovery = false;
        for (uint8_t i = 0; i < 2; ++i)
        {
            data.leg[i].out_F_L = 0.0f;
            data.leg[i].out_T_p = 0.0f;
            data.leg[i].out_joint_torque[joint_def::HIP] = 0.0f;
            data.leg[i].out_joint_torque[joint_def::KNEE] = 0.0f;
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
        }
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        request_switch(&owner->_state_passive);
    };

    // RECOVERY 与 AIR 阶段只保留本腿的摆角 beta 和摆角速度 dot_beta
    // 对本腿摆动虚拟力矩的 LQR 反馈，不引入车体、航向、腿长或另一条腿的状态。
    const auto leg_swing_lqr_output = [&](const uint8_t leg_index) {
        const uint8_t beta_index = leg_index == leg_def::L
                                       ? lqr_state_def::BETA_1
                                       : lqr_state_def::BETA_2;
        const uint8_t dot_beta_index = leg_index == leg_def::L
                                           ? lqr_state_def::DOT_BETA_1
                                           : lqr_state_def::DOT_BETA_2;
        const uint8_t input_index = leg_index == leg_def::L
                                        ? lqr_input_def::T_P1
                                        : lqr_input_def::T_P2;
        return std::clamp(
            -data.K[input_index][beta_index] * data.measured_state.data[beta_index] -
                data.K[input_index][dot_beta_index] *
                    data.measured_state.data[dot_beta_index],
            -JUMP_T_P_LIMIT, JUMP_T_P_LIMIT);
    };

    switch (data.jump_phase)
    {
    case jump_phase_t::PRE_COMPRESS:
        // 预压缩：
        // 使用完整平衡控制把双腿压到指定长度，同时锁定起跳航向并保持
        // 起跳前进速度。双腿均进入长度容差，或等待超时后，转入 JUMP 蹬腿。
        data.target_state.L = JUMP_PRECOMPRESS_LENGTH;
        data.target_state.dot_L = 0.0f;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        data.target_state.x += data.jump_forward_velocity_ref * data._dt;
        data.target_state.dot_x = data.jump_forward_velocity_ref;
        owner->_gain_calculate();
        owner->_balance_control();


        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        if ((std::fabs(data.leg[leg_def::L].current_leg_length -
                       JUMP_PRECOMPRESS_LENGTH) < 0.015f &&
             std::fabs(data.leg[leg_def::R].current_leg_length -
                       JUMP_PRECOMPRESS_LENGTH) < 0.015f) ||
            data.jump_phase_time >= JUMP_PRECOMPRESS_TIME)
        {
            // 保留当前落地速度恢复流程所需的触地速度参考。
            data.jump_precompress_exit_velocity = data.measured_state.dot_x;
            data.airborne.forward_velocity_ref =
                JUMP_LANDING_VELOCITY_SCALE * data.jump_precompress_exit_velocity;
            data.jump_phase = jump_phase_t::JUMP;
            data.jump_phase_time = 0.0f;
            data.jump_vertical_velocity_integral = 0.0f;
        }
        break;

    case jump_phase_t::JUMP:
    {
        // 蹬腿：
        // 在原有 LQR 输出上叠加 JUMP_TAU 对应的开环伸腿力。
        // 至少蹬腿 JUMP_PUSH_MIN_TIME 后，竖直加速度连续满足接近 -g 的条件，
        // 随后切换到 RECOVERY。
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        data.target_state.x += data.jump_forward_velocity_ref * data._dt;
        data.target_state.dot_x = data.jump_forward_velocity_ref;
        data.target_state.L = data.measured_state.L;
        data.target_state.dot_L = data.measured_state.dot_L;
        owner->_gain_calculate();
        const auto jump_lqr_output = [&](const uint8_t input) {
            float output = data.U0[input];
            for (uint8_t state = 0; state < STATE_DIM; ++state)
            {
                // 蹬腿时排除 位置/前进速度、航向/航向角速度 以及平均腿长/腿长速度，
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
                open_loop_force + jump_lqr_output(force_input), -500.0f, 500.0f);
            data.wheel[i].out_T_w = std::clamp(
                jump_lqr_output(wheel_input), -MAX_T_W, MAX_T_W);
            data.wheel[i].out_current = std::clamp(
                data.wheel[i].out_T_w / K_t, -MAX_CURRENT, MAX_CURRENT);
        }
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();

        const bool accel_near_gravity =
            std::fabs(data.airborne.accel_z_y_lpf + GRAVITY_ACCELERATION) <=
            JUMP_PUSH_ACCEL_EPSILON;
        if (data.jump_phase_time >= JUMP_PUSH_MIN_TIME && accel_near_gravity)
        {
            if (data.jump_push_accel_hold_ticks < JUMP_PUSH_ACCEL_HOLD_TICKS)
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

    case jump_phase_t::RECOVERY:
    {
        // 收腿：
        // 用固定开环力把腿收向空中长度；摆腿只使用 beta/dot_beta LQR，
        // 轮毂保持零输出。累计下落速度达到阈值或恢复超时后进入 AIR。
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        owner->_gain_calculate();
        for (uint8_t i = 0; i < 2; ++i)
        {
            auto &leg = data.leg[i];
            leg.target_leg_length = JUMP_AIR_LENGTH;
            // 收腿力矩之外，补偿腿与轮的重力以及气弹簧支撑力，
            // 避免这些被动项改变期望的净收腿力。
            leg.out_F_L = owner->_motor_torque_to_leg_force(leg, -JUMP_RECOVERY_TAU) -
                          (LEG_MASS + WHEEL_MASS) * GRAVITY_ACCELERATION -
                          leg.gas_spring_force;
            leg.out_T_p = leg_swing_lqr_output(i);
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
            // 记录进入 AIR 时两腿长度，清空落地检测历史量，从该时刻重新观察
            // 腿伸展速度峰值及其反转/骤降特征。
            data.airborne.L_air_ref[leg_def::L] =
                data.leg[leg_def::L].current_leg_length;
            data.airborne.L_air_ref[leg_def::R] =
                data.leg[leg_def::R].current_leg_length;
            data.jump_phase = jump_phase_t::AIR;
            data.jump_phase_time = 0.0f;
            data.jump_leg_extension_speed_peak[leg_def::L] = 0.0f;
            data.jump_leg_extension_speed_peak[leg_def::R] = 0.0f;
            data.jump_landing_confirm_time = 0.0f;
        }
        break;
    }

    case jump_phase_t::AIR:
    {
        // 空中阶段：双腿施加 JUMP_AIR_TAU 对应的开环力，腿摆角继续仅由
        // beta/dot_beta LQR 稳定，轮毂保持零输出，并持续进行落地检测。
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        owner->_gain_calculate();
        const bool landed = owner->_detect_landing(true);
        for (uint8_t i = 0; i < 2; ++i)
        {
            auto &leg = data.leg[i];
            const float air_force = 2.0f * JUMP_AIR_TAU / leg.J_L;
            leg.out_F_L = std::clamp(air_force, -500.0f, 500.0f);
            leg.out_T_p = leg_swing_lqr_output(i);
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
        }
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();

        if (landed)
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

    case jump_phase_t::RETURN:
        // 落地交接：锁存触地时双腿平均长度作为 Balance 恢复的腿长起点，
        // 打开 landing_recovery 标志，再切回 Balance。
        // 后续腿长、Pitch/腿摆角以及前进速度的限速限加速度恢复
        // 均由现有 Balance 落地恢复流程完成。
        data.airborne.L_ref =
            0.5f * (data.leg[leg_def::L].current_leg_length +
                    data.leg[leg_def::R].current_leg_length);
        data.target_state.L = data.airborne.L_ref;
        data.target_state.dot_L = 0.0f;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        data.airborne.landing_recovery = true;
        request_switch(&owner->_state_active._state_normal._state_balance);
        break;
    }
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
