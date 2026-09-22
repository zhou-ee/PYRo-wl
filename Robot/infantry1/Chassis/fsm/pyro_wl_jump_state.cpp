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
    data.jump_recovery_hold_ticks = 0;
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
    const auto abort_jump = [&]() {
        data.jump_virtual_wall_bypass = false;
        data.airborne.state = chassis_function_state_t::NONE;
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
        request_switch(&owner->_state_passive);
    };

    // 跳跃期间任一关节电机掉线，立即中止并下发零力矩。
    bool motor_offline = false;
    for (uint8_t leg = 0; leg < 2; ++leg)
    {
        for (uint8_t joint = 0; joint < 2; ++joint)
        {
            motor_offline = motor_offline ||
                             !owner->_ctx.motor.joint[leg][joint]->is_online();
        }
    }
    if (motor_offline)
    {
        abort_jump();
        return;
    }

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
        }
        break;

    // 起跳蹬腿：关闭弹性墙，轮子力矩为零，腿长方向输出开环推力。
    // 结束条件：双腿达到 0.34 m，任一腿达到硬限位 0.37 m，或蹬腿超过 0.5 s。
    case jump_phase_t::JUMP:
    {
        data.jump_virtual_wall_bypass = true;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        for (uint8_t i = 0; i < 2; ++i)
        {
            auto &leg = data.leg[i];
            leg.out_T_p = owner->_jump_beta_torque(i, JUMP_T_P_LIMIT);

        // F_L * J_L = 2 * JUMP_TAU；
        // JUMP_TAU 当前为 13 N m。
            leg.out_F_L = std::clamp(2.0f * JUMP_TAU / leg.J_L, -500.0f, 500.0f);
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
        }
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        const bool push_finished = data.leg[leg_def::L].current_leg_length >= JUMP_PUSH_END_LENGTH &&
                                   data.leg[leg_def::R].current_leg_length >= JUMP_PUSH_END_LENGTH;
        const bool hard_limit = data.leg[leg_def::L].current_leg_length >= JUMP_HARD_LENGTH ||
                                data.leg[leg_def::R].current_leg_length >= JUMP_HARD_LENGTH;
        if (push_finished || hard_limit)
        {
            data.jump_phase = jump_phase_t::RECOVERY;
            data.jump_phase_time = 0.0f;
            data.jump_recovery_hold_ticks = 0;
        }
        else if (data.jump_phase_time >= JUMP_PUSH_TIMEOUT)
        {
            abort_jump();
        }
        break;
    }

    // 收腿：重新启用弹性墙，用腿长 PD 将双腿收回约 0.20 m。
    // 进入空中阶段：双腿误差都小于 0.015 m 并连续保持 10 个控制周期，
    // 或收腿阶段超过 0.5 s。
    case jump_phase_t::RECOVERY:
    {
        data.jump_virtual_wall_bypass = false;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        for (uint8_t i = 0; i < 2; ++i)
        {
            auto &leg = data.leg[i];
            leg.target_leg_length = JUMP_AIR_LENGTH;
            const float manual_force = owner->_ctx.pid.leg_length[i]->calculate(
                JUMP_AIR_LENGTH, leg.current_leg_length, leg.current_leg_speed) - leg.gas_spring_force +leg.virtual_wall_force;
            leg.out_F_L = std::clamp(JUMP_RECOVERY_PD_SCALE * manual_force,
                                     -JUMP_RECOVERY_FORCE_MAX, JUMP_RECOVERY_FORCE_MAX);
            leg.out_T_p = owner->_jump_beta_torque(i, MAX_T_P);
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
        }
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        const bool ready = std::fabs(data.leg[leg_def::L].current_leg_length - JUMP_AIR_LENGTH) < 0.015f &&
                           std::fabs(data.leg[leg_def::R].current_leg_length - JUMP_AIR_LENGTH) < 0.015f;
        data.jump_recovery_hold_ticks = ready
                                            ? static_cast<uint16_t>(std::min<uint32_t>(data.jump_recovery_hold_ticks + 1, JUMP_RECOVERY_HOLD_TICKS))
                                            : 0;
        if (data.jump_recovery_hold_ticks >= JUMP_RECOVERY_HOLD_TICKS ||
            data.jump_phase_time >= JUMP_RECOVERY_TIMEOUT)
        {
            data.airborne.L_air_ref[leg_def::L] = data.leg[leg_def::L].current_leg_length;
            data.airborne.L_air_ref[leg_def::R] = data.leg[leg_def::R].current_leg_length;
            data.jump_phase = jump_phase_t::AIR;
            data.jump_phase_time = 0.0f;
            data.jump_landing_counter = 0;
        }
        break;
    }

    // 空中：F_L为零，只保留腿摆角姿态控制；弹性墙保持开启。
    // 落地候选：空中至少保持 0.06 s，且满足原落地判据或支持力和超过
    // 2 * 35 N；连续确认 8 个周期后进入 RETURN，空中超过 2.0 s 则退出。
    case jump_phase_t::AIR:
    {
        data.jump_virtual_wall_bypass = false;
        data.target_state.psi = data.jump_heading_ref;
        data.target_state.dot_psi = 0.0f;
        for (uint8_t i = 0; i < 2; ++i)
        {
            data.leg[i].out_F_L = 0.0f;
            data.leg[i].out_T_p = owner->_jump_beta_torque(i, MAX_T_P);
            data.wheel[i].out_T_w = 0.0f;
            data.wheel[i].out_current = 0.0f;
        }
        owner->_vmc_trans_v2j();
        owner->_send_joint_torque();
        owner->_send_wheel_torque();
        const bool support_contact = data.airborne.support_force_sum > 2.0f * AIR_CONTACT_FORCE_OFF;
        const bool landing_candidate = data.jump_phase_time >= JUMP_AIR_MIN_TIME &&
                                       (owner->_detect_landing() || support_contact);
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
        data.airborne.state = chassis_function_state_t::NONE;
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
