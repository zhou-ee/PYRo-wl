#include "pyro_wl_chassis.h"

#include <algorithm>
#include <cmath>

namespace pyro
{

void wl_chassis_t::fsm_active_t::state_normal_t::state_spin_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.data.odom.real_x = 0.0f;
    owner->_ctx.data.spin_decay_active = false;
    owner->_ctx.data.spin_recovery_active = false;
    owner->_ctx.data.spin_speed_ref = owner->_ctx.data.ins.gyro[0];

    auto &target = owner->_ctx.data.target_state;
    target.x = owner->_ctx.data.measured_state.x;
    target.dot_x = 0.0f;
    target.psi = owner->_ctx.data.measured_state.psi;
    target.dot_psi = owner->_ctx.data.spin_speed_ref;
    target.L = owner->_ctx.data.airborne.landing_recovery
                   ? owner->_ctx.data.airborne.L_ref
                   : NORMAL_LENGTH_TARGET;
    target.dot_L = 0.0f;
    target.theta = 0.0f;
    target.dot_theta = 0.0f;
    target.phi = 0.0f;
    target.dot_phi = 0.0f;
    target.beta1 = 0.0f;
    target.dot_beta1 = 0.0f;
    target.beta2 = 0.0f;
    target.dot_beta2 = 0.0f;

    owner->_ctx.data.normal_roll_force_trim = 0.0f;
    for(float &feedforward : owner->_ctx.data.U0)
    {
        feedforward = 0.0f;
    }

    const float avg_length =
        0.5f * (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                owner->_ctx.data.leg[leg_def::R].current_leg_length);
    for(auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_rad = leg.current_leg_rad;
        leg.target_leg_speed = leg.current_leg_speed;
        leg.target_leg_radps = leg.current_leg_radps;
        leg.target_leg_length = avg_length;
        leg.out_F_L = 0.0f;
        leg.out_T_p = 0.0f;
        leg.out_joint_torque[joint_def::HIP] = 0.0f;
        leg.out_joint_torque[joint_def::KNEE] = 0.0f;
    }

    owner->_ctx.motor.wheel[leg_def::L]->enable();
    owner->_ctx.motor.wheel[leg_def::R]->enable();
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_spin_t::execute(wl_chassis_t *owner)
{
    //退出机制
    if(owner->_ctx.data.current_function != chassis_function_state_t::SPIN)
    {
        

        request_switch(&owner->_state_active._state_normal._state_balance);
    }

    static uint16_t reset_count = 0;
    if(std::fabs(owner->_ctx.data.ins.euler_rad[1]) >= PI / 4.0f ||
       std::fabs(owner->_ctx.data.ins.euler_rad[2]) >= PI / 9.0f)
    {
        if(reset_count >= 50)
        {
            owner->_ctx.data.flag.leg_is_should_restart = true;
            return;
        }
        ++reset_count;
    }
    else
    {
        reset_count = 0;
    }

    if(owner->_ctx.data.airborne.landing_recovery)
    {
        owner->_execute_landing_recovery();
    }
    else
    {
        const float target_dot_L = owner->_current_cmd.dot_L;
        owner->_ctx.data.target_state.dot_L = target_dot_L;
        owner->_ctx.data.target_state.L = std::clamp(
            owner->_ctx.data.target_state.L +
                target_dot_L * owner->_ctx.data._dt,
            MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    }

    // Decouple accumulated displacement and yaw angle while retaining
    // velocity feedback for translation damping and spin-rate tracking.
    const float spin_accel_step = SPIN_YAW_ACCEL * owner->_ctx.data._dt;
    if(owner->_ctx.data.spin_speed_ref <
       owner->_current_cmd.wz - spin_accel_step)
    {
        owner->_ctx.data.spin_speed_ref += spin_accel_step;
    }
    else if(owner->_ctx.data.spin_speed_ref >
            owner->_current_cmd.wz + spin_accel_step)
    {
        owner->_ctx.data.spin_speed_ref -= spin_accel_step;
    }
    else
    {
        owner->_ctx.data.spin_speed_ref = owner->_current_cmd.wz;
    }

    owner->_ctx.data.measured_state.psi = owner->_ctx.data.ins.euler_rad[0];
    owner->_ctx.data.measured_state.dot_psi = owner->_ctx.data.ins.gyro[0];
    owner->_ctx.data.target_state.x = owner->_ctx.data.measured_state.x;
    owner->_ctx.data.target_state.dot_x = 0.0f;
    owner->_ctx.data.target_state.psi = owner->_ctx.data.measured_state.psi;
    owner->_ctx.data.target_state.dot_psi = owner->_ctx.data.spin_speed_ref;
    owner->_ctx.data.normal_roll_force_trim = 0.0f;

    owner->_gain_calculate();
    owner->_balance_control();

    owner->_ctx.data.leg[leg_def::L].out_F_L =
        owner->_ctx.data.control.F_l1;
    owner->_ctx.data.leg[leg_def::R].out_F_L =
        owner->_ctx.data.control.F_l2;

    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_spin_t::exit(wl_chassis_t *owner)
{
    owner->_ctx.data.target_state.x = owner->_ctx.data.measured_state.x;
    owner->_ctx.data.target_state.dot_x = 0.0f;
    owner->_ctx.data.target_state.psi = owner->_ctx.data.measured_state.psi;
    owner->_ctx.data.target_state.dot_psi = 0.0f;
    const bool resume_normal = owner->_current_cmd.cmd_continus_state ==
                               chassis_active_state_t::NORMAL;
    owner->_ctx.data.spin_decay_speed = owner->_ctx.data.ins.gyro[0];
    owner->_ctx.data.spin_decay_elapsed = 0.0f;
    const float spin_direction_source =
        std::fabs(owner->_current_cmd.wz) > 0.1f
            ? owner->_current_cmd.wz
            : owner->_ctx.data.ins.gyro[0];
    owner->_ctx.data.spin_direction =
        spin_direction_source >= 0.0f ? 1.0f : -1.0f;
    owner->_ctx.data.spin_recovery_active = false;
    owner->_ctx.data.spin_decay_active = resume_normal;
}

} // namespace pyro
