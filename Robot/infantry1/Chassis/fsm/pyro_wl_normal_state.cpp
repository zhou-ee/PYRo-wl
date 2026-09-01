#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{

static int reset_count;

void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{

    owner->_ctx.data.odom.real_x            = 0;
    owner->_ctx.data.measured_state[state_def::X] = 0.0f;
    owner->_ctx.data.predict_state = owner->_ctx.data.measured_state;
    owner->_ctx.data.dist = 0.0f;
    owner->_ctx.data.target_state = 0.0f;
    owner->_ctx.data.target_state[state_def::L] =
        owner->_ctx.data.airborne.landing_recovery
            ? owner->_ctx.data.airborne.L_ref
            : NORMAL_LENGTH_TARGET;
    owner->_ctx.data.U0 = 0.0f;

    float avg_length = (owner->_ctx.data.leg[leg_def::L].current_leg_length +
                        owner->_ctx.data.leg[leg_def::R].current_leg_length) *
                       0.5f;
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_rad                    = leg.current_leg_rad;
        leg.target_leg_speed                  = leg.current_leg_speed;
        leg.target_leg_radps                  = leg.current_leg_radps;
        leg.target_leg_length                 = avg_length;
        leg.out_F_L                           = 0;
        leg.out_T_p                           = 0;
        leg.out_joint_torque[joint_def::HIP]  = 0;
        leg.out_joint_torque[joint_def::KNEE] = 0;
    }
    // owner->_ctx.pid.leg_length[leg_def::L]->clear();
    // owner->_ctx.pid.leg_length[leg_def::R]->clear();
    owner->_ctx.pid.leg_rad[leg_def::L]->clear();
    owner->_ctx.pid.leg_rad[leg_def::R]->clear();


    owner->_ctx.motor.wheel[leg_def::L]->enable();
    owner->_ctx.motor.wheel[leg_def::R]->enable();
    owner->_ctx.data.airborne.state = chassis_state_t::NORMAL;
    owner->_ctx.data.airborne.takeoff_counter = 0;
    owner->_ctx.data.airborne.landing_counter = 0;
    owner->_ctx.data.flag.leg_is_ready = true;
}

void wl_chassis_t::fsm_active_t::state_normal_t::execute(wl_chassis_t *owner)
{
    //紧急下力判断
    if(abs(owner->_ctx.data.ins.euler_rad[1]) >= PI / 6.0f ||
       abs(owner->_ctx.data.ins.euler_rad[2]) >= PI / 9.0f)
    {
        if(reset_count >= 50)
        {
            owner->_ctx.data.flag.leg_is_should_restart = true;
        }
        reset_count++;
    }
    else
    {
        reset_count = 0;
    }

    if (owner->_ctx.data.airborne.landing_recovery)
    {
        owner->_execute_landing_recovery();
    }
    else
    {
        //腿长加上遥控器的小量
        const float target_dot_L = owner->_current_cmd.dot_L;
        owner->_ctx.data.target_state[state_def::DOT_L] = target_dot_L;
        owner->_ctx.data.target_state[state_def::L] +=
            target_dot_L * owner->_ctx.data._dt;
        //腿长限幅
        owner->_ctx.data.target_state[state_def::L] =
            std::clamp(owner->_ctx.data.target_state[state_def::L],
                       MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    }


    const float target_vx = owner->_current_cmd.v;
    owner->_ctx.data.target_state[state_def::X] +=
        target_vx * owner->_ctx.data._dt;
    owner->_ctx.data.target_state[state_def::DOT_X] = target_vx;

    const float target_wz               = owner->_current_cmd.wz;
    owner->_ctx.data.target_state[state_def::PSI] +=
        target_wz * owner->_ctx.data._dt;
    owner->_ctx.data.target_state[state_def::PSI] =
        loop_fp32_constrain(
            owner->_ctx.data.target_state[state_def::PSI], -PI, PI);
    owner->_ctx.data.target_state[state_def::DOT_PSI] = target_wz;

    if (!owner->_ctx.data.airborne.landing_recovery &&
        owner->_detect_takeoff())
    {
        owner->_ctx.data.airborne.state = chassis_state_t::AIR;
        owner->_ctx.data.airborne.takeoff_counter = 0;
        owner->_ctx.data.airborne.landing_counter = 0;
        request_switch(&owner->_state_active._state_air);
        return;
    }

    owner->_fit_params();
    owner->_balance_control();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
    owner->_leso_update();
}

void wl_chassis_t::fsm_active_t::state_normal_t::exit(wl_chassis_t *owner)
{
    owner->_ctx.data.dist = {};
}

} // namespace pyro
