#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{

static int reset_count;
static constexpr float REAL_MAX_LEG_LENGTH = 0.32f;

void wl_chassis_t::fsm_active_t::state_normal_t::enter(wl_chassis_t *owner)
{

    owner->_ctx.data.odom.real_x            = 0;

    owner->_ctx.data.target_state.x         = 0;
    owner->_ctx.data.target_state.dot_x     = 0.0f;
    owner->_ctx.data.target_state.psi       = owner->_ctx.data.ins.euler_rad[0];
    owner->_ctx.data.target_state.dot_psi   = 0.0f;
    owner->_ctx.data.target_state.h         = 0.2f;
    owner->_ctx.data.target_state.dot_h     = 0.0f;
    owner->_ctx.data.target_state.theta     = 0.0f;
    owner->_ctx.data.target_state.dot_theta = 0.0f;
    owner->_ctx.data.target_state.phi       = 0.0f;
    owner->_ctx.data.target_state.dot_phi   = 0.0f;
    owner->_ctx.data.target_state.beta1     = 0.0f;
    owner->_ctx.data.target_state.beta2     = 0.0f;
    owner->_ctx.data.target_state.dot_beta1 = 0.0f;
    owner->_ctx.data.target_state.dot_beta2 = 0.0f;


    for (float & i : owner->_ctx.data.U0)
    {
        i = 0.0f;
    }

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

    //腿长加上遥控器的小量
    owner->_ctx.data.target_state.h += owner->_current_cmd.delta_h;

    //腿长限幅
    owner->_ctx.data.target_state.h =
        std::clamp(owner->_ctx.data.target_state.h,MIN_LEG_LENGTH, REAL_MAX_LEG_LENGTH);
    

    const float target_vx = owner->_current_cmd.v;
    owner->_ctx.data.target_state.x += target_vx * owner->_ctx.data._dt;
    owner->_ctx.data.target_state.dot_x = target_vx;

    const float target_wz               = owner->_current_cmd.wz;
    owner->_ctx.data.target_state.psi += target_wz * owner->_ctx.data._dt;
    owner->_ctx.data.target_state.psi = loop_fp32_constrain(owner->_ctx.data.target_state.psi,-PI,PI);
    owner->_ctx.data.target_state.dot_psi = target_wz;

    owner->_gain_calculate();
    owner->_balance_control();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::exit(wl_chassis_t *owner)
{
    owner->_ctx.data.flag.leg_is_ready = false;
    (void)owner;
}

} // namespace pyro
