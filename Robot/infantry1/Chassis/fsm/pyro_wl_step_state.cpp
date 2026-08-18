#include "pyro_algo_common.h"
#include "pyro_wl_chassis.h"

#include <algorithm>


namespace pyro
{
static int stepping_phase = 0;

void wl_chassis_t::fsm_active_t::state_step_t::enter(wl_chassis_t *owner)
{
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_length                 = leg.current_leg_length;
        leg.target_leg_rad                    = leg.current_leg_rad;
        leg.target_leg_speed                  = leg.current_leg_speed;
        leg.target_leg_radps                  = leg.current_leg_radps;
        leg.out_F_L                           = 0;
        leg.out_T_p                           = 0;
        leg.out_joint_torque[joint_def::HIP]  = 0;
        leg.out_joint_torque[joint_def::KNEE] = 0;
    }
    owner->_ctx.motor.wheel[leg_def::L]->disable();
    owner->_ctx.motor.wheel[leg_def::R]->disable();

    stepping_phase = 0;

}

void wl_chassis_t::fsm_active_t::state_step_t::execute(wl_chassis_t *owner)
{
    if(stepping_phase == 0)
    {
        //腿角向后摆并加长腿长



        if(0)
        {
            stepping_phase++;
        }
    }
    else if(stepping_phase == 1)
    {
        //缩短腿长

        if(0)
        {
            stepping_phase++;
        }
    }
    else if(stepping_phase == 2)
    {
        //腿角回归


        if(0)
        {
            stepping_phase++;
        }
    }
    else if(stepping_phase == 3)
    {
        //清空标志位



    }
    


    owner->_manual_control();
    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_ctx.motor.wheel[leg_def::L]->send_torque(0);
    owner->_ctx.motor.wheel[leg_def::R]->send_torque(0);
}

void wl_chassis_t::fsm_active_t::state_step_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
