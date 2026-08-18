#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::fsm_active_t::on_enter(wl_chassis_t *owner)
{
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::L][joint_def::HIP])->clear_error();
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::L][joint_def::KNEE])->clear_error();
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::R][joint_def::HIP])->clear_error();
    static_cast<dm_motor_drv_t*>(owner->_ctx.motor.joint[leg_def::R][joint_def::KNEE])->clear_error();
    owner->_ctx.motor.joint[leg_def::L][joint_def::HIP]->enable();
    owner->_ctx.motor.joint[leg_def::L][joint_def::KNEE]->enable();
    owner->_ctx.motor.joint[leg_def::R][joint_def::HIP]->enable();
    owner->_ctx.motor.joint[leg_def::R][joint_def::KNEE]->enable();
}

void wl_chassis_t::fsm_active_t::on_execute(wl_chassis_t *ctx)
{
    static int last_step_time = 0;

    

    
    if (ctx->_current_cmd.balance_flag)
    {
        
        if(ctx->_ctx.data.flag.leg_is_ready)
        {
            change_state(&_state_normal);
        }
        else if(ctx->_current_cmd.step_times != last_step_time)
        {
            change_state(&_state_step);
        }
        else 
        {
            change_state(&_state_align);
        } 
    }
    else
    {
            change_state(&_state_manual);
    }

    last_step_time = ctx->_current_cmd.step_times;
}

void wl_chassis_t::fsm_active_t::on_exit(wl_chassis_t *ctx)
{
    (void)ctx;
}

} // namespace pyro
