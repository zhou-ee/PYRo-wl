#include "pyro_algo_common.h"
#include "pyro_wl_chassis.h"

#include <algorithm>


namespace pyro
{



void wl_chassis_t::fsm_active_t::state_normal_t::on_enter(wl_chassis_t *owner)
{
    change_state(&_state_align);
}

void wl_chassis_t::fsm_active_t::state_normal_t::on_execute(wl_chassis_t *owner)
{
    //判断是否下达了功能状态的指令
    static chassis_function_state_t last_function = chassis_function_state_t::NONE;
    if(owner->_current_cmd.cmd_function_state != last_function)
    {
        owner->_ctx.data.current_function = owner->_current_cmd.cmd_function_state;
    }
    else 
    {
        owner->_ctx.data.current_function = chassis_function_state_t::NONE;
    }
    last_function = owner->_current_cmd.cmd_function_state;


    if(owner->_ctx.data.current_function == chassis_function_state_t::STEP)
    {
        change_state(&_state_step);
    }
        if(owner->_ctx.data.current_function == chassis_function_state_t::JUMP)
    {
        change_state(&_state_jump);
    }
    
}

void wl_chassis_t::fsm_active_t::state_normal_t::on_exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro