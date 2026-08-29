#include "pyro_wl_booster.h"
#include "pyro_algo_common.h"
#include "booster_config.h"

void pyro::wl_booster_t::fsm_active_t::state_burstfire_t::enter(owner* owner) 
{
    // --- 初始化状态 ---
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = true;
    owner->_ctx.data.isCalibrated = false;
}

void pyro::wl_booster_t::fsm_active_t::state_burstfire_t::execute(owner* owner)
{
    owner->_ctx.data.target_state.targetTriggerSpeed = TRIGGER_SPEED;
    owner->_ctx.data.target_state.targetTriggerRad =  owner->_ctx.data.motor_state.trigger_rad;

    //堵转检测
    static uint16_t blocked_count = 0;
    if (fabs(owner->_ctx.data.target_state.targetTriggerSpeed) >= 50.0f && 
        fabs(owner->_ctx.data.motor_state.trigger.vel) <= 10.0f)
    {
        if(blocked_count >= 800)
        {
            owner->_ctx.data.target_state.jamSourceState = FireState::BurstFire;
            request_switch(&owner->_state_active._state_cali_reverse);
        }
        blocked_count++;
    }
    else 
    {
        blocked_count = 0;
    }

    //侦测到指令结束，切回ready
    if(owner->_ctx.data.cmd_event == ShootEvent::BURST_END)
    {
        request_switch(&owner->_state_active._state_ready);
    }


    owner->calculateFricCurrents();
    owner->calculateTriggerCurrents(owner->_ctx.data.target_state.useTriggerSpeedLoopOnly);
}

void pyro::wl_booster_t::fsm_active_t::state_burstfire_t::exit(owner* owner)
{

}