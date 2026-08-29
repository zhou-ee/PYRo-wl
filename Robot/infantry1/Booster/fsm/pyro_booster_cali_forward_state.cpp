#include "pyro_wl_booster.h"
#include "pyro_algo_common.h"

void pyro::wl_booster_t::fsm_active_t::state_cali_forward_t::enter(owner* owner) 
{
    // --- 初始化状态 ---
    owner->_ctx.data.target_state.useTriggerSpeedLoopOnly = false;

    owner->_ctx.data.target_state.targetTriggerRad += 9.0f * PI;
    if(owner->_ctx.data.target_state.targetTriggerRad >= 72 *PI)
    {
        owner->_ctx.data.target_state.targetTriggerRad -= 72 * PI;
    }
}

void pyro::wl_booster_t::fsm_active_t::state_cali_forward_t::execute(owner* owner)
{

    float err = wrap2pi_f32_normalized
        ((owner->_ctx.data.target_state.targetTriggerRad - owner->_ctx.data.motor_state.trigger_rad) / 36.0f);
    while (err >  PI) err -= 2.0f * PI;
    while (err < -PI) err += 2.0f * PI;

    //堵转检测
    static uint16_t blocked_count = 0;
    if((err >= PI / 16.0f) && (fabs(owner->_ctx.data.motor_state.trigger.vel) <= 10.0f))
    {
        if(blocked_count >= 800)
        {
            owner->_ctx.data.target_state.jamSourceState = FireState::CaliForward;
            request_switch(&owner->_state_active._state_cali_reverse);
        }
        blocked_count++;
    }
    else 
    {
        blocked_count = 0;
    }

    //到达目标 , 回到ready
    static int steady_count = 0;
    if(err <= PI / 16.0f)
    {
        if(steady_count >= 10)
        {
            //决定向前校准后应当进入什么状态
            owner->_ctx.data.isCalibrated = true;
            if(owner->_ctx.data.target_state.jamSourceState == FireState::BurstFire)
            {
                request_switch(&owner->_state_active._state_burstfire);
            }
            else if (owner->_ctx.data.target_state.jamSourceState == FireState::SingleFire) 
            {
                request_switch(&owner->_state_active._state_singlefire);
            }
            else 
            {
                request_switch(&owner->_state_active._state_ready);
            }
            
        }
        steady_count++;
    }
    else 
    {
        steady_count = 0;
    }

    owner->calculateFricCurrents();
    owner->calculateTriggerCurrents(owner->_ctx.data.target_state.useTriggerSpeedLoopOnly);
}

void pyro::wl_booster_t::fsm_active_t::state_cali_forward_t::exit(owner* owner)
{

}