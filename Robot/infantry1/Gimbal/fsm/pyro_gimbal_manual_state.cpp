#include "pyro_wl_gimbal.h"
#include "gimbal_config.h"




void pyro::wl_gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
    owner->_ctx.data.telem.targetPitchRad = 0.0f;
    owner->_ctx.data.telem.targetYawRad   = owner->_ctx.data.imu.yaw;
    owner->_ctx.data.output.pitchEn       = true;
    owner->_ctx.data.output.yawEn         = true;
    instance()->set_pitchstate(owner->_ctx.data.output.pitchEn);
    instance()->set_yawstate(owner->_ctx.data.output.yawEn);
}

void pyro::wl_gimbal_t::fsm_active_t::state_manual_t::execute(owner *owner) 
{
    owner->_ctx.data.telem.targetPitchRad += owner->_ctx.data.telem.target_pitch_vel * owner->_ctx.data.dt;

    // if(owner->_ctx.data.telem.targetPitchRad >= PI*0.4f)
    // {
    //     owner->_ctx.data.telem.targetPitchRad = PI*0.4f;
    // }
    // else if(owner->_ctx.data.telem.targetPitchRad <= -PI*0.4f)
    // {
    //     owner->_ctx.data.telem.targetPitchRad = -PI*0.4f;
    // }
    
    owner->_ctx.data.telem.targetYawRad = owner->wrapAngle(
        owner->_ctx.data.telem.targetYawRad - owner->_ctx.data.telem.target_yaw_vel * owner->_ctx.data.dt);
    owner->updatePitch();
    owner->updateYaw();
 
}

void pyro::wl_gimbal_t::fsm_active_t::state_manual_t::exit(owner *owner)
{
    (void)owner;
}


