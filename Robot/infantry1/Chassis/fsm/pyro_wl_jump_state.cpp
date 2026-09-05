#include "pyro_algo_common.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_wl_chassis.h"

#include <algorithm>

namespace pyro
{
    void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::enter(wl_chassis_t *owner)
    {
        owner->_ctx.data.odom.real_x            = 0;

        owner->_ctx.data.target_state.x         = 0;
        owner->_ctx.data.target_state.dot_x     = 0.0f;
        #if Using_Gimbal_Cmd
        owner->_ctx.data.target_state.psi       = owner->_ctx.data.current_state.psi;
        #else
        owner->_ctx.data.target_state.psi       = owner->_ctx.data.ins.euler_rad[0];
        #endif
        owner->_ctx.data.target_state.dot_psi   = 0.0f;
        owner->_ctx.data.target_state.L =
            owner->_ctx.data.airborne.landing_recovery
                ? owner->_ctx.data.airborne.L_ref
                : NORMAL_LENGTH_TARGET;
        owner->_ctx.data.target_state.dot_L     = 0.0f;
        owner->_ctx.data.target_state.theta     = 0.0f;
        owner->_ctx.data.target_state.dot_theta = 0.0f;
        owner->_ctx.data.target_state.phi       = 0.0f;
        owner->_ctx.data.target_state.dot_phi   = 0.0f;
        owner->_ctx.data.target_state.beta1     = 0.0f;
        owner->_ctx.data.target_state.beta2     = 0.0f;
        owner->_ctx.data.target_state.dot_beta1 = 0.0f;
        owner->_ctx.data.target_state.dot_beta2 = 0.0f;
        owner->_ctx.data.normal_roll_force_trim = 0.0f;
        
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


        owner->_ctx.motor.wheel[leg_def::L]->enable();
        owner->_ctx.motor.wheel[leg_def::R]->enable();
        owner->_ctx.data.airborne.state = chassis_function_state_t::NONE;
        owner->_ctx.data.airborne.takeoff_counter = 0;
        owner->_ctx.data.airborne.landing_counter = 0;
    }

    void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::execute(wl_chassis_t *owner)
    {
        request_switch(&owner->_state_active._state_normal._state_balance);
    }

    void wl_chassis_t::fsm_active_t::state_normal_t::state_jump_t::exit(wl_chassis_t *owner)
    {
        (void)owner;
    }

}