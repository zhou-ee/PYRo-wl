#include "pyro_wl_chassis.h"

namespace pyro
{

void wl_chassis_t::state_passive_t::enter(wl_chassis_t *owner)
{
    owner->_ctx.motor.wheel[leg_def::L]->disable();
    owner->_ctx.motor.wheel[leg_def::R]->disable();
}

void wl_chassis_t::state_passive_t::execute(wl_chassis_t *owner)
{
    for (auto &leg : owner->_ctx.data.leg)
    {
        leg.target_leg_length = leg.current_leg_length;
        leg.target_leg_rad = leg.current_leg_rad;
        leg.target_leg_speed = leg.current_leg_speed;
        leg.target_leg_radps = leg.current_leg_radps;
        leg.out_F_L = -leg.gas_spring_force;
        leg.out_T_p = 0.0f;
    }
    for (auto &wheel : owner->_ctx.data.wheel)
    {
        wheel.out_T_w = 0.0f;
        wheel.out_current = 0.0f;
    }
    owner->_vmc_trans_v2j(false);
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::state_passive_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
