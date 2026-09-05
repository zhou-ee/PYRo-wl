#include "pyro_wl_chassis.h"
#include "stm32h7xx_hal_rcc_ex.h"
#include <algorithm>
#include <cmath>
float debug1;
float debug2;
namespace pyro
{

static int reset_count;

void wl_chassis_t::fsm_active_t::state_normal_t::state_balance_t::enter(wl_chassis_t *owner)
{

    owner->_ctx.data.odom.real_x            = 0;

    owner->_ctx.data.target_state.x         = 0;
    owner->_ctx.data.target_state.dot_x     = 0.0f;
    owner->_ctx.data.target_state.psi       = owner->_ctx.data.ins.euler_rad[0];
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
    // owner->_ctx.pid.leg_length[leg_def::L]->clear();
    // owner->_ctx.pid.leg_length[leg_def::R]->clear();


    owner->_ctx.motor.wheel[leg_def::L]->enable();
    owner->_ctx.motor.wheel[leg_def::R]->enable();
    owner->_ctx.data.airborne.state = chassis_function_state_t::NONE;
    owner->_ctx.data.airborne.takeoff_counter = 0;
    owner->_ctx.data.airborne.landing_counter = 0;
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_balance_t::execute(wl_chassis_t *owner)
{
    //紧急下力判断
    if(abs(owner->_ctx.data.ins.euler_rad[1]) >= PI / 4.0f ||
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

    debug1 = owner->_ctx.data.leg[leg_def::R].current_leg_radps - owner->_ctx.data.current_state.dot_theta;
    debug2 = owner->_ctx.data.leg[leg_def::R].current_leg_rad + owner->_ctx.data.current_state.theta;
    

    //自动上台阶判断
    static uint16_t auto_step_count = 0;
    static int press_forward_time = 0;
    //只有在按下前行1s后才可以进;
    if(owner->_ctx.data.target_state.dot_x > 0)
    {
        press_forward_time++;
    }
    else 
    {
        press_forward_time = 0;
    }

    if (owner->_ctx.data.current_state.L >= 0.30f &&
        owner->_ctx.data.leg[leg_def::R].current_leg_rad< 1.2f && 
        owner->_ctx.data.leg[leg_def::L].current_leg_rad< 1.2f &&
        press_forward_time >= 500)
    {
        if(auto_step_count >= 100)
        {
            request_switch(&owner->_state_active._state_normal._state_step);
        }
        auto_step_count++;
    }
    else
    {
        auto_step_count = 0;
    }

    //离地检测
    if (!owner->_ctx.data.airborne.landing_recovery &&
        owner->_detect_takeoff())
    {
        owner->_ctx.data.airborne.state = chassis_function_state_t::AIR;
        owner->_ctx.data.airborne.takeoff_counter = 0;
        owner->_ctx.data.airborne.landing_counter = 0;
        request_switch(&owner->_state_active._state_normal._state_air);
        return;
    }

    //落地回复
    if (owner->_ctx.data.airborne.landing_recovery)
    {
        owner->_execute_landing_recovery();
    }
    else
    {
        //腿长加上遥控器的小量
        const float target_dot_L = owner->_current_cmd.dot_L;
        owner->_ctx.data.target_state.dot_L = target_dot_L;
        owner->_ctx.data.target_state.L +=
        target_dot_L * owner->_ctx.data._dt;
        //腿长限幅
        owner->_ctx.data.target_state.L =
        std::clamp(owner->_ctx.data.target_state.L,
                       MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    }
    
    
    const float target_vx = owner->_current_cmd.v;
    owner->_ctx.data.target_state.x += target_vx * owner->_ctx.data._dt;
    owner->_ctx.data.target_state.dot_x = target_vx;

    //角速度设置
    float target_wz;
    target_wz =owner->_current_cmd.wz;
    // if(owner->_current_cmd.wz != 0)
    // {
    //     target_wz = owner->_current_cmd.wz;
    // }
    // else
    // {
    //     static const float WZ_KP = 3.0f;
    //     constexpr float YAW_ALIGN_TARGET_RAD = -2.2f;
    //     target_wz = WZ_KP * wrap2pi_f32_normalized(owner->_ctx.data.yaw.pos - YAW_ALIGN_TARGET_RAD);
    //     if(fabs(target_wz) < 0.1f )
    //     {
    //         target_wz = 0;
    //     }
    // }
    //




    owner->_ctx.data.target_state.psi += target_wz * owner->_ctx.data._dt;
    owner->_ctx.data.target_state.psi = loop_fp32_constrain(owner->_ctx.data.target_state.psi,-PI,PI);
    owner->_ctx.data.target_state.dot_psi = target_wz;

    

    owner->_gain_calculate();
    owner->_balance_control();

    const float roll_error = owner->_ctx.data.target_state.phi -
                             owner->_ctx.data.current_state.phi;
    if (std::fabs(roll_error) > NORMAL_ROLL_INTEGRAL_DEADBAND)
    {
        owner->_ctx.data.normal_roll_force_trim = std::clamp(
            owner->_ctx.data.normal_roll_force_trim +
                NORMAL_ROLL_INTEGRAL_KI * roll_error * owner->_ctx.data._dt,
            -NORMAL_ROLL_INTEGRAL_LIMIT, NORMAL_ROLL_INTEGRAL_LIMIT);
    }

    owner->_ctx.data.control.F_l1 = std::clamp(
        owner->_ctx.data.control.F_l1 +
            owner->_ctx.data.normal_roll_force_trim,
        -MAX_F_L, MAX_F_L);
    owner->_ctx.data.control.F_l2 = std::clamp(
        owner->_ctx.data.control.F_l2 -
            owner->_ctx.data.normal_roll_force_trim,
        -MAX_F_L, MAX_F_L);
    owner->_ctx.data.leg[leg_def::L].out_F_L =
        owner->_ctx.data.control.F_l1;
    owner->_ctx.data.leg[leg_def::R].out_F_L =
        owner->_ctx.data.control.F_l2;

    owner->_vmc_trans_v2j();
    owner->_send_joint_torque();
    owner->_send_wheel_torque();
}

void wl_chassis_t::fsm_active_t::state_normal_t::state_balance_t::exit(wl_chassis_t *owner)
{
    (void)owner;
}

} // namespace pyro
