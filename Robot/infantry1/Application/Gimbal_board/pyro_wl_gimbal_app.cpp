#include "pyro_can_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_core.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_wl_gimbal.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_bsp_can.h"
#include "pyro_dji_motor_drv.h"
#include "gimbal_config.h"
#include "pyro_shared_data_def.h"
#include "pyro_board_drv.h"


using namespace pyro;





//云台部分



//发射机构部分
constexpr uint32_t EVENT_BIT_FRIC_TOGGLE              = (1 << 3);
constexpr uint32_t EVENT_BIT_SINGLE_FIRE              = (1 << 4);
constexpr uint32_t EVENT_BIT_BURST_FIRE               = (1 << 5);
constexpr uint32_t EVENT_BIT_BURST_END                = (1 << 6);






static TaskHandle_t gimbal_task_handle = nullptr;

static pyro::wl_gimbal_t *wl_gimbal_ptr         = nullptr;
static pyro::wl_gimbal_cmd_t *wl_gimbal_cmd_ptr = nullptr;
static pyro::wl_gimbal_deps_t *wl_gimbal_deps   = nullptr;
static pyro::board_drv_t *board_ptr               = nullptr;

static virtual_rc_t vrc_t;




extern GimbalBoosterShared shared_data;
extern rw_lock g_booster_shared_lock;

static void motor_deps_init();

static void gimbal_vt03cmd(virtual_rc_t vrc, uint32_t notify);
static void booster_vt03cmd(virtual_rc_t vrc, uint32_t notify);

static void gimbal_dr16cmd(uint32_t notify){};
static void booster_dr16cmd(uint32_t notify){};



extern "C"
{
    void wl_gimbal_thread(void *argument)
    {
        while(true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);
            

            if(board_ptr->check_online())
            {
                const auto& c2g_data = board_ptr->get_c2g_rx_data();
                wl_gimbal_cmd_ptr->chassis_is_ready = c2g_data.chassis_is_align_ready;
            }
            else 
            {
                wl_gimbal_cmd_ptr->chassis_is_ready = true;
            }


            if (vt03_drv_t::instance().check_online())
            {
                pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
                vrc_t = pyro::rc_drv_t::read();
                gimbal_vt03cmd(vrc_t, notify_val);
                booster_vt03cmd(vrc_t, notify_val);
            }
            else if(dr16_drv_t::instance().check_online())
            {
                gimbal_dr16cmd(notify_val);
                booster_dr16cmd(notify_val);
            }
            else
            {
                wl_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
                wl_gimbal_cmd_ptr->state_cmd = pyro::MotionState::Relax;
            }

            wl_gimbal_ptr->set_command(*wl_gimbal_cmd_ptr);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_gimbal_init(void *argument)
    {
        

        wl_gimbal_cmd_ptr = new pyro::wl_gimbal_cmd_t();
        wl_gimbal_ptr     = pyro::wl_gimbal_t::instance();

        motor_deps_init();
        wl_gimbal_ptr->configure(*wl_gimbal_deps);
        wl_gimbal_ptr->start();

        // 板间通信启动
        // 云台角色，使用 CAN1
        *board_ptr = pyro::board_drv_t::get_instance(pyro::board_drv_t::role_t::GIMBAL,pyro::bsp_can::can1);
        board_ptr->start_rx();


        xTaskCreate(wl_gimbal_thread, "infantry_gimbal_thread", 256, 
                    nullptr,configMAX_PRIORITIES - 1, &gimbal_task_handle);

        auto &vrc = pyro::rc_drv_t::read();

        //这里添加要订阅的按键
        pyro::btn_broker::subscribe(&vrc.buttons.fn_r, pyro::btn_event_t::SINGLE_CLICK, 
                            gimbal_task_handle, EVENT_BIT_FRIC_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::SINGLE_CLICK, 
                            gimbal_task_handle, EVENT_BIT_SINGLE_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::LONG_PRESS_START, 
                            gimbal_task_handle, EVENT_BIT_BURST_FIRE);
        pyro::btn_broker::subscribe(&vrc.buttons.trigger, pyro::btn_event_t::PRESS_UP, 
                            gimbal_task_handle, EVENT_BIT_BURST_END);
  
        vTaskDelete(nullptr);
    }
}



void gimbal_vt03cmd(virtual_rc_t vrc, uint32_t notify)
{
    //判断当前模式
    if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP)
    {
        wl_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
        wl_gimbal_cmd_ptr->state_cmd = pyro::MotionState::Relax;

    }
    else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN ||
            vrc.switches.gear.current_pos == pyro::sw_pos_t::MID)
    {
        wl_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        wl_gimbal_cmd_ptr->state_cmd = pyro::MotionState::Manual;

        float pitchInput =vrc.axes.ly+vrc.mouse_axes.y*100.0f;
        if(pitchInput > 1.0f)
        {
            pitchInput=1.0f;
        }
        else if(pitchInput < -1.0f)
        {
            pitchInput=-1.0f;
        }
        wl_gimbal_cmd_ptr->pitchVel = -pitchInput * 2.0f;

        float yawInput =vrc.axes.lx+vrc.mouse_axes.x*100.0f;
        if(yawInput > 1.0f)
        {
            yawInput=1.0f;
        }
        else if(yawInput < -1.0f)
        {
            yawInput=-1.0f;
        }
        wl_gimbal_cmd_ptr->yawVel = yawInput*3.0f;
    }
}


void booster_vt03cmd(virtual_rc_t vrc, uint32_t notify)
{
    {
        pyro::write_scope_lock lock(g_booster_shared_lock);
        if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP)
        {
            shared_data.mode = 0;//Passive
        }
        else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID ||
                vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN)
        {
            shared_data.mode = 1;//Active
        }

        if(notify & EVENT_BIT_FRIC_TOGGLE)
        {
            shared_data.event = 1;
        }
        else if(notify & EVENT_BIT_BURST_FIRE)
        {
            shared_data.event = 3;
        }
        else if(notify & EVENT_BIT_SINGLE_FIRE)
        {
            shared_data.event = 2;
        }
        else if(notify & EVENT_BIT_BURST_END)
        {
            shared_data.event = 4;
        }
        else 
        {
            shared_data.event = 0;
        }
    }
}




void motor_deps_init()
{
    wl_gimbal_deps = new pyro::wl_gimbal_deps_t();

    // 初始化电机
    wl_gimbal_deps->motor_deps.pitch = 
        new pyro::dm_motor_drv_t(0x01, 0x00, pyro::bsp_can::can2);;
    wl_gimbal_deps->motor_deps.yaw = 
        new pyro::dji_gm_6020_motor_drv_t(pyro::dji_motor_tx_frame_t::id_5, pyro::bsp_can::can1);
    wl_gimbal_deps->motor_deps.pitch->set_position_range(-12.5f,12.5f); // 设定位置限位 (rad)
    wl_gimbal_deps->motor_deps.pitch->set_rotate_range(-30.0f,30.0f); // 设定速度限位 (rad/s)
    wl_gimbal_deps->motor_deps.pitch->set_torque_range(-10.0f,10.0f); // 设定扭矩限位 (N.m)
    wl_gimbal_deps->pid_deps.pitch_pos =
        new pyro::pid_t(DM_POS_PITCH_KP, 0.0f, DM_POS_PITCH_KD, 10.0f, 24.0f);
    wl_gimbal_deps->pid_deps.yaw_pos =
        new pyro::pid_t(YAW_POS_PID_KP, YAW_POS_PID_KI, YAW_POS_PID_KD, 10.0f, 20.0f);
    wl_gimbal_deps->pid_deps.yaw_spd =
        new pyro::pid_t(YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, 0.0f, 20.0f);
    
    // 设置 MIT 模式下的阻抗参数 (若使用串级PID输出扭矩，Kp和Kd必须设为0)
    wl_gimbal_deps->motor_deps.pitch->set_runtime_kp(DM_MOT_PITCH_KP);
    wl_gimbal_deps->motor_deps.pitch->set_runtime_kd(DM_MOT_PITCH_KD);
    // wl_gimbal_deps->motor_deps.pitch->set_runtime_kp(0);
    // wl_gimbal_deps->motor_deps.pitch->set_runtime_kd(0);
}

