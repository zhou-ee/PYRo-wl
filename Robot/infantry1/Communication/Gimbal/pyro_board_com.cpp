#include "pyro_board_drv.h"
#include "pyro_module_base.h"
#include "pyro_rc_base_drv.h"
#include "pyro_vt03_rc_drv.h"

using namespace pyro;

//底盘部分
constexpr uint32_t EVENT_BIT_STEPCLIMB                = (1 << 0);     // - 左上按钮双击 上台阶
constexpr uint32_t EVENT_BIT_SPINING                  = (1 << 1);     // - pause键 小陀螺
constexpr uint32_t EVENT_BIT_LEG_LENGTH_MODE          = (1 << 2);     // - 左上按钮单击切换腿长变长变短或不动

static TaskHandle_t board_com_task_handl    = nullptr;
static board_drv_t *board_drv_ptr           = nullptr;


void chassis_vt03cmd(uint32_t notify)
{
    auto &tx_data = board_drv_ptr->get_g2c_tx_data();

    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc              = pyro::rc_drv_t::read();
    //判断当前模式
    if(vt03_drv_t::instance().check_online())
    {
        if(vrc.switches.gear.current_pos == pyro::sw_pos_t::UP)
        {
            tx_data.mode      = 0;
            tx_data.step_mode = 0;
            tx_data.spining   = 0;
            tx_data.delta_leg = 0;
            tx_data.vx        = 0;
            tx_data.w         = 0; 
        }
        else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::MID)
        {
            tx_data.mode      = 1;
            tx_data.step_mode = 0;
            tx_data.delta_leg = 0;
            tx_data.vx        = vrc.axes.rx * 31.0f;
            tx_data.w         = vrc.axes.ry * 31.0f;

            if(notify & EVENT_BIT_SPINING)
            {
                tx_data.spining   = !tx_data.spining;
            }
        }
        else if(vrc.switches.gear.current_pos == pyro::sw_pos_t::DOWN)
        {
            tx_data.mode      = 2;
            tx_data.step_mode = (notify & EVENT_BIT_STEPCLIMB);
            tx_data.vx        = vrc.axes.ry * 31.0f;
            tx_data.w         = vrc.axes.rx * 31.0f;

            if(notify & EVENT_BIT_SPINING)
            {
                tx_data.spining   = !tx_data.spining;
            }
            //腿长命令逻辑遵循以下循环：不变->变长->不变->变短，循环往复
            static int count = 0;
            if(notify & EVENT_BIT_LEG_LENGTH_MODE)
            {
                count++;
                count %= 4;
            }

            if(count == 0 || count == 2)
            {
                tx_data.delta_leg = 0;
            }
            else if(count == 1)
            {
                tx_data.delta_leg = 1;
            }
            else if(count == 3)
            {
                tx_data.delta_leg = 2;
            }
        }
    }
    else 
    {
        tx_data.mode      = 0;
        tx_data.step_mode = 0;
        tx_data.spining   = 0;
        tx_data.delta_leg = 0;
        tx_data.vx        = 0;
        tx_data.w         = 0; 
    }
    
}

extern "C"
{
    void infantry1_board_com_thread(void *argument)
    {
        
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);
            chassis_vt03cmd(notify_val);
            board_drv_ptr->send_data();
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_board_com_init(void *argument)
    {   // 板间通信启动
        // 云台角色，使用 CAN1
        

        board_drv_ptr = &board_drv_t::get_instance(pyro::board_drv_t::role_t::GIMBAL,pyro::bsp_can::can1);
        board_drv_ptr->start_rx();


        xTaskCreate(infantry1_board_com_thread, "board_com_app", 128, nullptr,
                    configMAX_PRIORITIES - 3, &board_com_task_handl);

        auto &vrc = pyro::rc_drv_t::read();
        //这里添加要订阅的按键
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::DOUBLE_CLICK, 
                            board_com_task_handl, EVENT_BIT_STEPCLIMB);
        pyro::btn_broker::subscribe(&vrc.buttons.fn_l, pyro::btn_event_t::SINGLE_CLICK, 
                            board_com_task_handl, EVENT_BIT_LEG_LENGTH_MODE);
        pyro::btn_broker::subscribe(&vrc.buttons.pause, pyro::btn_event_t::SINGLE_CLICK, 
                            board_com_task_handl, EVENT_BIT_SPINING);

        vTaskDelete(nullptr);
    }
}