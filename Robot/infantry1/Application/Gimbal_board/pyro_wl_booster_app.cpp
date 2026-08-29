#include "pyro_can_drv.h"
#include "pyro_wl_booster.h"
#include "pyro_bsp_can.h"
#include "pyro_dji_motor_drv.h"
#include "booster_config.h"
#include "pyro_rw_lock.h"
#include "pyro_shared_data_def.h"

using namespace pyro;








static TaskHandle_t booster_task_handle = nullptr;
static pyro::wl_booster_t *wl_booster_ptr         = nullptr;
static pyro::wl_booster_cmd_t *wl_booster_cmd_ptr = nullptr;
static pyro::wl_booster_deps_t *wl_booster_deps   = nullptr;

GimbalBoosterShared shared_data;
rw_lock g_booster_shared_lock;

static void motor_deps_init();
static void booster_cmd(GimbalBoosterShared cmd);


extern "C"
{
    void wl_booster_thread(void *argument)
    {
        while(true)
        {
            GimbalBoosterShared cmd;
            //读者
            {
                pyro::read_scope_lock lock(g_booster_shared_lock);
                cmd = shared_data;  // 拷贝一份，锁尽快释放
            }
            booster_cmd(cmd);
            wl_booster_ptr->set_command(*wl_booster_cmd_ptr);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    void infantry1_booster_init(void *argument)
    {
        

        wl_booster_cmd_ptr = new pyro::wl_booster_cmd_t();
        wl_booster_ptr     = pyro::wl_booster_t::instance();

        motor_deps_init();
        wl_booster_ptr->configure(*wl_booster_deps);
        wl_booster_ptr->start();

        xTaskCreate(wl_booster_thread, "infantry_booster_thread", 256, 
                    nullptr,configMAX_PRIORITIES - 1, &booster_task_handle);

        
        vTaskDelete(nullptr);
    }
}


void booster_cmd(GimbalBoosterShared cmd)
{
    wl_booster_cmd_ptr->mode = (cmd_base_t::mode_t)(cmd.mode);
    wl_booster_cmd_ptr->event = (ShootEvent)(cmd.event);
}





void motor_deps_init()
{
    wl_booster_deps = new pyro::wl_booster_deps_t();

    // 初始化电机

    wl_booster_deps->motor_deps.fric1 =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1,pyro::bsp_can::can2);
    wl_booster_deps->motor_deps.fric2 =
        new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2,pyro::bsp_can::can2);
    wl_booster_deps->motor_deps.trigger =
        new pyro::dji_m2006_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3,pyro::bsp_can::can1);

    // 初始化串级 PID

    wl_booster_deps->pid_deps.fric1_spd =
        new pyro::pid_t(FRIC_SPEED_PID_KP, 0.0f, FRIC_SPEED_PID_KD, 0.0f, 20.0f);
    wl_booster_deps->pid_deps.fric2_spd =
        new pyro::pid_t(FRIC_SPEED_PID_KP,0.0f, FRIC_SPEED_PID_KD, 0.0f, 20.0f);
    wl_booster_deps->pid_deps.trigger_pos =
        new pyro::pid_t(TRIGGER_POS_PID_KP, 0.0f, TRIGGER_POS_PID_KD, 100.0f, 1000.0f);
    wl_booster_deps->pid_deps.trigger_spd =
        new pyro::pid_t(TRIGGER_SPEED_PID_KP, 0.0f,TRIGGER_SPEED_PID_KD, 5.0f, 10.0f);
}