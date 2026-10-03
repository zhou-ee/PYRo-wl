#include "pyro_warning_drv.h"

using namespace pyro;


// ===================================================================
// 私有内部任务类实现 (代理桥接)
// ===================================================================
warning_drv_t::warning_task_t::warning_task_t(warning_drv_t *parent, const char *name)
    : task_base_t(name, 0, 256, task_base_t::priority_t::REALTIME),
      _parent(parent)
{
}


warning_drv_t::warning_drv_t(const char *task_name)
    : _task(this, task_name)
{
}


status_t warning_drv_t::warning_task_t::init()
{
    return _parent->task_init();//_parent是拥有这个任务对象的类，放在这里就是warning_drv_t
    
}

void warning_drv_t::warning_task_t::run_loop()
{
    _parent->task_run_loop();
}

void warning_drv_t::start()
{
    _task.start();
}

void warning_drv_t::stop()
{
    _task.stop();
}

status_t warning_drv_t::task_init()
{
    auto& led = ws2812_t::get_instance();
    led.init(1); // 初始化一个 LED
    BuzzerPlayer::getInstance().init();

    // // See You Again 前奏｜严格按MuseScore原版五线谱时值，每个音符带静音断开
    // _buzzer_task = new BuzzerTask();
    // _buzzer_task->add_event(0,      BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(788,    0, 100);

    // _buzzer_task->add_event(888,    BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(1150,   0, 100);

    // _buzzer_task->add_event(1250,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(2038,   0, 100);

    // _buzzer_task->add_event(2138,   BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(2400,   0, 100);

    // _buzzer_task->add_event(2500,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(2762,   0, 100);

    // _buzzer_task->add_event(2862,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(3125,   0, 100);

    // _buzzer_task->add_event(3225,   BUZZER_GET_FREQ(BUZZER_NOTE_D5),  100);
    // _buzzer_task->add_event(3488,   0, 100);

    // _buzzer_task->add_event(3588,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(3850,   0, 100);

    // _buzzer_task->add_event(3950,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(4212,   0, 100);

    // _buzzer_task->add_event(4312,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(4575,   0, 100);

    // _buzzer_task->add_event(4675,   BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(5462,   0, 100);

    // _buzzer_task->add_event(5562,   BUZZER_GET_FREQ(BUZZER_NOTE_C5),  100);
    // _buzzer_task->add_event(5825,   0, 100);

    // _buzzer_task->add_event(5925,   BUZZER_GET_FREQ(BUZZER_NOTE_AS4), 100);
    // _buzzer_task->add_event(6188,   0, 100);

    // _buzzer_task->add_event(6288,   BUZZER_GET_FREQ(BUZZER_NOTE_F4),  100);
    // _buzzer_task->add_event(7075,   0, 100);

    // _buzzer_task->add_event(7175, 0, 100);
    // _buzzer_task->set_duration(7175);


    // 欢乐颂
    // 创建旋律任务（只创建一次）
    _buzzer_task = new BuzzerTask();
    // 第一句：E4 E4 F4 G4
    _buzzer_task->add_event(0,    BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(300,  0, 100);   // 静音停200ms
    _buzzer_task->add_event(500,  BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(800,  0, 100);   // 静音停200ms
    _buzzer_task->add_event(1000, BUZZER_GET_FREQ(BUZZER_NOTE_F4), 100);
    _buzzer_task->add_event(1300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(1500, BUZZER_GET_FREQ(BUZZER_NOTE_G4), 100);
    _buzzer_task->add_event(1800, 0, 100);   // 静音停200ms

    // 第二句：G4 F4 E4 D4
    _buzzer_task->add_event(2000, BUZZER_GET_FREQ(BUZZER_NOTE_G4), 100);
    _buzzer_task->add_event(2300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(2500, BUZZER_GET_FREQ(BUZZER_NOTE_F4), 100);
    _buzzer_task->add_event(2800, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(3000, BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(3300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(3500, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(3800, 0, 100);   // 静音停200ms

    // 第三句：C4 C4 D4 E4
    _buzzer_task->add_event(4000, BUZZER_GET_FREQ(BUZZER_NOTE_C4), 100);
    _buzzer_task->add_event(4300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(4500, BUZZER_GET_FREQ(BUZZER_NOTE_C4), 100);
    _buzzer_task->add_event(4800, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(5000, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(5300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(5500, BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(5800, 0, 100);   // 静音停200ms

    // 第四句：E4 D4 D4
    _buzzer_task->add_event(6000, BUZZER_GET_FREQ(BUZZER_NOTE_E4), 100);
    _buzzer_task->add_event(6300, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(6500, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(6800, 0, 100);   // 静音停200ms
    _buzzer_task->add_event(7000, BUZZER_GET_FREQ(BUZZER_NOTE_D4), 100);
    _buzzer_task->add_event(7300, 0, 100);

    _buzzer_task->add_event(8000, 0, 100);
    _buzzer_task->set_duration(8000);





    return status_t::PYRO_OK;
}

void warning_drv_t::task_run_loop()
{
    //根据当前的报警等级，控制蜂鸣器和灯带的状态
    auto& led = ws2812_t::get_instance();
    auto& buzzer = BuzzerPlayer::getInstance();

    while (true)
    {
        static int tick = 0;
        tick += 2;
        tick %= 1000; // 每5秒钟循环一次
        switch (_current_warning_level)
        {
        case warning_type_t::NONE:
        {
            // 处理无报警状态
            led.set_led(0,0, 0, 0);
            buzzer.stop();  // ← 改为 stop()，彻底停止
            break;
        }
        case warning_type_t::PLAYING:
        {
            // 处理娱乐状态
            //rgb部分
            static uint16_t hue = 0;
            hue = (hue + 1) % 360; // 色相滚动
            auto output = led.hsv_to_rgb(hue, 255, 100);
            led.set_led(0, output.r, output.g, output.b);

            // 蜂鸣器部分
            static bool melody_loaded = false;
            if (!melody_loaded && _buzzer_task != nullptr)
            {
                buzzer.loadTask(_buzzer_task);  // 加载旋律
                melody_loaded = true;
            }

            // 如果旋律播放完毕，重置标志以便循环播放
            if (!(buzzer.getState() & BUZZER_STATE_MUSIC_PLAYING_BIT))
            {
                melody_loaded = false;  // 重新加载
            }
            break;
        }
        case warning_type_t::WARNING_TYPE_1:
        {
            // 处理警告等级1
            // rgb部分
            // 每5秒闪烁一次红灯
            if(tick < 50)
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }
            break;
        }
        case warning_type_t::WARNING_TYPE_2:
        {
            // 处理警告等级2
            // rgb部分
            // 每5秒闪烁两次红灯
            if(tick < 50 || (tick >= 100 && tick < 150))
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }


            break;
        }
        case warning_type_t::WARNING_TYPE_3:
        {
            // 处理警告等级3
            // rgb部分
            // 每5秒闪烁三次红灯
            if (tick < 50 || (tick >= 100 && tick < 150) ||
                (tick >= 200 && tick < 250))
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }
            break;
        }
        case warning_type_t::WARNING_TYPE_4:
        {
            // 处理警告等级4
            // rgb部分
            // 每5秒闪烁四次红灯
            if (tick < 50 || (tick >= 100 && tick < 150) || 
                (tick >= 200 && tick < 250) || (tick >= 300 && tick < 350))
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }
            break;
        }
        case warning_type_t::WARNING_TYPE_5:
        {
            // 处理警告等级5
            // rgb部分
            // 每55秒闪烁五次红灯
            if (tick < 50 || (tick >= 100 && tick < 150) || 
                (tick >= 200 && tick < 250) || (tick >= 300 && tick < 350) ||
                (tick >= 400 && tick < 450))
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }
            break;
        }
        case warning_type_t::WARNING_TYPE_6:
        {
            // 处理警告等级6
            // rgb部分
            // 每5秒闪烁六次红灯
            if (tick < 50 || (tick >= 100 && tick < 150) || 
                (tick >= 200 && tick < 250) || (tick >= 300 && tick < 350) ||
                (tick >= 400 && tick < 450) || (tick >= 500 && tick < 550))
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }
            break;
        }
        case warning_type_t::WARNING_TYPE_7:
        {
            // 处理警告等级7
            // rgb部分
            // 每5秒闪烁七次红灯
            if (tick < 50 || (tick >= 100 && tick < 150) || 
                (tick >= 200 && tick < 250) || (tick >= 300 && tick < 350) ||
                (tick >= 400 && tick < 450) || (tick >= 500 && tick < 550) ||
                (tick >= 600 && tick < 650))
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }
            break;
        }
        case warning_type_t::WARNING_TYPE_8:
        {
            // 处理警告等级8
            // rgb部分
            // 每5秒闪烁八次红灯
            if (tick < 50 || (tick >= 100 && tick < 150) || 
                (tick >= 200 && tick < 250) || (tick >= 300 && tick < 350) ||
                (tick >= 400 && tick < 450) || (tick >= 500 && tick < 550) ||
                (tick >= 600 && tick < 650) || (tick >= 700 && tick < 750))
            {
                led.set_led(0, 255, 0, 0); // 红色
                buzzer.tone_set(1000, 100); // 频率1000Hz，音量100%
                buzzer.tone_start();
            }
            else
            {
                led.set_led(0, 0, 0, 0); // 灭灯
                buzzer.tone_stop();
            }
            break;
        }
        }
        
        led.update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}