#ifndef WARNING_H
#define WARNING_H

#include "pyro_task.h"
#include "ws2812_driver.h"
#include "pyro_buzzer_player.h"
#include "pyro_buzzer_task.h"
#include "pyro_buzzer_notes.h"

using namespace pyro;

class warning_drv_t
{
  public:
    enum class warning_type_t
    {
        NONE,//无报警
        PLAYING,//供自定义娱乐
        WARNING_TYPE_1,
        WARNING_TYPE_2,
        WARNING_TYPE_3,
        WARNING_TYPE_4,
        WARNING_TYPE_5,
        WARNING_TYPE_6,
        WARNING_TYPE_7,
        WARNING_TYPE_8,
        
    };
    static warning_drv_t& instance()
    {
        static warning_drv_t _inst("warning_task");
        return _inst;
    }

    // 暴露给 App 层的生命周期控制接口
    void start();
    void stop();


    void set_warning_level(warning_type_t level)
    {
        _current_warning_level = level;
    }

  protected:

    // 构造函数接收引用
    warning_drv_t(const char *task_name);


  private:
    // ---------------------------------------------------------
    // 【架构精髓】通过私有内部类实现组合，代理任务的运行逻辑
    // ---------------------------------------------------------
    class warning_task_t : public task_base_t
    {
      public:
      


        warning_task_t(warning_drv_t *parent, const char *name);

      protected:
        status_t init() override;
        void run_loop() override;

      private:
        warning_drv_t *_parent;
    };

    // 供内部任务调用的实际执行函数
    status_t task_init();
    void task_run_loop();

    warning_task_t _task;
    warning_type_t _current_warning_level = warning_type_t::NONE;
    BuzzerTask* _buzzer_task;  // ← 新增：旋律任务指针

};





#endif