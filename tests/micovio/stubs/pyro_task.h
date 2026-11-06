#ifndef MICOVIO_TEST_TASK_H
#define MICOVIO_TEST_TASK_H
#include "pyro_core_def.h"
#include "stm32h7xx_hal.h"
using BaseType_t = int;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(ms) (ms)
struct micovio_test_yield {};
inline void vTaskDelay(uint32_t) { throw micovio_test_yield{}; }
namespace pyro
{
// Execute one production task iteration, yielding at vTaskDelay().
class task_base_t
{
  public:
    enum class priority_t { ABOVE_NORMAL };
    inline static task_base_t *last_started{};
    inline static bool fail_creation{};
    task_base_t(const char *, uint16_t, uint16_t, priority_t) {}
    virtual ~task_base_t() = default;
    status_t start()
    {
        if (!fail_creation)
        {
            _loop_task_handle = this;
            last_started = this;
        }
        return PYRO_OK;
    }
    void stop()
    {
        _loop_task_handle = nullptr;
        if (last_started == this)
            last_started = nullptr;
    }
    void run_once()
    {
        try { run_loop(); }
        catch (const micovio_test_yield &) {}
    }
  protected:
    virtual status_t init() = 0;
    virtual void run_loop() = 0;
    void *_loop_task_handle{};
};
}
#endif
