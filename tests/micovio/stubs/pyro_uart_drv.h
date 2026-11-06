#ifndef MICOVIO_TEST_UART_H
#define MICOVIO_TEST_UART_H
#include "pyro_task.h"
#include <functional>
#include <vector>
namespace pyro
{
class uart_drv_t
{
  public:
    using callback_t = std::function<bool(uint8_t *, uint16_t, BaseType_t &)>;
    callback_t callback{};
    status_t reset_result{PYRO_OK}, rx_result{PYRO_OK};
    uint32_t baud{};
    bool receiving{};
    status_t reset(uint32_t rate, uint32_t, uint32_t, uint32_t)
    {
        baud = rate;
        receiving = false;
        return reset_result;
    }
    void add_rx_event_callback(const callback_t &cb, uint32_t) { callback = cb; }
    status_t remove_rx_event_callback(uint32_t) { callback = {}; return PYRO_OK; }
    status_t enable_rx_dma() { receiving = rx_result == PYRO_OK; return rx_result; }
    status_t disable_rx_dma() { receiving = false; return PYRO_OK; }
    void inject(std::vector<uint8_t> bytes)
    {
        BaseType_t woken = 0;
        if (receiving && callback)
            callback(bytes.data(), static_cast<uint16_t>(bytes.size()), woken);
    }
};
}
#endif
