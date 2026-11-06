#ifndef PYRO_MICOVIO_DRV_H
#define PYRO_MICOVIO_DRV_H

#include "Protocol/pyro_micovio_protocol.h"
#include "fifo.h"
#include "pyro_task.h"
#include "pyro_uart_drv.h"
#include "pyro_core_config.h"

namespace pyro
{

struct micovio_config_t
{
    uint32_t timeout_ms{100};
};

class micovio_drv_t
{
  public:
    static constexpr size_t FIFO_BUFFER_SIZE = 8192;

#ifdef VIO_UART
    static micovio_drv_t *get_instance();
#endif
    explicit micovio_drv_t(uart_drv_t &uart);
    ~micovio_drv_t();
    micovio_drv_t(const micovio_drv_t &) = delete;
    micovio_drv_t &operator=(const micovio_drv_t &) = delete;

    // Task-context APIs. The caller owns UART configuration and DMA reception.
    void init(const micovio_config_t &config = {});
    // References remain valid for this object's lifetime. Callers synchronize
    // reads with reception-task updates when they need a consistent packet.
    [[nodiscard]] const micovio_odometry_t &get_odometry() const
    {
        return _odometry;
    }
    [[nodiscard]] const micovio_distance_t &get_distance() const
    {
        return _distance;
    }

  private:
    class micovio_task final : public task_base_t
    {
      public:
        explicit micovio_task(micovio_drv_t *parent);
        [[nodiscard]] bool running() const { return _loop_task_handle != nullptr; }
        status_t init() override;
        void run_loop() override;

      private:
        micovio_drv_t *_parent;
    };

    struct rx_chunk_header_t
    {
        uint32_t rx_ms{};
        uint16_t length{};
        uint16_t reserved{};
    };

    bool rx_callback(uint8_t *data, uint16_t length);
    void poll();
    static void frame_callback(const micovio_frame_t &frame, void *user);
    void on_frame(const micovio_frame_t &frame);
    void invalidate_stream();

    uart_drv_t &_uart;
    micovio_task _task;
    micovio_parser_t _parser;
    micovio_config_t _config{};
    fifo_s_t _fifo{};
    uint8_t _fifo_buffer[FIFO_BUFFER_SIZE]{};
    micovio_odometry_t _odometry{};
    micovio_distance_t _distance{};

    // ISR latches overflow until the task discards queued and partial input.
    volatile bool _rx_overflow{};
    uint32_t _parse_rx_ms{};
    uint32_t _last_frame_rx_ms{};
    uint8_t _last_seq{};
    bool _have_seq{};
    bool _initialized{};
};

} // namespace pyro

#endif
