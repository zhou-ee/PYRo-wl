#include "pyro_micovio_drv.h"

#include "pyro_bsp_uart.h"
#include "pyro_core_config.h"

namespace pyro
{
micovio_drv_t::micovio_task::micovio_task(micovio_drv_t *parent)
    : task_base_t("micovio", 0, 1024, priority_t::ABOVE_NORMAL), _parent(parent)
{
}

status_t micovio_drv_t::micovio_task::init()
{
    return PYRO_OK;
}

void micovio_drv_t::micovio_task::run_loop()
{
    while (true)
    {
        _parent->poll();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

#ifdef VIO_UART
micovio_drv_t *micovio_drv_t::get_instance()
{
    static micovio_drv_t instance(VIO_UART);
    return &instance;
}
#endif

micovio_drv_t::micovio_drv_t(uart_drv_t &uart) : _uart(uart), _task(this)
{
    fifo_s_init(&_fifo, _fifo_buffer, sizeof(_fifo_buffer));
    _parser.set_callback(frame_callback, this);
}

micovio_drv_t::~micovio_drv_t()
{
    if (_initialized)
    {
        taskENTER_CRITICAL();
        _uart.remove_rx_event_callback(static_cast<uint32_t>(reinterpret_cast<uintptr_t>(this)));
        taskEXIT_CRITICAL();
    }
    _task.stop();
}

void micovio_drv_t::init(const micovio_config_t &config)
{
    if (_initialized)
        return;
    if (config.timeout_ms == 0 || config.timeout_ms > INT32_MAX)
        return;
    _config = config;
    // task_base_t::start() may return OK even when task creation fails.
    if (_task.start() != PYRO_OK || !_task.running())
        return;
    // The caller may already have enabled reception on this UART.
    taskENTER_CRITICAL();
    _uart.add_rx_event_callback(
        [this](uint8_t *data, uint16_t length, BaseType_t &) -> bool
        { return rx_callback(data, length); },
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(this)));
    _initialized = true;
    taskEXIT_CRITICAL();
}

bool micovio_drv_t::rx_callback(uint8_t *data, uint16_t length)
{
    if (length == 0 || _rx_overflow)
        return true;
    rx_chunk_header_t header{};
    header.rx_ms = HAL_GetTick();
    header.length = length;
    const size_t record_size = sizeof(header) + length;
    // Enqueue a whole timestamped chunk, preserving the frame completion time.
    if (static_cast<size_t>(fifo_s_free(&_fifo)) < record_size)
    {
        _rx_overflow = true;
        return true;
    }
    fifo_s_puts(&_fifo, reinterpret_cast<char *>(&header), sizeof(header));
    fifo_s_puts(&_fifo, reinterpret_cast<char *>(data), length);
    return true;
}

void micovio_drv_t::invalidate_stream()
{
    _parser.reset();
    _have_seq = false;
}

void micovio_drv_t::poll()
{
    uint8_t buffer[256];
    while (true)
    {
        rx_chunk_header_t header{};
        taskENTER_CRITICAL();
        if (_rx_overflow)
        {
            fifo_s_flush(&_fifo);
            _rx_overflow = false;
            invalidate_stream();
        }
        const bool have_chunk = fifo_s_used(&_fifo) >= static_cast<int>(sizeof(header));
        if (have_chunk)
        {
            fifo_s_gets(&_fifo, reinterpret_cast<char *>(&header), sizeof(header));
        }
        taskEXIT_CRITICAL();
        if (!have_chunk)
            break;
        if (HAL_GetTick() - header.rx_ms > _config.timeout_ms)
        {
            taskENTER_CRITICAL();
            fifo_s_flush(&_fifo);
            invalidate_stream();
            taskEXIT_CRITICAL();
            break;
        }
        _parse_rx_ms = header.rx_ms;
        size_t remaining = header.length;
        while (remaining > 0)
        {
            const size_t count = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
            fifo_s_gets(&_fifo, reinterpret_cast<char *>(buffer), count);
            _parser.feed(buffer, count);
            remaining -= count;
        }
    }
}

void micovio_drv_t::frame_callback(const micovio_frame_t &frame, void *user)
{
    static_cast<micovio_drv_t *>(user)->on_frame(frame);
}

void micovio_drv_t::on_frame(const micovio_frame_t &frame)
{
    taskENTER_CRITICAL();
    if (_rx_overflow || HAL_GetTick() - _parse_rx_ms > _config.timeout_ms)
    {
        taskEXIT_CRITICAL();
        return;
    }
    const bool estimator_reset = frame.message_id == MICOVIO_ODOMETRY_ID &&
                                 _odometry.reset_counter != frame.odometry.reset_counter;
    // Ignore repeated packets, while accepting estimator resets and reconnects.
    if (_have_seq && frame.seq == _last_seq && !estimator_reset &&
        _parse_rx_ms - _last_frame_rx_ms <= _config.timeout_ms)
    {
        taskEXIT_CRITICAL();
        return;
    }
    _last_seq = frame.seq;
    _have_seq = true;
    _last_frame_rx_ms = _parse_rx_ms;
    if (frame.message_id == MICOVIO_ODOMETRY_ID)
    {
        _odometry = frame.odometry;
    }
    else if (frame.message_id == MICOVIO_DISTANCE_ID)
    {
        _distance = frame.distance;
    }
    taskEXIT_CRITICAL();
}

} // namespace pyro
