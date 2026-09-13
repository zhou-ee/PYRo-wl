#include "pyro_bsp_can.h"
#include "pyro_bsp_uart.h"
#include "pyro_dwt_drv.h"
#include "pyro_ins.h"
#include "pyro_referee.h"
#include "pyro_supercap_drv.h"

namespace pyro
{
can_drv_t *can1_drv = nullptr;
can_drv_t *can2_drv = nullptr;
can_drv_t *can3_drv = nullptr;
ins_drv_t *ins_drv = nullptr;
referee_drv_t *referee_drv = nullptr;

extern "C"
{
void pyro_init_thread(void *argument)
{
    dwt_drv_t::init(480);
    bsp_uart::init_all();
    bsp_can::init_all();

    can1_drv = &bsp_can::get_can1();
    can2_drv = &bsp_can::get_can2();
    can3_drv = &bsp_can::get_can3();

    ins_drv = ins_drv_t::get_instance();
    ins_config_t ins_cfg;
    ins_cfg.calibrate = IMU_CALIBRATION_EN;
    ins_cfg.direct = ins_config_t::imu_direct_t::DIRECT_3;
    ins_cfg.gx_offset = 0.00132003485f;
    ins_cfg.gy_offset = 0.00487931306f;
    ins_cfg.gz_offset = -8.64639369e-05f;
    ins_cfg.g_norm = 9.83213902f;
    ins_drv->init(ins_cfg);

#ifdef REFEREE_UART
    REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                       UART_PARITY_NONE);
    REFEREE_UART.enable_rx_dma();
    referee_drv = referee_drv_t::get_instance();
    referee_drv->init();
#endif

#ifdef SUPERCAP_UART
    SUPERCAP_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                        UART_PARITY_NONE);
    SUPERCAP_UART.enable_rx_dma();
    supercap_drv_t::get_instance()->start_rx();
#endif

    vTaskDelete(nullptr);
}
}
} // namespace pyro
