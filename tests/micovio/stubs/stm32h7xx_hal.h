#ifndef MICOVIO_TEST_HAL_H
#define MICOVIO_TEST_HAL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
uint32_t HAL_GetTick(void);
static inline unsigned long __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(unsigned long value) { (void)value; }
#ifdef __cplusplus
}
#endif
#define UART_WORDLENGTH_8B 8
#define UART_STOPBITS_1 1
#define UART_PARITY_NONE 0
#endif
