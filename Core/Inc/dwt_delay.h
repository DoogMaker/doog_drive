#ifndef DWT_DELAY_H
#define DWT_DELAY_H

#include <stdint.h>

/* DWT 高精度延时（替代 HAL_Delay，基于 Cortex-M 内核 DWT cycle 计数器，us 级精度） */

void DWT_Init(void);            // 使能 DWT 计数器（在 SystemClock_Config 后调用一次）
void DWT_DelayUs(uint32_t us);  // 微秒级阻塞延时
void DWT_DelayMs(uint32_t ms);  // 毫秒级阻塞延时（内部调用 DWT_DelayUs）

#endif /* DWT_DELAY_H */
