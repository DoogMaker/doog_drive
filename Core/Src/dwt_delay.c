#include "dwt_delay.h"
#include "stm32g4xx_hal.h"   // 提供 CoreDebug / DWT 寄存器映射与 SystemCoreClock

/* DWT 高精度延时实现（替代 HAL_Delay，避免依赖 SysTick 调度） */

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能 DWT 跟踪
    DWT->CYCCNT = 0;                                  // 计数器清零
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;             // 使能 cycle 计数器
}

void DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000U);  // us -> CPU 周期数
    while ((DWT->CYCCNT - start) < cycles);
}

void DWT_DelayMs(uint32_t ms)
{
    while (ms--) DWT_DelayUs(1000);
}
