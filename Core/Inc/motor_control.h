#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <KTH7823.h>
#include <encoder2.h>
#include <foc.h>
#include <sense.h>

// LED指示灯
#define Run_ON()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define Run_OFF() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)

// 电机控制状态机由 FOC.c 实现（FOC_STATE_*），本文件仅负责初始化与节拍。

extern volatile uint32_t control_tick;   // 控制节拍计数（motor_control.c 定义）
extern float control_dt;                 // 控制周期 (秒)，由 MotorControl_Init 计算

void MotorControl_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */