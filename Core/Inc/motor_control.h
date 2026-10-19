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

extern volatile uint32_t control_tick;   // 控制节拍计数
extern float control_dt;                 // 控制周期 (秒)

// 初始化与节拍
void MotorControl_Init(void);

// 上电自动执行启动序列
void MotorControl_BootSequence(void);

// 控制命令门控：1=放行 0=拒绝（硬件错误锁死后拒绝控制命令，诊断命令不调用本函数）
uint8_t MotorControl_CommandGate(void);

// 上电自检是否失败（1=硬件错误、控制命令已锁死）：供 FOC.C 的 LED 状态机读取
uint8_t MotorControl_BootIsHwError(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */