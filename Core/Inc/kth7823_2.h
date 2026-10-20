#ifndef __KTH7823_2_H
#define __KTH7823_2_H

#include "stm32g4xx_hal.h"
#include <math.h>

// 第二编码器（减速器输出端，SPI3）
// 芯片型号待定，本模块为对称于 KTH7823 的框架；
// 芯片相关的初始化/读取协议见 Encoder2_Init / Encoder2_ReadAngleRaw 内的 TODO。

#define ENCODER2_ANGLE_RESOLUTION  65536.0f  // 16位分辨率（按实际芯片调整）
#define ENCODER2_FILTER_ALPHA      0.10f     // sin/cos 一阶低通系数
#define ENCODER2_SPI               (&hspi3)

void Encoder2_Init(void);
uint16_t Encoder2_ReadAngleRaw(void);
void Encoder2_UpdateAngle(void);
void Encoder2_UpdateSpeed(void);

// 输出端角度/速度
extern volatile float Encoder2_Angle_Rad;       // 单圈角度 [0, 2π)
extern float Encoder2_Angle_Deg;                // 单圈角度（度）
extern float Encoder2_Angle_Multiturn;          // 多圈总角度（连续无界，弧度）
extern volatile float Encoder2_Speed_Rads;      // 速度 (rad/s)
extern float Encoder2_Speed_RPM;                // 速度 (RPM)

// 刷新心跳计数器：Encoder2_UpdateAngle 每调用一次 +1
extern volatile uint32_t encoder2_update_count;

#endif
