#ifndef __KTH7823_H
#define __KTH7823_H

#include "stm32g4xx_hal.h"
#include <math.h>


#define KTH7823_ANGLE_RESOLUTION  65536.0f  // 16位分辨率  
#define ANGLE_FILTER_ALPHA    0.10f
#define SPEED_FILTER_ALPHA    0.15f   // 速度一阶低通滤波系数（越小越平滑，响应越慢）
#define KTH7823_SPI           (&hspi1)  

void KTH7823_Init(void);    
uint16_t KTH7823_ReadAngleRaw(void);    
void KTH7823_UpdateAngle(void);
void KTH7823_UpdateSpeed(void);

// 刷新心跳计数器：KTH7823_UpdateAngle 每调用一次 +1，供失速/断线监测
extern volatile uint32_t encoder_update_count;

#endif