#ifndef __KTH7823_H
#define __KTH7823_H

#include "stm32g4xx_hal.h"
#include <math.h>


#define KTH7823_ANGLE_RESOLUTION  65536.0f  // 16位分辨率  
#define ANGLE_FILTER_ALPHA    0.10f
#define SPEED_FILTER_ALPHA    0.15f   // 速度一阶低通滤波系数（越小越平滑，响应越慢）
#define KTH7823_SPI           (&hspi1)  

// 连续异常读数（0x0000 / 0xFFFF）次数阈值：达到即判编码器读数失效（断线/无磁）
#define KTH7823_RAW_FAULT_LIMIT   8u

// 多圈解包裹"野值剔除"上限
#define KTH7823_UNWRAP_MAX_DELTA   1024

void KTH7823_Init(void);    
uint16_t KTH7823_ReadAngleRaw(void);    
void KTH7823_UpdateAngle(void);
void KTH7823_UpdateSpeed(void);

/**
 * @brief 编码器刷新入口（供 TIM3 中断调用）
 * @note  内部依次执行 UpdateAngle + UpdateSpeed；中断内为唯一写者
 */
void KTH7823_UpdateISR(void);

// 刷新心跳计数器：KTH7823_UpdateISR 每执行一次 +1（由 UpdateAngle 递增），作采样心跳
extern volatile uint32_t encoder_update_count;

// 连续读到异常原始值（0x0000/0xFFFF）的次数，供 Protect 判定读数有效性；正常读数清零
extern volatile uint16_t kth7823_raw_fault_cnt;

// 被丢弃的解包裹野值累计次数（诊断用，只增不清零）
extern volatile uint16_t kth7823_unwrap_glitch_cnt;

#endif