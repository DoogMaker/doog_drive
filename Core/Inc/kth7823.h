#ifndef __KTH7823_H
#define __KTH7823_H

#include "stm32g4xx_hal.h"
#include <math.h>


/* ==================== 刷新方式与读取协议 ====================
 * 刷新位置：唯一路径是 main.c 主循环（while(1) 每轮调用 KTH7823_UpdateAngle + UpdateSpeed，
 *           末尾 DWT_DelayMs(1) 定节拍，刷新率 ≈1kHz；20kHz 控制中断读取，角度最旧约 1ms）。
 * 读取协议：KTH7823 为流水线 SPI —— 当前帧发送命令，返回的是"上一命令帧的回读"。
 *           因此每拍只需发 1 帧：命令仍是读角度，返回值即上一拍命令对应的角度（滞后一帧
 *           ≈1.78µs @9MHz，可忽略）。CS 每帧拉高一次，满足"两帧之间 ≥50ns 高电平"要求。
 * ============================================================ */

#define KTH7823_ANGLE_RESOLUTION  65536.0f  // 16位分辨率  
#define ANGLE_FILTER_ALPHA    0.10f
#define SPEED_FILTER_ALPHA    0.15f   // 速度一阶低通滤波系数（越小越平滑，响应越慢）
#define KTH7823_SPI           (&hspi1)  

// 连续异常读数（0x0000 / 0xFFFF）次数阈值：达到即判编码器读数失效（断线/无磁）
#define KTH7823_RAW_FAULT_LIMIT   8u

// 多圈解包裹"野值剔除"上限
#define KTH7823_UNWRAP_MAX_DELTA   1024

void KTH7823_Init(void);    
uint16_t KTH7823_ReadAngleRaw(void);    // 发 1 帧读命令，返回上一命令帧的回读（流水线，滞后一帧）
void KTH7823_UpdateAngle(void);
void KTH7823_UpdateSpeed(void);

// 刷新心跳计数器：KTH7823_UpdateAngle 每执行一次 +1，作采样心跳（供 Protect 判断是否在刷新）
extern volatile uint32_t encoder_update_count;

// 连续读到异常原始值（0x0000/0xFFFF）的次数，供 Protect 判定读数有效性；正常读数清零
extern volatile uint16_t kth7823_raw_fault_cnt;

// 被丢弃的解包裹野值累计次数（诊断用，只增不清零）
extern volatile uint16_t kth7823_unwrap_glitch_cnt;

#endif