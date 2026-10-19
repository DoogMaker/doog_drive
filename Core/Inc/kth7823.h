#ifndef __KTH7823_H
#define __KTH7823_H

#include "stm32g4xx_hal.h"
#include <math.h>


/* ==================== [TEST] 编码器刷新位置开关（对比测试用，测完可整体删除）====================
 * 本宏决定"谁在什么上下文里刷新 KTH7823"（单写者，两个分支互斥）：
 *   1 = 主循环轮询（旧版本方式，~1kHz）：main.c 的 while(1) 末尾调用 UpdateAngle + UpdateSpeed，
 *       并 DWT_DelayMs(1) 定节拍；TIM3 不启动、不进中断。
 *   0 = TIM3 更新中断（10kHz，原方案）：motor_control.c 启动 TIM3 并以 KTH7823_UpdateISR() 刷新。
 *
 * 为什么要有这个开关：需要对比"中断内刷新"与"主循环刷新"对测速噪声/控制动态的影响。
 *   1kHz 下 SPEED_FILTER_ALPHA/ANGLE_FILTER_ALPHA 的等效时间常数会放大约 10 倍（系数本身不改，
 *   否则失去对比意义）；20kHz 控制中断读到的角度最旧约 1ms。
 *
 * 测试通过（或需要恢复原方案）时：把下面的 1 改成 0 即可，无需改动其它代码。
 * 确认永久不需要本测试后删除方法：删除本段宏 + main.c 主循环里的 [TEST] 条件编译块
 *   + motor_control.c 里 [TEST] 标的 TIM3 启动/回调两处 #if 包裹（恢复为无条件执行）。
 * ============================================================================ */
#define ENCODER_REFRESH_IN_MAIN_LOOP   1    // 1 = 主循环刷新（测试）；0 = TIM3 中断刷新（原方案）

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