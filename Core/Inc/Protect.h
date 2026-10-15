#ifndef __PROTECT_H
#define __PROTECT_H

#include "stm32g4xx_hal.h"

// 堵转保护：大电流 + 转速接近 0 才判定（正常高速大电流不误触发）
// 滞回由 FOC.c 的 FOC_STATE_FAULT 状态机退避冷却实现，无需本地锁存
#define STALL_CURRENT_LIMIT   45.0f   // 堵转判定电流（三相合成幅值 A）
#define STALL_SPEED_THRESHOLD 10.0f   // 堵转判定转速 (rad/s)，|speed| 低于此且大电流才判堵转

// 单相峰值过流（瞬时硬保护，响应最快）
// 直接检测单相瞬时电流绝对值，捕获不对称尖峰/短路，超过 50A 额定峰值 10% 立即停机
#define PHASE_PEAK_CURRENT_LIMIT  55.0f   // 单相瞬时电流硬上限 (A)

// I²t 热积累保护（模拟绕组热积累，防持续过流烧电机）
// 模型：i2t_accum += (I_rms² - I_nom²) × dt，下限 0；I_rms² = i_mag²/3（三相合成幅值平方的 1/3）
//   I_nom 为允许长期运行的相 RMS 电流（散热平衡点），超过则累积、低于则冷却
//   额定 14A：50A 峰值（RMS 35.4A）净积累速率 = 35.4²-14² ≈ 1054 A²/s，约 1.5s 触发
//   即允许 50A 峰值短时运行约 1.5s，随后必须降回额定内散热
#define I2T_NOMINAL_CURRENT   14.0f     // 允许长期运行的相 RMS 电流 (A) = 电机额定电流
#define I2T_TRIP_THRESHOLD    1600.0f   // 触发阈值 (A²·s)：50A 峰值约 1.5s
#define I2T_RESET_THRESHOLD   100.0f    // 恢复阈值 (A²·s)，热积累回落到此才允许恢复
#define I2T_RECHECK_DELAY     2000      // 恢复重检周期 (ms)

// ---------------------------------------------------------------------------
// 控制节拍基准：TIM1 中心对齐 10kHz PWM + RCR=0
//   -> 更新中断在每个 PWM 周期触发 2 次 = 20kHz，即 1 拍 = 50µs
//   下面所有 *_TICKS 宏均按 50µs/拍 换算；
//   改 TIM1 的 ARR / RCR（见 motor_control.c 的 control_dt 推导）时必须同步复核这里。
// ---------------------------------------------------------------------------

// 位置环相关阈值
// 注意：位置相关单位均为减速器输出端弧度（来自 Encoder2_Angle_Multiturn）
#define POS_ERR_TIMEOUT_TICKS   1000   // 位置跟随误差持续超限的控制周期数（20kHz 下 = 50ms）
#define SOFT_LIMIT_TIMEOUT_TICKS 400   // 目标位置在软限位外持续未收敛的控制周期数（20kHz 下 = 20ms）

// 编码器读数有效性/断线监测阈值
// 编码器刷新已在 TIM3 中断（10kHz）内完成，主循环存活由 IWDG 单独保障；
// 判据改为"读数有效性"：连续读到异常原始值（0x0000/0xFFFF）达 KTH7823_RAW_FAULT_LIMIT
// 次即判定编码器断线/无磁失效（该阈值定义在 kth7823.h，本检测在 20kHz 控制中断内调用）。
#define ENCODER_ARM_DELAY_MS  100    // 编码器开始刷新后再观察 100ms，避免上电误触发

// 上位机通讯失联保护（CAN 心跳看门狗）
// 上位机以 1kHz（每 1ms 一帧）周期性发送 CAN 帧，每收到一帧调用 Protect_CommFeed() 刷新心跳。
// 本检测在 20kHz 控制中断内调用（1ms = 20 个控制周期），收到首帧后开始生效；
// 连续 COMM_TIMEOUT_TICKS 个控制周期心跳无增长即判定失联，立即停机
// （防上位机死机/线缆断开/总线故障）。
// 阈值取 20ms ≈ 丢失 20 帧（20 倍于正常帧间隔），既灵敏又抗偶发丢帧/中断抖动。
#define COMM_TIMEOUT_TICKS   400    // 失联判定阈值（20kHz 下 = 20ms）

// 过温保护阈值（NTC 测线圈温度）
// 超过关断阈值停驱动；温度回落到恢复阈值以下才允许恢复（20℃ 滞回，防震荡）。
// 阈值按电机绕组绝缘等级调整
#define OVERTEMP_SHUTDOWN      90.0f   // 过温关断阈值 (℃)
#define OVERTEMP_RECOVER        70.0f   // 恢复阈值 (℃)，滞回 20℃
#define OVERTEMP_RECHECK_DELAY  2000    // 过温恢复重检周期 (ms)

// 母线过压/欠压保护阈值（26V 系统）
// 过压：关节电机回馈泵升；欠压：栅极驱动不足可能直通，均需立即停机。
#define BUS_TRIP_CONFIRM_TICKS  10  // 连续 10 拍（0.5ms @20kHz）越限才认定
#define BUS_OVERVOLTAGE_THRESHOLD   32.0f   // 过压停机阈值 (V)
#define BUS_OVERVOLTAGE_RECOVER     26.0f   // 过压恢复阈值 (V)，滞回 1.5V
#define BUS_UNDERVOLTAGE_THRESHOLD  16.0f   // 欠压停机阈值 (V)，DRV8353 最低 9V，留足余量
#define BUS_UNDERVOLTAGE_RECOVER    20.0f   // 欠压恢复阈值 (V)，滞回 2V
#define BUS_VOLTAGE_RECHECK_DELAY   2000    // 母线电压恢复重检周期 (ms)

// 超速保护阈值（KTH7823_Speed_Rads，电机轴机械角速度 rad/s）
// 关节电机防飞车，正反转均检测。
// 电机轴额定 = 减速器输出 146 RPM × 减速比 12.5 = 1825 RPM ≈ 191.1 rad/s。
// 超速停机取额定 × 1.2 ≈ 230 rad/s；恢复取额定 × 1.05 ≈ 200 rad/s（滞回 30 rad/s）。
// 电气跟踪校验：191.1 rad/s × 21 极对 = 4013 rad/s(电) × 50µs = 0.20 rad/拍 ≈ 11.5°，
// 远小于 45° 安全上限，电气跟踪无压力。
#define OVERSPEED_THRESHOLD   230.0f   // 超速停机阈值 (rad/s)，≈1.2×电机轴额定
#define OVERSPEED_RECOVER     200.0f   // 恢复阈值 (rad/s)，≈1.05×额定，滞回 30

// 超速"连续 N 拍确认"：抑制编码器解包裹跳变导致的单拍速度尖峰误触发。
// 依据：速度低通 α=SPEED_FILTER_ALPHA(0.15)、刷新 10kHz；最坏单次跳变(Δ=π → 原始 31416 rad/s)
// 经滤波后约 4712 rad/s，按 ×0.85/样衰减，需 ≈19 个速度样点(≈1.9ms) 才回落到 230 以下；
// Protect 在 20kHz(50µs/拍) 巡检，故约 38 拍读数仍超阈 → 取 50 拍(≈2.5ms) 留余量即可滤掉。
// 真实飞车会持续超阈，2.5ms 的确认延时相对机械时间常数可忽略。
#define OVERSPEED_CONFIRM_TICKS  50u   // 连续超阈拍数（20kHz 下 ≈2.5ms）

// 关闭三路 PWM 输出但保持 TIM1 计数（供所有保护分支/故障回调统一调用）
void Protect_StopPWM(void);

// 检测堵转电流
void Protect_StallCheck(void);

// 监测编码器读数有效性（连续无效读数），失效则触发硬件保护
void Protect_EncoderStallCheck(void);

// CAN 每收到一帧数据时调用（在 HAL_FDCAN_RxFifo0Callback 中断内），刷新通讯心跳
void Protect_CommFeed(void);

// 关闭失联保护（进入本地调试态），串口 enable 命令调用；CAN 再次发帧后自动恢复
void Protect_CommDisarm(void);

// 通讯心跳计数：每收到一帧本节点 CAN 命令帧 +1（接收中断内自增），串口 can 诊断命令读取
extern volatile uint32_t comm_rx_count;

// 当前失联保护是否已武装（1 = 收到过 CAN 帧、失联检测已生效），串口 can 诊断命令显示
uint8_t Protect_CommIsArmed(void);

// 上位机通讯连续中断超时
void Protect_CommTimeoutCheck(void);

// MOSFET/线圈过温
void Protect_OvertempCheck(void);

// 母线过压/欠压
void Protect_BusVoltageCheck(void);

// 机械超速
void Protect_OverspeedCheck(void);

// 单相瞬时峰值过流
void Protect_PhasePeakCheck(void);

// 累计 I²t 热积累并检测持续过流
void Protect_I2tCheck(void);

// I²t 热积累量（FOC.c 恢复分支据此判断是否仍过热）
extern volatile float i2t_accum;

// 软限位阈值（减速器输出端位置，rad，默认 ±π = ±180° 输出端）
extern float soft_limit_min;
extern float soft_limit_max;
// 位置跟随误差阈值（rad）
extern float pos_error_limit;

// 位置跟随误差
void Protect_PositionErrorCheck(void);

// 软限位持续被钳制
void Protect_SoftLimitCheck(void);

// 保护巡检
void Protect_FOC(void);
#endif
