#include "Protect.h"
#include "DRV8353.H"
#include "KTH7823.h"
#include "encoder2.h"
#include "sense.h"
#include "tim.h"
#include "foc.h"
#include "arm_math.h"

// 过流保护
extern volatile uint8_t fault_triggered;
extern volatile uint8_t fault_code;   // 故障诊断码锁存
extern volatile float KTH7823_Speed_Rads;   // 机械角速度 (rad/s)，超速/堵转检测用
extern float control_dt;   // 控制周期 (s)，I²t 累积用

//编码器掉线
static uint32_t encoder_last_count = 0; // 编码器更新基准，上一次的值
static uint32_t encoder_stall_ticks = 0; // 连续未变化的中断周期数
volatile uint16_t encoder_stall_max = 0; // 记录主循环到底最长卡过多久
static uint8_t  enc_armed = 0; // 监测是否准备标志位
static uint32_t enc_arm_tick = 0; // 记录监测起始点

// 上位机通讯心跳（CAN）：每收到一帧 Protect_CommFeed 递增一次
volatile uint32_t comm_rx_count = 0;    // 通讯心跳计数
static uint32_t comm_last_count = 0;    // 上次检测时的心跳基准
static uint32_t comm_silent_ticks = 0;  // 通讯静默节拍（连续未收到数据的周期数）
static uint8_t  comm_armed = 0;         // 收到首帧后置 1，失联保护才生效（未接上位机时不误触发）

// I²t 热积累量（全局，供 FOC.c 恢复分支判断是否仍过热）
volatile float i2t_accum = 0.0f;

// 软限位/位置误差阈值（减速器输出端位置，rad，此处定义默认值，MotorControl_Init 可覆盖）
float soft_limit_min  = -3.14159f;  // 默认 -π（约 -180°）
float soft_limit_max  =  3.14159f;  // 默认 +π
float pos_error_limit =  0.05f;     // 默认位置跟随误差 0.05 rad（约 2.9°，按需调整）

// 检测三相合成电流幅值，超阈值即停驱动并交状态机处理
// 无本地锁存：每拍独立判定，重试后仍过流会自然再次触发，避免竞态
void Protect_StallCheck(void)
{
    // 三相合成电流幅值（比单看一相更稳，能反映真实堵转/过流）
    float i_mag = sqrtf(current_A * current_A +
                         current_B * current_B +
                         current_C * current_C);

    // 堵转判定：大电流 且 转速接近 0（正常高速大电流运行不误触发）
    if (i_mag > STALL_CURRENT_LIMIT &&
        fabsf(KTH7823_Speed_Rads) < STALL_SPEED_THRESHOLD) {
        fault_code = FAULT_CODE_STALL;   // 锁存软件故障码（先于 Disable，供主循环上报）
        // 触发硬件保护：关闭栅极驱动 + 停三路 PWM
        DRV8353_Disable();
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        fault_triggered = 1;   // 交由 FOC_Update 顶部的 FOC_STATE_FAULT 状态机走指数退避自动恢复
    }
}

// 编码器失速/断线监测
// 原理：KTH7823_UpdateAngle 在主循环以 ~1kHz 刷新 encoder_update_count。
// 本函数在控制中断（20kHz，50µs/拍）内调用，若连续 ENCODER_STALL_TIMEOUT_TICKS个
// 中断周期计数都未增长，说明主循环未刷新编码器（SPI 卡死/断线/主循环阻塞），
// 则触发硬件保护，交由 FOC_Update 顶部的 FOC_STATE_FAULT 状态机走退避恢复。
void Protect_EncoderStallCheck(void)
{
    // ---- 准备阶段：等主循环至少刷新过一次，再观察 ARM_DELAY 后才开始监测 ----
    if (!enc_armed) {
        encoder_last_count  = encoder_update_count;
        encoder_stall_ticks = 0;
        if (encoder_update_count != 0) {                 // 主循环已跑起来
            if (enc_arm_tick == 0) enc_arm_tick = HAL_GetTick();
            else if (HAL_GetTick() - enc_arm_tick > ENCODER_ARM_DELAY_MS) {
                enc_armed = 1;
                encoder_last_count = encoder_update_count;
            }
        } else {
            enc_arm_tick = 0;                            // 还没跑起来，计时作废
        }
        return;
    }

    if (encoder_update_count != encoder_last_count) {        // 编码器正常刷新：更新基准并清零失速计数
        encoder_last_count = encoder_update_count;
        encoder_stall_ticks = 0;
    } else {        // 计数未增长：累计失速节拍
        encoder_stall_ticks++;
        if (encoder_stall_ticks > encoder_stall_max) encoder_stall_max = encoder_stall_ticks;
        if (encoder_stall_ticks >= ENCODER_STALL_TIMEOUT_TICKS) {         // 持续未刷新（默认 10ms @20kHz），判定编码器失效
            fault_code = FAULT_CODE_ENCODER_STALL;  // 锁存软件故障码
            DRV8353_Disable();
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            fault_triggered = 1;
            enc_armed = 0;      // 触发后重新准备
            enc_arm_tick = 0;
        }
    }
}

// 上位机通讯心跳喂狗：CAN 每收到一帧数据调用一次（在接收中断内）
void Protect_CommFeed(void)
{
    comm_rx_count++;
    comm_armed = 1;   // 收到首帧后使能失联保护
}

// 上位机通讯失联保护：连续 COMM_TIMEOUT_TICKS 个控制周期无新帧则停机
// 原理与编码器失速检测一致：比较心跳计数，无增长则累计静默节拍，超时即保护。
void Protect_CommTimeoutCheck(void)
{
    if (!comm_armed) return;   // 尚未收到任何上位机帧：独立运行，不启用失联保护

    if (comm_rx_count != comm_last_count) {
        // 收到新帧：更新基准并清零静默节拍
        comm_last_count = comm_rx_count;
        comm_silent_ticks = 0;
    } else {
        // 无新帧：累计静默节拍（饱和计数，防 uint32 回绕）
        if (comm_silent_ticks < COMM_TIMEOUT_TICKS) comm_silent_ticks++;
        if (comm_silent_ticks >= COMM_TIMEOUT_TICKS) {
            fault_code = FAULT_CODE_COMM_TIMEOUT;   // 锁存通讯失联故障码
            DRV8353_Disable();
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            fault_triggered = 1;   // 交由 FOC_STATE_FAULT 状态机退避恢复；重新收到帧后自动解除
        }
    }
}
// 关闭失联保护（进入本地调试态）：串口 enable 命令调用
// 清零 armed 使失联检测失效，并复位静默节拍/心跳基准，保持状态干净。
// 之后若 CAN 再次发帧，Protect_CommFeed 会重新置位 armed，失联保护自动恢复。
void Protect_CommDisarm(void)
{
    comm_armed = 0;
    comm_silent_ticks = 0;
    comm_last_count = comm_rx_count;
}

// MOSSFET/线圈过温保护
// 温度超过关断阈值即停驱动并交状态机退避；
// 恢复条件在 FOC_Update 的 FOC_STATE_FAULT 分支内：温度未降到恢复阈值则继续冷却（滞回）
void Protect_OvertempCheck(void)
{
    if (ntc_temperature > OVERTEMP_SHUTDOWN || mos_temperature > OVERTEMP_SHUTDOWN) {
        fault_code = FAULT_CODE_OVERTEMP;   // 锁存过温故障码（先于 Disable，供主循环上报）
        DRV8353_Disable();
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        fault_triggered = 1;   // 交由 FOC_Update 顶部的 FOC_STATE_FAULT 状态机退避恢复
    }
}

// 母线过压/欠压保护
// 过压：回馈能量泵升母线，立即停机（能量最终需硬件刹车电阻泄放）；
// 欠压：栅极驱动电源不足可能导致 MOSFET 直通，立即停机。
// 恢复条件在 FOC_Update 的 FOC_STATE_FAULT 分支内：电压回到安全范围才恢复（滞回）。
void Protect_BusVoltageCheck(void)
{
    static uint16_t ov_cnt = 0, uv_cnt = 0;
    
    ov_cnt = (bus_voltage > BUS_OVERVOLTAGE_THRESHOLD) ? (ov_cnt + 1) : 0;
    uv_cnt = (bus_voltage < BUS_UNDERVOLTAGE_THRESHOLD) ? (uv_cnt + 1) : 0;

    if (ov_cnt >= BUS_TRIP_CONFIRM_TICKS) {
        ov_cnt = 0;
        fault_code = FAULT_CODE_OVERVOLT;   // 锁存过压故障码
        DRV8353_Disable();
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        fault_triggered = 1;
    } else if (uv_cnt >= BUS_TRIP_CONFIRM_TICKS){
        uv_cnt = 0;
        fault_code = FAULT_CODE_UNDERVOLT;  // 锁存欠压故障码
        DRV8353_Disable();
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        fault_triggered = 1;
    }
}

// 位置跟随误差保护：仅闭环位置级联调用。误差持续超限 POS_ERR_TIMEOUT_TICKS 控制周期则停驱动
// 注意：此函数由 FOC_Update 的 FOC_STATE_CLOSEDLOOP 位置级联分支内调用，不在其他模式触发。
void Protect_PositionErrorCheck(void)
{
    if (foc.mode != FOC_STATE_CLOSEDLOOP || foc.closedloop_mode < CL_MODE_POSITION) {
        foc.pos_err_integral_acc = 0.0f;   // 未启用位置级联时清零累计
        return;
    }

    float err = fabsf(foc.position_error);
    if (err > pos_error_limit) {
        foc.pos_err_integral_acc++;
        if (foc.pos_err_integral_acc >= POS_ERR_TIMEOUT_TICKS) {
            fault_code = FAULT_CODE_POS_ERROR;   // 锁存位置跟随误差故障码
            DRV8353_Disable();
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            fault_triggered = 1;
        }
    } else {
        foc.pos_err_integral_acc = 0.0f;   // 误差回落到安全范围，清理累计
    }
}

// 软限位保护：FOC_SetTargetPosition 已做钳制，但若目标持续被钳制（=外部不断设置超出限位的目标），
// 持续 SOFT_LIMIT_TIMEOUT_TICKS 控制周期则停驱动，防止上层控制逻辑失控反复撞限位。
void Protect_SoftLimitCheck(void)
{
    if (foc.mode != FOC_STATE_CLOSEDLOOP || foc.closedloop_mode < CL_MODE_POSITION) {
        foc.pos_err_integral_acc = 0.0f;
        return;
    }

    // 检测：当前目标位置等于限位边界（说明被钳制了）
    uint8_t clipped = (foc.target_position >= soft_limit_max - 1e-4f) ||
                      (foc.target_position <= soft_limit_min + 1e-4f);
    if (clipped) {
        foc.pos_err_integral_acc++;
        if (foc.pos_err_integral_acc >= SOFT_LIMIT_TIMEOUT_TICKS) {
            fault_code = FAULT_CODE_SOFT_LIMIT;   // 锁存软限位故障码
            DRV8353_Disable();
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
            fault_triggered = 1;
        }
    } else {
        foc.pos_err_integral_acc = 0.0f;
    }
}

// 超速保护
// 机械角速度绝对值超阈值即停机（正反转均检测）。恢复条件在 FOC_STATE_FAULT 分支内：
// 速度回落到恢复阈值以下才允许恢复（滞回，防震荡）。
void Protect_OverspeedCheck(void)
{
    if (fabsf(KTH7823_Speed_Rads) > OVERSPEED_THRESHOLD) {
        fault_code = FAULT_CODE_OVERSPEED;   // 锁存超速故障码
        DRV8353_Disable();
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        fault_triggered = 1;
    }
}

// 单相瞬时峰值过流：检测单相电流绝对值，捕获不对称尖峰/短路，响应最快
void Protect_PhasePeakCheck(void)
{
    float i_peak = fabsf(current_A);
    float t = fabsf(current_B);
    if (t > i_peak) i_peak = t;
    t = fabsf(current_C);
    if (t > i_peak) i_peak = t;

    if (i_peak > PHASE_PEAK_CURRENT_LIMIT) {
        fault_code = FAULT_CODE_PHASE_OC;   // 锁存单相峰值过流故障码
        DRV8353_Disable();
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        fault_triggered = 1;
    }
}

// I²t 热积累保护：累计 (I_rms² - I_nom²)·dt，超阈值停驱动
// 区分"短时峰值"（允许）与"持续过流"（保护）；恢复需热积累回落到复位阈值以下。
void Protect_I2tCheck(void)
{
    float i_mag2 = current_A * current_A +
                   current_B * current_B +
                   current_C * current_C;
    float i_rms2 = i_mag2 / 3.0f;   // 单相等效 RMS 电流平方（i_mag = √3·I_rms）
    float nom2   = I2T_NOMINAL_CURRENT * I2T_NOMINAL_CURRENT;

    // 净热积累：发热 - 散热（负值表示冷却，下限 0）
    i2t_accum += (i_rms2 - nom2) * control_dt;
    if (i2t_accum < 0.0f) i2t_accum = 0.0f;

    if (i2t_accum > I2T_TRIP_THRESHOLD) {
        fault_code = FAULT_CODE_I2T;   // 锁存 I²t 过流故障码
        DRV8353_Disable();
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        fault_triggered = 1;
    }
}

// 保护巡检
void Protect_FOC(void)
{
	// 软堵转/过流保护：三相合成电流超阈值则关驱动、停 PWM，并置 fault 标志
	Protect_StallCheck();
	// 编码器失速/断线监测：主循环未刷新编码器则触发保护
	Protect_EncoderStallCheck();
	// 上位机通讯失联监测：CAN 心跳超时则触发保护
	Protect_CommTimeoutCheck();
	// 过温保护：线圈温度超关断阈值则触发保护
	Protect_OvertempCheck();
	// 母线过压/欠压保护：回馈泵升或栅极欠压均需停机
	Protect_BusVoltageCheck();
	// 超速保护：机械角速度超阈值停机（防飞车）
	Protect_OverspeedCheck();
	// 单相瞬时峰值过流：捕获不对称尖峰/短路
	Protect_PhasePeakCheck();
	// I²t 热积累保护：防持续过流烧电机
	Protect_I2tCheck();
}
	