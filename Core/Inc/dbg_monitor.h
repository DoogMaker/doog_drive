#ifndef __DBG_MONITOR_H
#define __DBG_MONITOR_H

#include <stdint.h>

/* =====================================================================
 * 临时调试监控结构体
 * 用途：把分散在各模块的关键运行变量集中快照到一个 volatile 全局结构体，
 *       方便 KEIL 调试时在 Watch 窗口添加 "dbg" 一次展开查看全部状态，
 *       且 volatile + 全局实例不会被编译器优化掉，断点/单步时值稳定可见。
 *
 * 用法：
 *   1. 进入调试，打开 Watch 1，添加变量名 "dbg"，展开即可看到全部字段。
 *   2. 调试结束、确认问题后，可整体删除本结构体、dbg 实例及更新调用。
 * ===================================================================== */
typedef struct {
    /* ---- 运行状态 ---- */
    uint8_t  mode;              // FOC 状态机：0=IDLE 1=ALIGN 2=OPENLOOP 3=CLOSEDLOOP 4=FAULT
    uint8_t  closedloop_mode;   // 闭环级联深度：0=力矩 1=速度 2=位置
    uint8_t  fault_code;        // 当前故障码 (FAULT_CODE_*，上报后清零)
    uint8_t  last_fault_code;   // 最近一次故障码（锁存，不清零，便于事后定位）
    uint8_t  fault_triggered;   // 故障触发标志
    uint8_t  fault_retry_cnt;   // 连续故障次数（指数退避）
    uint16_t fault_status1;     // DRV8353 硬件故障 STATUS1
    uint16_t fault_status2;     // DRV8353 硬件故障 STATUS2
    uint32_t tick;              // HAL_GetTick() 当前毫秒

    /* ---- 电流 (A) ---- */
    float Ia, Ib, Ic;           // 三相电流
    float Id, Iq;               // d/q 轴电流
    float i_mag;                // 三相合成电流幅值
    float i2t_accum;            // I²t 热积累

    /* ---- 电压 (V) ---- */
    float bus_voltage;          // 母线电压
    float Vd, Vq;               // d/q 轴电压指令
    float Vd_norm, Vq_norm;     // 归一化电压指令

    /* ---- 速度 / 位置 ---- */
    float speed;                // 当前转速 (rad/s)
    float target_speed;         // 命令目标转速 (rad/s)
    float target_speed_ramp;    // 斜坡后目标转速 (rad/s)
    float angle;                // 电角度 (rad)
    float kth_angle;            // 电机轴编码器角度 (rad)
    float kth_speed;            // 电机轴编码器速度 (rad/s)
    float enc2_angle;           // 输出端多圈角度 (rad)
    float position_error;       // 位置误差 (rad)
    float target_position;      // 目标位置 (rad)
    float target_Iq;            // 目标 Iq (A)

    /* ---- 温度 (℃) ---- */
    float ntc_temperature;      // 线圈温度

    /* ---- SVPWM 占空比 [0~1] ---- */
    float Ta, Tb, Tc;

    /* ---- PWM 输出指示（bit 掩码，全 1 = 0x1F 表示三路 PWM 正常输出）---- */
    uint8_t pwm_out;   // bit0=CEN计数, bit1=MOE主输出, bit2=CC1E, bit3=CC2E, bit4=CC3E

    /* ---- 关键引脚电平 ---- */
    uint8_t pin_en;     // PA11 EN（DRV8353 使能脚）：1=高(使能) 0=低(睡眠)
    uint8_t pin_nfault; // PA12 nFAULT（故障脚，低有效）：1=高(无故障) 0=低(有故障)

    /* ---- 计数 ---- */
    uint32_t encoder_update_count;  // 编码器刷新心跳
    uint32_t control_tick;          // 控制节拍计数
} DBG_Monitor_t;

extern volatile DBG_Monitor_t dbg;   // 监控快照实例（定义于 FOC.c）

void DBG_Monitor_Update(void);       // 刷新快照（主循环调用）

#endif
