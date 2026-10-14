#ifndef FOC_H
#define FOC_H

#include "pid.h"
#include "drv8353.h"      // DRV8353_Recover / DRV8353_Disable 等

// 运行状态 LED 宏
// PB9 低电平点亮（共阳接法）：Run_ON=常亮，Run_OFF=熄灭，Run_Toggle=翻转
#define Run_ON()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define Run_OFF()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define Run_Toggle() HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9)

// 故障 LED 快速闪烁参数：半周期(ms)，亮/灭各占一半 → 约 5Hz 闪烁
#define LED_FAULT_BLINK_HALF_MS  100

#define FOC_CLOSEDLOOP_ENABLE 0		// 0 = 禁用闭环（硬件验证阶段使用）  1 = 启用闭环（功能调试阶段打开）
#define MOTOR_POLE_PAIRS  21  	// 电机极对数

// 故障退避冷却时间（FOC_STATE_FAULT 状态机使用，指数退避：1次→2s,2次→4s...封顶）
#define FAULT_RETRY_DELAY     2000    // 基础冷却时间(ms)，每次重试按 2^(n-1) 增长
#define FAULT_RETRY_DELAY_MAX 16000   // 退避冷却时间上限(ms)，防止等待过久

// 故障诊断码（主循环上报用）：0=无故障，其余为已触发并锁存的故障类型
#define FAULT_CODE_NONE          0
#define FAULT_CODE_HARDWARE      1   // DRV8353 nFAULT 硬件故障（具体位见 fault_status1/2）
#define FAULT_CODE_STALL         2   // 软件堵转/过流（三相合成电流超限）
#define FAULT_CODE_ENCODER_STALL 3   // 编码器失速/断线
#define FAULT_CODE_OVERTEMP      4   // 线圈过温关断
#define FAULT_CODE_OVERVOLT      5   // 母线过压（回馈泵升）
#define FAULT_CODE_UNDERVOLT     6   // 母线欠压
#define FAULT_CODE_OVERSPEED     7   // 机械超速
#define FAULT_CODE_PHASE_OC      8   // 单相瞬时峰值过流
#define FAULT_CODE_I2T           9   // I²t 热积累过流（持续过流）
#define FAULT_CODE_POS_ERROR     10  // 位置跟随误差超限（位置级联）
#define FAULT_CODE_SOFT_LIMIT    11  // 软限位（目标位置持续被钳制在限位边界）
#define FAULT_CODE_COMM_TIMEOUT  12  // 上位机通讯失联（CAN 心跳超时）

// 统一缓启动斜坡参数（开环/闭环共用，防止上电速度/电压突变冲击）
#define FOC_SPEED_RAMP_RATE  80.0f   // 速度加速度上限 (rad/s per second)
#define FOC_VOLT_RAMP_RATE   5.8f    // 开环电压爬升速率 (V per second)

// 开环电压：改为 V/f + 低速升压，与速度斜坡同步，避免欠励磁窗口
#define FOC_VF_GAIN     0.0725f   // V/(rad/s)，≈ 额定 V/f
#define FOC_VOLT_BOOST  0.2f     // 低速升压 (V)，克服 R 压降/静摩擦

// ==================== 开环"转到指定输出端角度 + 到位悬停保持"参数 ====================
// CAN 0x07：按当前开环方式转到指定减速器输出端角度，到位后进入"电弹簧"悬停保持。
// 悬停**不计时自动释放**，会一直保持，直到满足以下任一释放条件（释放后电压、速度归零）：
//   ① 上位机发送 0x02 且电压为 0（v=0）——显式释放；
//   ② 离开开环模式（0x03 切模式、保护停机）——自动中止并释放。
// 完整说明见 docs/CAN上位机控制指令说明.md。
#define GEAR_RATIO_OUT2MOTOR        12.5f   // 减速器输出端 -> 电机轴 的减速比
#define FOC_OL_DEFAULT_SPEED        5.0f    // 未设转速时的默认开环转速 (rad/s，电机轴)
#define FOC_OL_DEFAULT_VOLTAGE      0.6f    // 未设电压时的默认开环电压 (V)
#define FOC_OL_MOVE_TIMEOUT_MS      5000    // 运动阶段超时 (ms)：未转够则直接断电释放
#define FOC_OL_HOLD_VOLTAGE_MAX     0.6f    // 悬停/回拉电压上限 (V)：d 轴持续通电，越大保持力矩越强、发热越大
#define FOC_OL_HOLD_BAND_DEG        10.0f   // 偏差死区 (±deg，电机轴)：超出即触发编码器回拉
#define FOC_OL_HOLD_HYST_DEG        3.0f    // 回拉滞环 (±deg，电机轴)：回到该范围内即重新冻结保持
#define FOC_OL_HOLD_PULL_SPEED      3.0f    // 回拉速度 (rad/s，电机轴)：低速，避免越过到位点
#define FOC_OL_HOLD_PULL_TIMEOUT_MS 1000    // 单次回拉最长耗时 (ms)
#define FOC_OL_DEG2RAD              0.01745329252f   // 度 -> rad（不依赖 PI 宏，避免头文件依赖差异）
#define FOC_OL_RAD2DEG              57.29577951308f  // rad -> 度
#define FOC_OL_TWO_PI               6.28318530718f   // 2π

// FOC状态机
typedef enum {
    FOC_STATE_IDLE,
    FOC_STATE_ALIGN,
    FOC_STATE_OPENLOOP,
    FOC_STATE_CLOSEDLOOP,    // 闭环（内部按 closedloop_mode 分力矩/速度/位置级联）
    FOC_STATE_FAULT          // 故障退避：停输出、倒计时冷却，到点后由 FOC_Update 自动恢复
} FOC_State_t;

// 闭环控制模式（级联深度）：三层环始终级联存在，值越大级联越深。
//   位置环(外) → 速度环(中) → 电流环/力矩环(内)
// 本枚举决定"目标值从哪一层注入"，不是三选一的割裂模式：
typedef enum {
    CL_MODE_TORQUE   = 0,  // 力矩环（最内层注入）：目标 Iq 直接给定，位置/速度环旁路
    CL_MODE_SPEED    = 1,  // 速度环（中层注入）：目标速度直接给定，速度环→电流环
    CL_MODE_POSITION = 2,  // 位置环（最外层注入）：位置环→速度环→电流环 全链路级联
} FOC_ClosedLoopMode_t;

// 开环"运动/悬停保持"子状态（FOC_State.openloop_move_state，也直接作为 0x300 状态帧 byte0）
typedef enum {
    FOC_OL_MOVE_IDLE = 0,   // 非活动/已释放
    FOC_OL_MOVE_RUN,        // 转动中（按开环命令速率定步长积分判到位）
    FOC_OL_MOVE_HOLD,       // 已到位悬停保持（冻结电角度 + d 轴通电形成电弹簧）
    FOC_OL_MOVE_PULL        // 回拉中（偏差超出死区，低速反向回拉至滞环内）
} FOC_OpenLoopMoveState_t;

// 力矩模式最大 Iq 限幅（A）：取电机额定 14A
//   原 45A 会算出 i_mag = √1.5×45 = 55A，既超 STALL_CURRENT_LIMIT(45A)，又正好顶到
//   单相峰值阈值 55A，低速/堵转时必触发保护。限到 14A 后：
//   i_mag = 17.1A < 45A（不触发堵转）、I_rms = 9.9A < 14A（不累积 I²t），可连续运行。
#define TORQUE_MAX_IQ   14.0f


// FOC 状态
typedef struct {
    float Ia, Ib, Ic;     // 三相电流 (A)
    float I_alpha, I_beta; // Clark 变换结果
    float Id, Iq;         // Park 变换结果 (A)
    float Vd, Vq;         // 电压指令 (V)
    float V_alpha, V_beta; // 逆 Park 变换结果
    float Ta, Tb, Tc;     // SVPWM 占空比 [0-1]
    float angle;          // 电角度 (rad)
    float speed;          // 转速 (rad/s)
    float target_speed;        // 命令目标转速 (rad/s) —— 由外部设置
    float target_speed_ramp;   // 斜坡后的实际目标转速 (rad/s) —— FOC_Update 内部收敛
    float speed_ramp_rate;     // 加速度限幅 (rad/s per second)
    float openloop_voltage_ramp; // 开环电压斜坡值 (V) —— FOC_Update 内部收敛
    float target_Iq;      // 目标 Iq (A)
    float target_torque;      // 目标转矩 (Nm)，力矩级联使用
    float torque_constant;    // 电机转矩常数 Kt (Nm/A)，需按实测设置
	float Vd_norm, Vq_norm;				 // 归一化电压指令
	float openloop_angle;            // 开环模式下独立累加的电角度 (rad)
    float openloop_voltage;          // 开环模式下手动给定的电压幅值 (V)
    uint8_t openloop_hold;           // 1 = 开环保持（电弹簧）：冻结 openloop_angle，改用 Vd 通电保持力矩，
                                     //     与 ALIGN 磁力定位同形态；0 = 正常开环（角度按 target_speed_ramp 积分）
                                     //     所有 FOC_SetMode 调用都会清零本标志（含 CAN/串口/启动序列）
	  FOC_State_t mode;		// 当前工作状态
    FOC_ClosedLoopMode_t closedloop_mode;   // 闭环级联深度（力矩/速度/位置）
    // 开环"转到指定输出端角度 + 悬停保持"子状态机（由 FOC_Update 20kHz 中断非阻塞推进）
    uint8_t openloop_move_state;      // FOC_OpenLoopMoveState_t：0=IDLE 1=RUN 2=HOLD 3=PULL，直接作为状态帧 byte0
    uint8_t openloop_move_stop_req;   // 1 = 请求中止"运动/悬停"动作（主循环置位，20kHz 中断原子消费并释放）
    float   openloop_move_remain;     // RUN 阶段剩余电角度 (rad，非负)，递减到 0 判到位；其余状态为 0
    float   openloop_move_ref_mech;   // 本次动作起点：电机轴多圈机械角 (rad)，供 CAN 层换算输出端相对角
    float   openloop_move_hold_mech;  // 到位悬停点：电机轴多圈机械角 (rad)，供 CAN 层换算悬停偏差
    // 位置环字段（减速器输出端位置，单位 rad，Encoder2_Angle_Multiturn 范围）
    float target_position;       // 目标输出端位置 (rad)
    float position_error;        // 实时位置误差 = target_position - current_position（仅诊断/保护读）
    float pos_err_integral_acc;  // 位置误差持续超限的周期累计（保护用）
    // 故障退避（FOC_STATE_FAULT 期间使用，非阻塞倒计时）
    uint8_t  fault_retry_cnt;   // 连续故障次数（指数退避）
    uint32_t fault_recover_time;// 计划恢复时间戳(ms)，由 HAL_GetTick() 比较
    uint32_t last_recover_tick; // 最近一次故障恢复时刻(ms)，稳定运行超时后据此重置 fault_retry_cnt
} FOC_State;

// 对齐定位
typedef enum {
    ALIGN_STEP_0 = 0,
    ALIGN_STEP_90,
    ALIGN_STEP_DONE
} AlignStep_t;


extern volatile FOC_State foc;
extern float encoder_offset;      // 编码器零点偏移（机械角度）
extern int8_t encoder_direction;  // 编码器方向：1 正转增加，-1 正转减少

// 对齐结果上报：对齐逻辑在控制中断（20kHz）里只存数值 + 置标志，
// sprintf/串口发送由主循环的 UART_ReportAlignResult() 完成（避免中断里浮点格式化）。
extern volatile uint8_t align_report_pending;    // 1 = 有新的对齐结果待上报
extern volatile float   align_report_offset;     // 本次得到的零点偏移 (rad)
extern volatile float   align_report_delta;      // 本次测得的角度变化 (rad)
extern volatile int8_t  align_report_direction;  // 本次得到的方向 (+1/-1)
extern volatile uint32_t align_report_samples;   // 本次窗口平均用到的样点数（诊断用）

// PID 控制器句柄（定义于 FOC.c，供 UART 在线调参等外部模块访问）
extern PID_Handle_t pid_speed;     // 速度环 PID
extern PID_Handle_t pid_id;        // d 轴电流环 PID
extern PID_Handle_t pid_iq;        // q 轴电流环 PID
extern PID_Handle_t pid_position;  // 位置环 PID

// 故障诊断锁存变量（定义于 main.c）
extern volatile uint8_t  fault_code;      // 当前故障码，主循环上报后清零
extern volatile uint16_t fault_status1;   // 硬件故障 STATUS1 锁存（Disable 前读取）
extern volatile uint16_t fault_status2;   // 硬件故障 STATUS2 锁存

// 软限位/位置误差阈值（定义于 Protect.c）
extern float soft_limit_min;
extern float soft_limit_max;
extern float pos_error_limit;

void FOC_Init(void);
void FOC_Update(void);
void LED_Update(void);   // LED 状态指示（运行常亮，待机熄灭，故障闪烁，主循环调用）

void FOC_Clark(void);
void FOC_Park(void);
void FOC_InversePark(void);
void FOC_SVPWM(void);

void FOC_SetMode(FOC_State_t mode);
void FOC_SetClosedLoopMode(FOC_ClosedLoopMode_t mode);  // 设置闭环级联深度（力矩/速度/位置）
void FOC_SetOpenLoopVoltage(float voltage);
void FOC_StartOpenLoopMoveHold(float out_rad);    // 开环转到输出端角度 out_rad(rad，0 忽略)并悬停保持
void FOC_StopOpenLoopMoveHold(void);              // 中止"转角度/悬停"动作并释放（电压、速度归零）
void FOC_SetTargetPosition(float position_rad);   // 设置目标位置（位置级联），内部已含软限位钳制
void FOC_SetSoftLimits(float min_rad, float max_rad); // 设置软限位阈值（需 min < max）
void FOC_SetPositionErrorLimit(float error_rad);  // 设置位置跟随误差阈值
void FOC_SetTorqueConstant(float kt_Nm_per_A);    // 设置转矩常数 Kt (Nm/A)，力矩级联使用
void FOC_SetTargetTorque(float torque_Nm);        // 设置目标转矩 (Nm)，内部已限幅到 ±TORQUE_MAX_IQ×Kt

#endif



