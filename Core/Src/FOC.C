#include "foc.h"
#include "sense.h"
#include "KTH7823.h"
#include "encoder2.h"
#include "arm_math.h"
#include "Protect.h"
#include "dbg_monitor.h"


#define ALIGN_VOLTAGE       0.6f    // 对齐电压 (V)，用于将转子拉到目标位置
#define ALIGN_STEP_DELAY    800     // 每步对齐持续时间 (ms)
#define ALIGN_AVG_MS        200     // 每步末尾用于"平均定位"的时间窗口 (ms)
                                    // 必须 < ALIGN_STEP_DELAY，且远大于转子残余振荡周期

// 全局变量
volatile FOC_State foc = {0};    // FOC 状态结构体（跨中断/主循环共享）
PID_Handle_t pid_speed;     // 速度环 PID 控制器
PID_Handle_t pid_id;    // d轴电流环 PID 控制器
PID_Handle_t pid_iq;    //q轴电流环 PID 控制器
PID_Handle_t pid_position;    // 位置环 PID 控制器（最外环：位置→速度）

float encoder_offset = 0.0f;    // 编码器零点偏移（机械角度）
float mechanical_angle;		//转子机械角度
int8_t encoder_direction = 1;  // 编码器方向：1=正转增加，-1=正转减少

static AlignStep_t align_step = ALIGN_STEP_0;   // 对齐步骤状态
static uint32_t align_step_time = 0;    // 对齐步骤计时器
static float start_angle_raw, end_angle_raw;    // 对齐起始/结束角度
static float delta_angle = 0;		//变化角度

// 对齐结果上报：控制中断（20kHz，50µs/拍）里只存数值 + 置标志，
// sprintf/串口发送交给主循环，避免在中断里做浮点格式化。
volatile uint8_t align_report_pending   = 0;    // 1 = 有新的对齐结果待上报
volatile float   align_report_offset    = 0.0f; // 本次得到的零点偏移 (rad)
volatile float   align_report_delta     = 0.0f; // 本次测得的角度变化 (rad)
volatile int8_t  align_report_direction = 1;    // 本次得到的方向 (+1/-1)
volatile uint32_t align_report_samples  = 0;    // 本次窗口平均用到的样点数（诊断用）

static uint32_t led_blink_tick = 0;    // LED 闪烁计时基准(ms)，进入 FAULT 时重置
extern float control_dt;       // 控制周期 (秒)，由 motor_control.c 提供
extern volatile float bus_voltage;			//母线电压采样
extern volatile float KTH7823_Angle_Rad;
extern float KTH7823_Angle_Multiturn;    // 原始累积多圈角度（未滤波，闭环角度源用）
extern volatile float KTH7823_Speed_Rads;
extern volatile uint8_t fault_triggered;	
extern volatile uint32_t control_tick;   // 控制节拍计数（motor_control.c 定义）

// 开环"转到指定输出端角度 + 到位悬停保持"状态机私有量（仅 20kHz 控制中断读写）
static float    ol_hold_voltage = 0.0f;   // 本次动作的悬停/回拉电压 (V)，启动时已按上限限幅
static float    ol_theta_hold   = 0.0f;   // 到位悬停点对应的电角度 (rad)，回拉后用它复位以防逐次漂移
static uint32_t ol_move_t0      = 0;      // 运动阶段起始时刻 (ms)，用于超时判定
static uint32_t ol_pull_t0      = 0;      // 本次回拉起始时刻 (ms)
static float    ol_pull_speed   = 0.0f;   // 本次回拉的低速给定 (rad/s，带符号)，回拉期间每拍重申

/* ---------------- 对齐：末尾窗口平均定位（替代单点采样） ----------------
 * 单点采样对"转子残余振荡"和"编码器噪声"都极其敏感：励磁电压越高转子冲得越猛，
 * 800ms 采样那一刻越可能撞上振荡相位，表现为 |delta| 忽大忽小（0.8V 实测就是如此）。
 * 这里改为：在每步末尾的一个时间窗口内持续累积角度的 sin/cos，最后取 atan2 平均。
 *   - 振荡周期远小于窗口长度 -> 直接平均掉；
 *   - 上千个样点平均 -> 噪声降低 √N 倍（等效分辨率提升）；
 *   - 累积 sin/cos 而不是角度本身 -> 天然不受 0~2π 环绕影响。
 */
static float    align_sum_sin     = 0.0f;
static float    align_sum_cos     = 0.0f;
static uint32_t align_sum_cnt     = 0;
static uint32_t align_last_samples = 0;   // 最近一次平均用到的样点数（诊断用）

// 清空窗口累积（进入对齐 / 每步切换时调用）
static void align_accum_reset(void)
{
    align_sum_sin = 0.0f;
    align_sum_cos = 0.0f;
    align_sum_cnt = 0;
}

// 每个控制周期调用：仅在"步进末尾窗口"内累积样点。
// 前半段转子还在运动，把那段也累计进来会把结果往运动路径上拉偏。
static void align_accumulate(void)
{
    if (align_step_time == 0) return;                                        // 计时未启动
    if ((HAL_GetTick() - align_step_time) < (ALIGN_STEP_DELAY - ALIGN_AVG_MS)) return;

    align_sum_sin += sinf(KTH7823_Angle_Rad);
    align_sum_cos += cosf(KTH7823_Angle_Rad);
    align_sum_cnt++;
}

// 取窗口内的平均角度 [0, 2π)；窗口内无有效样点时回退为瞬时值
static float align_average_angle(void)
{
    align_last_samples = align_sum_cnt;

    if (align_sum_cnt == 0u) {
        return KTH7823_Angle_Rad;                                            // 兜底
    }
    float a = atan2f(align_sum_sin, align_sum_cos);
    if (a < 0.0f) a += 2.0f * PI;
    return a;
}

// 速度环 PID 参数
// 单位：target/feedback = rad/s(机械)；输出 = 目标 Iq(A)
// 输出限幅取 ±14A（电机额定）：
//   - i_mag = √1.5 × 14 = 17.1A < STALL_CURRENT_LIMIT(45A) → 低速大扭矩不会误触发堵转保护
//   - I_rms = 14/√2 = 9.9A < I2T_NOMINAL_CURRENT(14A) → 永不累积 I²t，可连续运行
//   （原 ±40A 会算出 i_mag = 49A > 45A：只要 |speed| < 10rad/s 就立刻跳 STALL）
// 积分限幅 = out_max/Ki，使 I 项单独就能顶到输出上限（否则积分饱和点被限死，动态变差）
// Kp/Ki 需按机械惯量实调：先 Ki=0 加 Kp 到轻微振荡再回退一半，然后加 Ki 消稳态误差
PID_Params_t speed_params = {
	.Kp = 0.10f,   .Ki = 0.50f,   .Kd = 0.0f,
	.out_min = -14.0f, .out_max = 14.0f,   // 输出限幅：±14A（额定）→ 目标 Iq
	.int_min = -28.0f, .int_max = 28.0f    // 积分限幅 = out_max/Ki = 14/0.5
};

//d轴电流环 PID 参数 (Id 目标通常为 0，弱磁时可为负)
// 单位：target/feedback = A；输出 = Vd(V)
// 设计依据（被控对象 L=29.85µH、R=72.5mΩ 相值，控制周期 50µs）：
//   Kp = ω_c·L，Ki = ω_c·R，ω_c = 2π·f_bw
//   Kp=0.15 → f_bw ≈ 800Hz（远低于 20kHz 采样率，相位裕度充裕）
//   Ki=450 → Ti = Kp/Ki = 0.33ms ≈ τ = L/R = 0.41ms（PI 零点抵消对象极点）
//   int_max ≈ out_max/Ki = 12/450 = 0.027，使 I 项单独就能顶满输出
//   Kd 保持 0：电流采样 1LSB ≈ 0.040A，微分会放大噪声
PID_Params_t id_params = {
	.Kp = 0.15f,    .Ki = 450.0f,    .Kd = 0.0f,
	.out_min = -12.0f, .out_max = 12.0f, // 输出电压限幅
	.int_min = -0.03f, .int_max = 0.03f
};

// q轴电流环 PID 参数
// 单位：target/feedback = A；输出 = Vq(V)
// 与 d 轴同源（表贴式 PMSM 的 Ld≈Lq），设计依据与 id_params 完全一致
PID_Params_t iq_params = {
	.Kp = 0.15f,    .Ki = 450.0f,    .Kd = 0.0f,
	.out_min = -12.0f, .out_max = 12.0f,
	.int_min = -0.03f, .int_max = 0.03f
};

// 位置环 PID 参数（最外环：位置 → 速度）
// 单位：target/feedback = 输出端位置 (rad)；输出 = 目标速度 (rad/s，电机轴)
// 输出限幅 = 最大目标速度（与超速阈值一致），Kp/Ki/D 需按机械惯量调参
PID_Params_t position_params = {
	.Kp = 5.0f,    .Ki = 0.5f,    .Kd = 0.0f,
	.out_min = -230.0f, .out_max = 230.0f,
	.int_min = -230.0f, .int_max = 230.0f
};

//FOC算法初始化
void FOC_Init(void)
{
	// 初始化四个 PID 控制器（速度/电流×2/位置）
	PID_Init(&pid_speed, &speed_params);
	PID_Init(&pid_id, &id_params);
	PID_Init(&pid_iq, &iq_params);
	PID_Init(&pid_position, &position_params);
	
	//设置 FOC 初始状态（空闲，全部归零）
	foc.mode = FOC_STATE_IDLE;    // 默认空闲模式：上电无输出（安全），由启动序列/上位机指令切换
	foc.closedloop_mode = CL_MODE_SPEED;   // 闭环默认速度子模式
	foc.target_speed = 0.0f;
	foc.target_torque = 0.0f;
	foc.torque_constant = 0.1f;   // Kt 占位值，必须按电机实测设置（Nm/A）；暂未知时可设 1.0 让转矩值等于 Iq(A)
	foc.target_speed_ramp = 0.0f;     // 斜坡实际值从 0 起步
	foc.speed_ramp_rate = FOC_SPEED_RAMP_RATE;
	foc.openloop_voltage = 0.0f;    // 初始电压为 0，电机不转
	foc.target_position = 0.0f;
	foc.position_error = 0.0f;
	foc.pos_err_integral_acc = 0.0f;
	foc.openloop_voltage_ramp = 0.0f;
	foc.target_Iq = 0.0f;
	foc.openloop_angle = 0.0f;
	foc.openloop_hold = 0;          // 开环保持（电弹簧）默认关闭
	foc.openloop_move_state     = FOC_OL_MOVE_IDLE;   // 开环"转角度+悬停"子状态机默认空闲
	foc.openloop_move_stop_req  = 0;
	foc.openloop_move_remain    = 0.0f;
	foc.openloop_move_ref_mech  = 0.0f;
	foc.openloop_move_hold_mech = 0.0f;
	ol_hold_voltage = 0.0f;
	ol_theta_hold   = 0.0f;
	ol_move_t0      = 0;
	ol_pull_t0      = 0;
	ol_pull_speed   = 0.0f;
	foc.Vd = 0.0f;
	foc.Vq = 0.0f;
	
	//重置所有 PID 积分项
	PID_Reset(&pid_speed);
	PID_Reset(&pid_id);
	PID_Reset(&pid_iq);
}

//Clark变换：三相电流 → 两相静止坐标系
void FOC_Clark(void)
{
	foc.I_alpha = foc.Ia;
	foc.I_beta = (foc.Ib - foc.Ic) / 1.73205f;
}

//Park 变换：静止坐标系 → 旋转坐标系
void FOC_Park(void)
{
	float cos_theta = cosf(foc.angle);
	float sin_theta = sinf(foc.angle);

	foc.Id = foc.I_alpha * cos_theta + foc.I_beta * sin_theta;
	foc.Iq = -foc.I_alpha * sin_theta + foc.I_beta * cos_theta;
}

//逆 Park 变换：旋转坐标系 → 静止坐标系
void FOC_InversePark(void)
{
	float cos_theta = cosf(foc.angle);
	float sin_theta = sinf(foc.angle);

	foc.V_alpha = foc.Vd_norm * cos_theta - foc.Vq_norm * sin_theta;
	foc.V_beta = foc.Vd_norm * sin_theta + foc.Vq_norm * cos_theta;
}

//七段式 SVPWM 算法
void FOC_SVPWM(void)
{
	float T = 1.0f;  //PWM 周期归一化为 1

	// 计算三个扇区边界值
	float V1 = foc.V_beta;
	float V2 = -0.86605f * foc.V_alpha - 0.5f * foc.V_beta;
	float V3 = 0.86605f * foc.V_alpha - 0.5f * foc.V_beta;

	int sector = 0;
	if (V1 > 0) sector |= 1;
	if (V2 > 0) sector |= 2;
	if (V3 > 0) sector |= 4;

	// 计算相邻矢量的作用时间
	int S;
	switch (sector) {
        case 5: S = 1; break;
        case 1: S = 2; break;
        case 3: S = 3; break;
        case 2: S = 4; break;
        case 6: S = 5; break;
        case 4: S = 6; break;
        default: S = 0; break;
    }

	float T1, T2;
	switch (S) {
		case 1: T1 = V3; T2 = V1; break;
		case 2: T1 = -V2; T2 = -V3; break;
		case 3: T1 = V1; T2 = V2; break;
		case 4: T1 = -V3; T2 = -V1; break;
		case 5: T1 = V2; T2 = V3; break;
		case 6: T1 = -V1; T2 = -V2; break;
		default: T1 = 0; T2 = 0; break;
	}

	// 防止浮点误差导致微小负值
    if (T1 < 0) T1 = 0;
    if (T2 < 0) T2 = 0;

	// 限幅（确保 T1+T2 不超过 T）
	float T_sum = T1 + T2;
	if (T_sum > T) {
		T1 = T1 / T_sum * T;
		T2 = T2 / T_sum * T;
	}

	float T0 = (T - T1 - T2) / 2.0f;

	// 计算三相占空比
	float Ta, Tb, Tc;
	switch (S) {
		case 1: Ta = T0 + T1 + T2; Tb = T0 + T2; Tc = T0; break;
		case 2: Ta = T0 + T1; Tb = T0 + T1 + T2; Tc = T0; break;
		case 3: Ta = T0; Tb = T0 + T1 + T2; Tc = T0 + T2; break;
		case 4: Ta = T0; Tb = T0 + T1; Tc = T0 + T1 + T2; break;
		case 5: Ta = T0 + T2; Tb = T0; Tc = T0 + T1 + T2; break;
		case 6: Ta = T0 + T1 + T2; Tb = T0; Tc = T0 + T1; break;
		default: Ta = Tb = Tc = 0.5; break;
	}

	foc.Ta = Ta;
	foc.Tb = Tb;
	foc.Tc = Tc;
}

/* ==================== 开环"转到指定输出端角度 + 到位悬停保持"（电弹簧）====================
 * 设计要点（完整说明见 docs/CAN上位机控制指令说明.md）：
 *   - RUN  ：开环无位置反馈，靠对"命令电角度速率 target_speed_ramp"按控制节拍定步长积分判到位
 *            （与同一中断内推进 openloop_angle 同源）；输出端机械角 -> 电机轴(×减速比) -> 电角度(×极对数)。
 *   - HOLD ：到位后冻结电角度、改用 d 轴持续通电（与 ALIGN 磁力定位同形态）形成"电弹簧"，
 *            转子被磁力钳在到位点附近，偏离即自动产生回复力矩；每拍重申给定（幂等自愈）。
 *   - PULL ：以电机轴编码器(KTH7823_Angle_Multiturn，未滤波)监测偏差，超出 ±FOC_OL_HOLD_BAND_DEG
 *            时解除冻结、按 FOC_OL_HOLD_PULL_SPEED 低速回拉，回到 ±FOC_OL_HOLD_HYST_DEG 内重新冻结。
 *   - 悬停**不计时释放**，一直保持；仅当①收到 v=0（0x02 电压为 0）或②离开开环模式时释放
 *     （释放后电压、速度归零）。因此悬停期间 d 轴持续通电发热，请按需及时释放。
 * 全部在 20kHz 控制中断内非阻塞推进：每拍 O(1)，无循环/等待/动态内存，ISR 安全。
 */

// 电角度归一化到 [0, 2π)（与 FOC_Update 开环分支一致）
static float FOC_OL_NormElecAngle(float a)
{
	a = fmodf(a, FOC_OL_TWO_PI);
	if (a < 0.0f) a += FOC_OL_TWO_PI;
	return a;
}

// 当前转子电角度（与 FOC_SetMode 进入开环时的表达式/角度源完全一致）
static float FOC_OL_RotorElecAngle(void)
{
	return FOC_OL_NormElecAngle((KTH7823_Angle_Multiturn - encoder_offset) *
	                            (float)encoder_direction * (float)MOTOR_POLE_PAIRS);
}

// 释放：清状态与保持标志，电压/速度归零（不动 manual_speed/manual_voltage，保持既有使用顺序）
static void FOC_OL_Release(void)
{
	foc.openloop_move_state  = FOC_OL_MOVE_IDLE;
	foc.openloop_move_remain = 0.0f;
	foc.openloop_hold        = 0;
	foc.target_speed         = 0.0f;
	foc.target_speed_ramp    = 0.0f;
	foc.openloop_voltage     = 0.0f;
	ol_pull_speed            = 0.0f;
}

// 20kHz 控制中断内推进的状态机（非阻塞）
static void FOC_OpenLoopMoveHold_Service(void)
{
	// 离开开环模式（切模式/保护停机）或收到外部中止请求（如 v=0 释放）时中止悬停并释放输出。
	// 外部请求由主循环置位、这里在中断内统一消费完成，避免主循环与中断并发写同一状态相互覆盖。
	// （保护停机是直接改 foc.mode，未经 FOC_SetMode，故这里必须自查模式。）
	uint8_t stop_req = foc.openloop_move_stop_req;
	if (stop_req) foc.openloop_move_stop_req = 0;

	if (stop_req || foc.mode != FOC_STATE_OPENLOOP) {
		if (foc.openloop_move_state != FOC_OL_MOVE_IDLE) {
			// 回拉阶段曾临时改写目标速度，退出时一并清零，避免残留速度在后续进入开环时意外转动
			if (foc.openloop_move_state == FOC_OL_MOVE_PULL) {
				foc.target_speed      = 0.0f;
				foc.target_speed_ramp = 0.0f;
			}
			if (stop_req && foc.mode == FOC_STATE_OPENLOOP) {
				FOC_OL_Release();     // v=0 显式释放：动作 + 电压/速度全归零
			} else {
				foc.openloop_move_state  = FOC_OL_MOVE_IDLE;   // 仅清动作，避免影响其它模式的给定
				foc.openloop_move_remain = 0.0f;
				foc.openloop_hold        = 0;
			}
		}
		return;
	}

	const float band = FOC_OL_HOLD_BAND_DEG * FOC_OL_DEG2RAD;   // 死区 (±rad，电机轴)
	const float hyst = FOC_OL_HOLD_HYST_DEG * FOC_OL_DEG2RAD;   // 回拉滞环 (±rad，电机轴)
	float dev;

	switch ((FOC_OpenLoopMoveState_t)foc.openloop_move_state) {
		case FOC_OL_MOVE_RUN:
			// 定步长积分判到位：remain 从目标电角度递减到 0
			foc.openloop_move_remain -= fabsf(foc.target_speed_ramp) * (float)MOTOR_POLE_PAIRS * control_dt;
			if (foc.openloop_move_remain <= 0.0f) {
				// 到位：清速度给定 + 冻结电角度 + d 轴通电，进入电弹簧悬停
				foc.target_speed            = 0.0f;
				foc.target_speed_ramp       = 0.0f;
				foc.openloop_voltage        = ol_hold_voltage;
				ol_theta_hold               = FOC_OL_RotorElecAngle();
				foc.openloop_angle          = ol_theta_hold;
				foc.openloop_move_hold_mech = KTH7823_Angle_Multiturn;   // 记录悬停点(电机轴机械角)
				foc.openloop_hold           = 1;
				foc.openloop_move_remain    = 0.0f;
				foc.openloop_move_state     = FOC_OL_MOVE_HOLD;   // 无限期保持，由 v=0 / 切模式释放
			} else if ((HAL_GetTick() - ol_move_t0) >= FOC_OL_MOVE_TIMEOUT_MS) {
				// 运动超时（未转够）：不进入悬停，直接断电释放
				FOC_OL_Release();
			}
			break;

		case FOC_OL_MOVE_HOLD:
			// 幂等重申：冻结电角度 + d 轴通电 + 速度给定归零（自愈，保证不被常规刷新打断）
			foc.openloop_angle   = ol_theta_hold;
			foc.openloop_hold    = 1;
			foc.openloop_voltage = ol_hold_voltage;
			foc.target_speed      = 0.0f;
			foc.target_speed_ramp = 0.0f;
			// 偏差：正 = 转子朝"正转(+开环方向)"偏出到位点
			dev = (KTH7823_Angle_Multiturn - foc.openloop_move_hold_mech) * (float)encoder_direction;

			if (fabsf(dev) > band) {
				// 逃出死区：解除冻结，平衡点对齐当前位置（避免跨极反向弹走），低速反向回拉
				foc.openloop_hold    = 0;
				foc.openloop_angle   = FOC_OL_RotorElecAngle();
				ol_pull_speed        = (dev > 0.0f) ? -FOC_OL_HOLD_PULL_SPEED : FOC_OL_HOLD_PULL_SPEED;
				foc.target_speed     = ol_pull_speed;
				foc.openloop_voltage = ol_hold_voltage;
				ol_pull_t0           = HAL_GetTick();
				foc.openloop_move_state = FOC_OL_MOVE_PULL;
			}
			// 注意：悬停不按时间自动释放，会一直保持到 v=0（0x02 电压为 0）或离开开环模式
			break;

		case FOC_OL_MOVE_PULL:
			// 回拉中：幂等重申低速给定与回拉电压（防被 0x01 等常规刷新打断）
			foc.openloop_hold    = 0;
			foc.openloop_voltage = ol_hold_voltage;
			foc.target_speed     = ol_pull_speed;
			dev = (KTH7823_Angle_Multiturn - foc.openloop_move_hold_mech) * (float)encoder_direction;

			if (fabsf(dev) <= hyst ||
			           ((dev > 0.0f) != (ol_pull_speed < 0.0f)) ||
			           ((HAL_GetTick() - ol_pull_t0) >= FOC_OL_HOLD_PULL_TIMEOUT_MS)) {
				// 回到滞环内 / 已越过到位点(偏差变号) / 单次回拉超时：重新冻结保持
				foc.target_speed      = 0.0f;
				foc.target_speed_ramp = 0.0f;
				foc.openloop_voltage  = ol_hold_voltage;
				// 悬停点优先回归原始位置（防逐次回拉累积漂移）；偏差超出弹簧吸引域
				// (±180 电角度 ≈ ±8.6° 电机轴机械角)时退化为对齐当前位置，否则回复力矩会反向推开转子
				if (fabsf(dev) * (float)MOTOR_POLE_PAIRS < FOC_OL_TWO_PI * 0.5f) {
					foc.openloop_angle = ol_theta_hold;
				} else {
					foc.openloop_angle = FOC_OL_RotorElecAngle();
				}
				foc.openloop_hold       = 1;
				foc.openloop_move_state = FOC_OL_MOVE_HOLD;   // 重新进入无限期悬停保持
			}
			break;

		case FOC_OL_MOVE_IDLE:
		default:
			break;
	}
}

// 启动一次"开环转动 + 到位悬停保持"。out_rad 为减速器输出端目标角度(rad)，0 忽略。
// 说明：内部会调用 FOC_SetMode(FOC_STATE_OPENLOOP) 保证状态一致；
//       速度/电压优先复用当前给定(foc.target_speed/foc.openloop_voltage)，未设定则用缺省值。
void FOC_StartOpenLoopMoveHold(float out_rad)
{
	if (fabsf(out_rad) < 1e-5f) return;   // 0° 视为无效，直接忽略（不驱动、不改状态）

	float spd = fabsf(foc.target_speed);
	if (spd < 0.1f) spd = FOC_OL_DEFAULT_SPEED;      // 速度未设定则用缺省
	float vol = foc.openloop_voltage;
	if (vol < 0.05f) vol = FOC_OL_DEFAULT_VOLTAGE;   // 电压未设定则用缺省
	float hold_vol = (vol > FOC_OL_HOLD_VOLTAGE_MAX) ? FOC_OL_HOLD_VOLTAGE_MAX : vol;

	float dir = (out_rad >= 0.0f) ? 1.0f : -1.0f;

	// 输出端机械角 -> 电机轴机械角 -> 电角度 (rad)
	float target_elec = fabsf(out_rad) * GEAR_RATIO_OUT2MOTOR * (float)MOTOR_POLE_PAIRS;

	// ---- 启动交接顺序：先置 IDLE，再写全部参数，最后置 RUN（主循环写 / 中断读，避免半成品被执行）----
	foc.openloop_move_state = FOC_OL_MOVE_IDLE;

	// 进入开环并给定速度/电压（FOC_SetMode 会清零 openloop_hold 与状态机，故参数在其后写）
	FOC_SetMode(FOC_STATE_OPENLOOP);
	foc.target_speed      = dir * spd;
	foc.target_speed_ramp = 0.0f;                 // 从零起步，交给统一斜坡块缓启动
	foc.openloop_hold     = 0;                    // 运动阶段为正常开环（角度按 target_speed_ramp 积分）
	FOC_SetOpenLoopVoltage(vol);

	foc.openloop_move_remain    = target_elec;
	foc.openloop_move_ref_mech  = KTH7823_Angle_Multiturn;   // 本次动作起点（供 CAN 层算输出端相对角）
	foc.openloop_move_hold_mech = KTH7823_Angle_Multiturn;   // 悬停点先占位，到位时再刷新

	ol_hold_voltage = hold_vol;
	ol_move_t0      = HAL_GetTick();

	foc.openloop_move_stop_req = 0;                 // 清除可能残留的中止请求（FOC_SetMode 会置位）
	foc.openloop_move_state    = FOC_OL_MOVE_RUN;   // 最后一步：使能状态机
}

// 中止"开环转动 + 到位悬停保持"动作并释放（电压、速度归零）
void FOC_StopOpenLoopMoveHold(void)
{
	foc.openloop_move_stop_req = 1;
}

// FOC 核心更新函数
void FOC_Update(void)
{
	//读取反馈信号
	foc.speed = KTH7823_Speed_Rads;	    // 当前转速 (rad/s)
	foc.Ia = current_A; // 当前A相电流
	foc.Ib = current_B;
	foc.Ic = current_C;

	// ---- 故障拦截：任意保护触发即进入 FAULT 状态（非阻塞退避）----
	if (fault_triggered) {
		fault_triggered = 0;
		if (foc.mode != FOC_STATE_FAULT) {
			foc.mode = FOC_STATE_FAULT;
			foc.fault_retry_cnt++;
			// 指数退避：1次→2s, 2次→4s, 3次→8s ... 封顶 16s
			uint32_t cooldown = FAULT_RETRY_DELAY;
			for (uint8_t i = 1; i < foc.fault_retry_cnt; i++) {
				cooldown <<= 1;
				if (cooldown > FAULT_RETRY_DELAY_MAX) { cooldown = FAULT_RETRY_DELAY_MAX; break; }
			}
			foc.fault_recover_time = HAL_GetTick() + cooldown;
			// 立即停输出（PWM 已在 Protect 里被 Disable/Stop，这里双重保险）
			TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
			Run_OFF();                      // 进入故障先熄灭，作为闪烁起点
			led_blink_tick = HAL_GetTick(); // 重置闪烁计时基准
		}
	}

	// ---- 故障恢复后稳定运行超过最大退避时长，重置连续故障计数（避免 fault_retry_cnt 永久封顶 16s）----
	if (foc.mode != FOC_STATE_FAULT &&
	    foc.last_recover_tick != 0 &&
	    (HAL_GetTick() - foc.last_recover_tick) > FAULT_RETRY_DELAY_MAX) {
		foc.fault_retry_cnt = 0;
		foc.last_recover_tick = 0;
	}

	// ---- 开环"转到指定输出端角度 + 悬停保持"状态机（20kHz 非阻塞推进）----
	// 放在故障拦截之后、统一命令斜坡之前：状态机先决定状态并重申保持给定，
	// 再由统一斜坡块按（可能刚被更新的）openloop_hold/电压完成电压斜坡。
	FOC_OpenLoopMoveHold_Service();

	// ---- 统一命令斜坡（开环/闭环共用，防启动冲击）----
	// 速度斜坡：每拍只允许 target_speed 朝命令值靠近 rate*dt
	{
		float speed_step = foc.speed_ramp_rate * control_dt;
		float speed_err  = foc.target_speed - foc.target_speed_ramp;
		if (speed_err >  speed_step) foc.target_speed_ramp += speed_step;
		else if (speed_err < -speed_step) foc.target_speed_ramp -= speed_step;
		else foc.target_speed_ramp = foc.target_speed;   // 进入死区，直接对齐
	}
	// 开环电压斜坡：仅开环分支使用，闭环下恒等于给定值（无影响）
	{
    	float v_cmd = foc.openloop_voltage;                    // 用户设定的上限
		float v_lim = bus_voltage / 2.0f;
		if (v_cmd > v_lim) v_cmd = v_lim;
    	if (v_cmd < 0.0f)  v_cmd = 0.0f;
    	if (foc.openloop_hold) {
       		// 电弹簧保持：零速下 V/f 只给到 FOC_VOLT_BOOST(0.2V)，力矩不足，
       		// 故保持阶段直接用给定电压，保证"保持一定力矩"可调。
        	foc.openloop_voltage_ramp = v_cmd;
    	} else if (v_cmd > 0.0f) {
       		// 目标励磁 = boost + V/f×转速，但不超过用户设定
        	float v_vf = FOC_VOLT_BOOST + FOC_VF_GAIN * fabsf(foc.target_speed_ramp);
        	if (v_vf > v_cmd) v_vf = v_cmd;
        	foc.openloop_voltage_ramp = v_vf;                  // 直接给定，不再二次斜坡
    	} else {
        	foc.openloop_voltage_ramp = 0.0f;
    	}
	}

	// Clark 变换 
	FOC_Clark();
	
	#if (FOC_CLOSEDLOOP_ENABLE == 0)	
	if (foc.mode == FOC_STATE_CLOSEDLOOP) {
		FOC_SetMode(FOC_STATE_OPENLOOP);   // 经统一入口切回：重置 PID + 对齐开环角度/清零电压
	}
	#endif
	
	switch (foc.mode) {
		case FOC_STATE_IDLE:// 空闲模式：输出全部为零，PWM 占空比清零（真正停转，不悬空中点）
			foc.Vd = 0.0f;
			foc.Vq = 0.0f;
			foc.angle = KTH7823_Angle_Rad; // 仅用于监测
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			return;  // 跳过后续 SVPWM/占空比更新，保持 PWM 输出为 0
            
		case FOC_STATE_ALIGN:// 对齐模式：将转子拉到固定位置（0° 和 90°）
			// 每拍累积"末尾窗口"内的角度样点，供步进结束时取平均（替代单点采样）
			align_accumulate();

			switch (align_step) {
				case ALIGN_STEP_0:
					// 拉到 0° 电角度
					foc.Vd = ALIGN_VOLTAGE;  
					foc.Vq = 0.0f;
					foc.angle = 0.0f;
					if (align_step_time == 0) align_step_time = HAL_GetTick();
					if (HAL_GetTick() - align_step_time > ALIGN_STEP_DELAY) {
						start_angle_raw = align_average_angle();   // 窗口平均，非单点
						align_step = ALIGN_STEP_90;
						align_step_time = 0;
						align_accum_reset();
					}
					break;

				case ALIGN_STEP_90:
					// 拉到 90° 电角度（显式给励磁，不依赖上一步残留的 Vd，
					// 否则从 STEP_90 重入时会完全没有励磁，转子不动、结果随机）
					foc.Vd = ALIGN_VOLTAGE;
					foc.Vq = 0.0f;
					foc.angle = PI / 2.0f;
					if (align_step_time == 0) align_step_time = HAL_GetTick();
					if (HAL_GetTick() - align_step_time > ALIGN_STEP_DELAY) {
						end_angle_raw = align_average_angle();     // 窗口平均，非单点
						align_step = ALIGN_STEP_DONE;
						align_step_time = 0;
						align_accum_reset();
					}
					break;

				case ALIGN_STEP_DONE:	// 计算方向和偏移				
					delta_angle = end_angle_raw - start_angle_raw;
					if (delta_angle > PI) delta_angle -= 2*PI;
					if (delta_angle < -PI) delta_angle += 2*PI;
					if (delta_angle > 0) {
						encoder_direction = 1;   // 正转时编码器增加
					} else {
						encoder_direction = -1;  // 正转时编码器减少
					}
					// 保存偏移（以第一次位置为基准）
					encoder_offset = start_angle_raw;

					// 存快照供主循环上报（中断里不做 sprintf）
					align_report_offset    = encoder_offset;
					align_report_delta     = delta_angle;
					align_report_direction = encoder_direction;
					align_report_samples   = align_last_samples;   // 诊断：本次平均用了多少点
					align_report_pending   = 1;

					// 清零输出
					foc.Vd = 0.0f;
					foc.Vq = 0.0f;
					FOC_SetMode(FOC_STATE_OPENLOOP);
					align_step = ALIGN_STEP_0;
					align_step_time = 0;
				break;
			}
	    break;
    
		case FOC_STATE_OPENLOOP:// 开环模式
			if (foc.openloop_hold) {
				// ---- 电弹簧保持：冻结电角度 + d 轴通电（与 ALIGN 磁力定位同形态）----
				// 平衡点 = openloop_angle（到位时已对齐到转子电角度），偏离即产生回复力矩，
				// 稳定吸引域约 ±180 电角度 ≈ ±8.6° 电机轴机械角（极对数 21 → 电周期 17.14°）；
				// 更大的偏离由命令层用电机轴编码器检测后重新走开环回拉。
				foc.angle = foc.openloop_angle;
				foc.Vd = foc.openloop_voltage_ramp;
				foc.Vq = 0.0f;
				break;
			}
			// 电角度由斜坡后的目标速度积分累加
			foc.openloop_angle += foc.target_speed_ramp  * MOTOR_POLE_PAIRS * control_dt;
			// 角度归一化到 0~2π
			if (foc.openloop_angle > 2.0f * PI) foc.openloop_angle -= 2.0f * PI;
			if (foc.openloop_angle < 0.0f) foc.openloop_angle += 2.0f * PI;
			foc.angle = foc.openloop_angle;
            
			// 电压给定（Vd=0, Vq=斜坡后的手动电压）
			foc.Vq = foc.openloop_voltage_ramp;
			foc.Vd = 0.0f;
			break;
            
		case FOC_STATE_CLOSEDLOOP://闭环模式：位置环→速度环→电流环 三级联（按 closedloop_mode 决定注入深度）
			// ★ 角度源必须用"未滤波"的 KTH7823_Angle_Multiturn：
			//   KTH7823_Angle_Rad 是 sin/cos 一阶低通(α=0.1 @主循环~830Hz)的输出，
			//   等效群延迟约 12ms —— 10rad/s 时电角度滞后约 145°，闭环必然失控。
			//   Multiturn 是原始累积角度（连续无界，无需解环绕）；转子静止时两者相等，
			//   所以 mode=3 对齐得到的 encoder_offset 在本角度空间下同样有效。
			mechanical_angle = (KTH7823_Angle_Multiturn - encoder_offset) * encoder_direction;
			// 电角度 = 机械角度 × 极对数
			foc.angle = mechanical_angle * MOTOR_POLE_PAIRS;
			// 归一化到 [0, 2π)，避免大角度（多圈累积）造成 sinf/cosf 浮点精度损失
			foc.angle = fmodf(foc.angle, 2.0f * PI);
			if (foc.angle < 0.0f) foc.angle += 2.0f * PI;
			// Park变换（得到 Id, Iq）
			FOC_Park();

			// ===== 级联流水线：位置环(外) → 速度环(中) → 电流环(内)，三层数据单向流动 =====
			float speed_ref;

			// 第 1 层：位置环（最外层）。closedloop_mode >= POSITION 时，位置环输出作为速度环目标
			if (foc.closedloop_mode >= CL_MODE_POSITION) {
				foc.position_error = foc.target_position - Encoder2_Angle_Multiturn;
				speed_ref = PID_Calculate(&pid_position,
				                          foc.target_position,
				                          Encoder2_Angle_Multiturn,
				                          control_dt);
				// 位置相关保护（仅位置级联下检测）
				Protect_PositionErrorCheck();
				Protect_SoftLimitCheck();
			} else {
				// 速度/力矩模式：速度环目标直接来自外部给定（已斜坡）
				speed_ref = foc.target_speed_ramp;
			}

			// 第 2 层：速度环（中层）。closedloop_mode >= SPEED 时启用；力矩模式直接给定 Iq 旁路速度环
			if (foc.closedloop_mode >= CL_MODE_SPEED) {
				foc.target_Iq = PID_Calculate(&pid_speed, speed_ref, foc.speed, control_dt);
			} else {
				// 力矩模式：目标 Iq 由力矩直接换算（T = Kt × Iq）
				foc.target_Iq = foc.target_torque / foc.torque_constant;
			}

			// 第 3 层：电流环（最内层，始终运行，所有模式共用）
			foc.Vq = PID_Calculate(&pid_iq, foc.target_Iq, foc.Iq, control_dt);
			foc.Vd = PID_Calculate(&pid_id, 0.0f, foc.Id, control_dt);  // Id 目标 = 0
			break;
            
		case FOC_STATE_FAULT:// 故障退避：停输出，倒计时到点后自动恢复开环
			foc.Vd = 0.0f;
			foc.Vq = 0.0f;
			TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
			if (HAL_GetTick() >= foc.fault_recover_time) {
				// 恢复前需同时满足：温度正常 + 母线电压安全 + 转速已回落（滞回防震荡）
				if (ntc_temperature > OVERTEMP_RECOVER) {
					foc.fault_recover_time = HAL_GetTick() + OVERTEMP_RECHECK_DELAY;
				} else if (bus_voltage > BUS_OVERVOLTAGE_RECOVER ||
				           bus_voltage < BUS_UNDERVOLTAGE_RECOVER) {
					foc.fault_recover_time = HAL_GetTick() + BUS_VOLTAGE_RECHECK_DELAY;
				} else if (fabsf(KTH7823_Speed_Rads) > OVERSPEED_RECOVER) {
					foc.fault_recover_time = HAL_GetTick() + BUS_VOLTAGE_RECHECK_DELAY;
				} else if (i2t_accum > I2T_RESET_THRESHOLD) {
					foc.fault_recover_time = HAL_GetTick() + I2T_RECHECK_DELAY;
				} else {
					// 冷却结束且电压安全且转速正常：恢复驱动 + 切回开环（斜坡缓启动，不会瞬间满电流）
					DRV8353_Recover();
					FOC_SetMode(FOC_STATE_OPENLOOP);
					foc.last_recover_tick = HAL_GetTick();   // 记录恢复时刻，供稳定运行后重置计数
					Run_ON();
				}
			}
			return;  // FAULT 期间不执行 SVPWM/占空比更新

		default:
		// 未知状态：安全输出 0
		foc.Vd = 0.0f;
		foc.Vq = 0.0f;
		break;
	}
	
//  保护部分
	Protect_FOC();

	// 电压限幅与归一化
	float V_mag = sqrtf(foc.Vd * foc.Vd + foc.Vq * foc.Vq);
	float V_max = bus_voltage / 1.732f;  // Vdc/√3：SVPWM 线性调制区上限（六边形内切圆）
	if (V_mag > V_max) {
		foc.Vd = foc.Vd / V_mag * V_max;
		foc.Vq = foc.Vq / V_mag * V_max;	
	}

	// 归一化到 SVPWM 标幺值：满调制 r=1 对应 Vdc/√3，与上方限幅基准 V_max 一致
	foc.Vd_norm = foc.Vd / V_max;
	foc.Vq_norm = foc.Vq / V_max;

	// 逆 Park 变换
	FOC_InversePark();  

	// SVPWM 调制 
	FOC_SVPWM();

	// 更新 PWM 占空比
	// 3x PWM 模式下，使用 Ta 取决于 MOSFET 逻辑电平
	// 先对占空比做 [0,1] 限幅，防止浮点误差导致 CCR 超出 ARR
	float Ta = (foc.Ta > 1.0f) ? 1.0f : (foc.Ta < 0.0f) ? 0.0f : foc.Ta;
	float Tb = (foc.Tb > 1.0f) ? 1.0f : (foc.Tb < 0.0f) ? 0.0f : foc.Tb;
	float Tc = (foc.Tc > 1.0f) ? 1.0f : (foc.Tc < 0.0f) ? 0.0f : foc.Tc;
	uint32_t arr = TIM1->ARR;
	TIM1->CCR1 = (uint32_t)(Ta * arr);
	TIM1->CCR2 = (uint32_t)(Tb * arr);
	TIM1->CCR3 = (uint32_t)(Tc * arr);
}

// 切换 FOC 控制模式
void FOC_SetMode(FOC_State_t mode)
{
	// 任意模式切换（含串口 mode=/ang=、CAN、启动序列）都退出开环保持，
	// 避免电弹簧状态残留在后续模式里造成意外通电；放在提前返回前，保证同一模式的重复调用也生效。
	foc.openloop_hold = 0;
	// 同时请求中止"开环运动/悬停"状态机：只置中断消费的请求标志，实际释放由 20kHz 控制中断原子完成，
	// 避免主循环与中断并发写同一状态而相互覆盖（否则释放可能被中断的"重申"复活）。
	// 注意：绝不动 manual_speed/manual_voltage —— 否则会破坏"先发速度/电压、再切 mode=1"的既有使用顺序。
	foc.openloop_move_stop_req = 1;

	if (foc.mode == mode) return;

	// 重置所有 PID 积分，避免模式切换时输出突变
	PID_Reset(&pid_speed);
	PID_Reset(&pid_id);
	PID_Reset(&pid_iq);
	PID_Reset(&pid_position);   // 位置环 PID 也需重置

	if (mode == FOC_STATE_ALIGN) {
		// 进入对齐必须从第一步重新开始：
		// 否则中途被打断（再发一次 enable/disable、或触发 FAULT）后 align_step 会停在中间，
		// 下次 mode=3 从中间步骤续跑，start_angle_raw 是旧值 -> 结果错误且没有任何提示。
		align_step = ALIGN_STEP_0;
		align_step_time = 0;
		align_accum_reset();   // 清空窗口累积，避免上一次的残留样点混入
	}

	if (mode == FOC_STATE_OPENLOOP) {
		// 切换到开环时，将开环角度与当前编码器电角度对齐（含方向/零偏，保证闭环↔开环切换连续）
		// 必须与闭环分支用同一个角度源（未滤波的 Multiturn）：否则闭环→开环那一拍会因
		// 滤波滞后产生上百度的电角度跳变，造成电流冲击。
		foc.openloop_angle = (KTH7823_Angle_Multiturn - encoder_offset) * encoder_direction * MOTOR_POLE_PAIRS;
		// 电压归零，防止冲击
    	foc.openloop_voltage = 0.0f;
	}
	foc.mode = mode;
}

// 设置闭环控制模式（级联深度），仅在 FOC_STATE_CLOSEDLOOP 下有意义
void FOC_SetClosedLoopMode(FOC_ClosedLoopMode_t mode)
{
	if (foc.closedloop_mode == mode) return;
	foc.closedloop_mode = mode;
	// 级联深度切换时的平滑处理
	if (mode >= CL_MODE_POSITION) {
		// 进入位置级联：位置目标 = 当前位置（防阶跃冲击），重置位置误差累积
		foc.target_position = Encoder2_Angle_Multiturn;
		foc.position_error = 0.0f;
		foc.pos_err_integral_acc = 0.0f;
	}
	if (mode == CL_MODE_TORQUE) {
		// 进入力矩模式：清零目标 Iq/转矩，防止残留 Iq 造成瞬间冲击
		foc.target_Iq = 0.0f;
		foc.target_torque = 0.0f;
	}
	// 切换级联深度时重置各环积分，避免输出突变
	PID_Reset(&pid_speed);
	PID_Reset(&pid_position);
	PID_Reset(&pid_iq);
}

// 设置开环模式下的手动电压
void FOC_SetOpenLoopVoltage(float voltage)
{
	if (foc.mode == FOC_STATE_OPENLOOP) {
		// 限幅保护
		if (voltage > bus_voltage / 2.0f) voltage = bus_voltage / 2.0f;
		if (voltage < 0.0f) voltage = 0.0f;
		foc.openloop_voltage = voltage;
    }
}

// 设置软限位阈值（减速器输出端位置，rad），min < max
void FOC_SetSoftLimits(float min_rad, float max_rad)
{
	if (min_rad >= max_rad) return;   // 非法参数，直接拒绝
	soft_limit_min = min_rad;
	soft_limit_max = max_rad;
}

// 设置位置跟随误差阈值（rad），超过即累计，累积到 POS_ERR_TIMEOUT_TICKS 触发保护
void FOC_SetPositionErrorLimit(float error_rad)
{
	if (error_rad > 0.0f) pos_error_limit = error_rad;
}

// 设置目标位置（位置环模式）。内部已含软限位钳制，避免目标值超界导致位置 PID 持续饱和。
void FOC_SetTargetPosition(float position_rad)
{
	// 软限位钳制（保护逻辑内部还会检查持续超限的异常情况）
	if (position_rad > soft_limit_max) position_rad = soft_limit_max;
	if (position_rad < soft_limit_min) position_rad = soft_limit_min;
	foc.target_position = position_rad;
}

// 设置电机转矩常数 Kt (Nm/A)，力矩子模式使用。必须按实测设置，否则力矩值与实际不符。
// 若暂时未知 Kt，可设 1.0，此时目标转矩数值上等于目标 Iq (A)。
void FOC_SetTorqueConstant(float kt_Nm_per_A)
{
	if (kt_Nm_per_A > 0.0f) {
		foc.torque_constant = kt_Nm_per_A;
	}
}

// 设置目标转矩 (Nm)，内部限幅到 ±TORQUE_MAX_IQ × Kt，防止目标 Iq 超出峰值电流能力。
void FOC_SetTargetTorque(float torque_Nm)
{
	float max_torque = TORQUE_MAX_IQ * foc.torque_constant;
	if (torque_Nm >  max_torque) torque_Nm =  max_torque;
	if (torque_Nm < -max_torque) torque_Nm = -max_torque;
	foc.target_torque = torque_Nm;
}

// LED 状态指示（主循环调用）：运行常亮，待机(IDLE)熄灭，故障(FAULT)闪烁
void LED_Update(void)
{
	switch (foc.mode) {
		case FOC_STATE_IDLE:
			Run_OFF();   // 待机：熄灭
			break;

		case FOC_STATE_FAULT:
			// 故障：5Hz 闪烁（进入 FAULT 时已在 FOC_Update 里 Run_OFF 作为起始）
			if ((HAL_GetTick() - led_blink_tick) >= LED_FAULT_BLINK_HALF_MS) {
				led_blink_tick = HAL_GetTick();
				Run_Toggle();
			}
			break;

		default:   // ALIGN / OPENLOOP / CLOSEDLOOP：运行
			Run_ON();   // 运行：常亮
			break;
	}
}






// ==================== 调试监控快照（临时，可整体删除）====================
volatile DBG_Monitor_t dbg = {0};   // 监控实例（KEIL Watch 添加 "dbg" 展开查看）

// 集中刷新关键运行变量到 dbg（主循环调用，~1kHz）
void DBG_Monitor_Update(void)
{
	// ---- 运行状态 ----
	dbg.mode            = (uint8_t)foc.mode;
	dbg.closedloop_mode = (uint8_t)foc.closedloop_mode;
	dbg.fault_code      = fault_code;
	if (fault_code != FAULT_CODE_NONE) {
		dbg.last_fault_code = fault_code;   // 锁存最近一次故障码，上报清零后仍可查
	}
	dbg.fault_triggered = fault_triggered;
	dbg.fault_retry_cnt = foc.fault_retry_cnt;
	dbg.fault_status1   = fault_status1;
	dbg.fault_status2   = fault_status2;
	dbg.tick            = HAL_GetTick();

	// ---- 电流 ----
	dbg.Ia = foc.Ia;
	dbg.Ib = foc.Ib;
	dbg.Ic = foc.Ic;
	dbg.Id = foc.Id;
	dbg.Iq = foc.Iq;
	dbg.i_mag = sqrtf(foc.Ia * foc.Ia + foc.Ib * foc.Ib + foc.Ic * foc.Ic);
	dbg.i2t_accum = i2t_accum;

	// ---- 电压 ----
	dbg.bus_voltage = bus_voltage;
	dbg.Vd = foc.Vd;
	dbg.Vq = foc.Vq;
	dbg.Vd_norm = foc.Vd_norm;
	dbg.Vq_norm = foc.Vq_norm;

	// ---- 速度 / 位置 ----
	dbg.speed             = foc.speed;
	dbg.target_speed      = foc.target_speed;
	dbg.target_speed_ramp = foc.target_speed_ramp;
	dbg.angle             = foc.angle;
	dbg.kth_angle         = KTH7823_Angle_Rad;
	dbg.kth_speed         = KTH7823_Speed_Rads;
	dbg.enc2_angle        = Encoder2_Angle_Multiturn;
	dbg.position_error    = foc.position_error;
	dbg.target_position   = foc.target_position;
	dbg.target_Iq         = foc.target_Iq;

	// ---- 温度 ----
	dbg.ntc_temperature = ntc_temperature;

	// ---- SVPWM 占空比 ----
	dbg.Ta = foc.Ta;
	dbg.Tb = foc.Tb;
	dbg.Tc = foc.Tc;

	// ---- PWM 输出指示（读 TIM1 关键使能位，全 1 = 0x1F 表示三路正常输出）----
	dbg.pwm_out = 0;
	if (TIM1->CR1  & TIM_CR1_CEN)   dbg.pwm_out |= 0x01;  // 计数器在跑
	if (TIM1->BDTR & TIM_BDTR_MOE)  dbg.pwm_out |= 0x02;  // 主输出使能
	if (TIM1->CCER & TIM_CCER_CC1E) dbg.pwm_out |= 0x04;  // CH1 输出使能
	if (TIM1->CCER & TIM_CCER_CC2E) dbg.pwm_out |= 0x08;  // CH2 输出使能
	if (TIM1->CCER & TIM_CCER_CC3E) dbg.pwm_out |= 0x10;  // CH3 输出使能

	// ---- 关键引脚电平 ----
	dbg.pin_en     = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) ? 1 : 0;  // PA11 EN
	dbg.pin_nfault = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) ? 1 : 0;  // PA12 nFAULT

	// ---- 计数 ----
	dbg.encoder_update_count = encoder_update_count;
	dbg.control_tick         = control_tick;

	// ---- 编码器读数有效性（连续异常读数 0x0000/0xFFFF 判定断线）----
	dbg.kth_raw_fault_cnt = kth7823_raw_fault_cnt;
	dbg.kth_raw_fault     = (kth7823_raw_fault_cnt >= KTH7823_RAW_FAULT_LIMIT) ? 1 : 0;
}