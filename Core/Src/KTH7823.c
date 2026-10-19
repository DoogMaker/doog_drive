#include "kth7823.h"
#include "main.h"        // ME_CS_Pin / ME_CS_GPIO_Port
#include "arm_math.h"
#include "spi.h"
#include "dwt_delay.h"   // DWT_DelayMs


// 全局变量
volatile float KTH7823_Angle_Rad = 0.0f;	// 单圈角度 [0, 2π)
float KTH7823_Angle_Deg        = 0.0f;	// 当前角度（度）
float KTH7823_Angle_Multiturn  = 0.0f;	// 多圈总角度（连续无界，弧度）
float KTH7823_Speed_RPM        = 0.0f;	// 当前速度（RPM）
volatile float KTH7823_Speed_Rads = 0.0f;	// 当前速度（rad/s）

// 刷新心跳计数器：每次 KTH7823_UpdateAngle 调用 +1，供失速/断线监测判断编码器是否在刷新
volatile uint32_t encoder_update_count = 0;

// 连续异常读数（0x0000/0xFFFF）计数：正常读数清零，供 Protect_EncoderStallCheck 判定读数有效性
volatile uint16_t kth7823_raw_fault_cnt = 0;

// 被丢弃的解包裹野值累计次数（诊断用，只增不清零；正常应保持 0 或偶发）
volatile uint16_t kth7823_unwrap_glitch_cnt = 0;

// 滤波变量
static float filtered_cos = 1.0f;
static float filtered_sin = 0.0f;
static uint8_t filter_initialized = 0;
static uint16_t last_raw_angle = 0;
static uint8_t  raw_initialized = 0;
static float    angle_unwrapped = 0.0f;  // 连续单圈角度
static float    last_unwrapped_angle = 0.0f;  // 上一帧连续角度（供速度差分，Init 时复位）
static uint32_t last_speed_tick = 0;     // 上次 UpdateSpeed 的 DWT->CYCCNT 时间戳（周期数，Init 时复位）

// SPI 收发一帧 16bit（uint16_t 单帧 + Size=1，避免 16 位模式下字节序/越界问题）
static uint16_t spi_exchange_frame(uint16_t tx)
{
    // 初始化为0
    uint16_t txw = tx;   // 直接 16 位，硬件 MSB 先发，无字节序问题
    uint16_t rxw = 0;

    HAL_GPIO_WritePin(ME_CS_GPIO_Port, ME_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(KTH7823_SPI, (uint8_t *)&txw, (uint8_t *)&rxw, 1, 100);
    HAL_GPIO_WritePin(ME_CS_GPIO_Port, ME_CS_Pin, GPIO_PIN_SET);

    return rxw;   // HAL 16 位模式已写入完整 16 位
}

// 读取 KTH7823 原始角度值（16位）—— 单帧流水线读取
// 协议：当前帧发送命令，返回的是“上一命令帧的回读”。每拍都发同一个读命令（0x0000），
//       故本帧的返回值就是上一拍命令对应的角度，等价于旧版“发两帧取第二帧”，但线时减半
//       （1 帧 16bit @9MHz ≈ 1.78µs，旧版 2 帧 ≈ 3.56µs），仅滞后一帧。
// CS：每帧独立“拉低→传输→拉高”，两帧之间高电平远大于 50ns 的要求（HAL 调用开销已足够）。
// 注：流水线第一拍由 KTH7823_Init() 的那次调用建立，其返回值（上电旧值）被丢弃。
uint16_t KTH7823_ReadAngleRaw(void)
{
    return spi_exchange_frame(0x0000); // 发读命令并直接取回读（= 上一命令帧对应的角度）
}

// 角度弧度转换
static float raw_to_rad(uint16_t raw)
{
    return (float)raw / KTH7823_ANGLE_RESOLUTION * 2.0f * PI;  // 65536 个刻度覆盖 2π
}

// KTH7823 初始化
void KTH7823_Init(void)
{
    KTH7823_Angle_Rad = 0.0f;
    KTH7823_Angle_Deg = 0.0f;
    KTH7823_Angle_Multiturn = 0.0f;
    KTH7823_Speed_Rads = 0.0f;
    KTH7823_Speed_RPM = 0.0f;

    filtered_cos = 1.0f;
    filtered_sin = 0.0f;
    filter_initialized = 0;
    raw_initialized = 0;
    angle_unwrapped = 0.0f;
    last_unwrapped_angle = 0.0f;  // 复位速度差分基准，避免重新 Init 后首帧速度跳变
    last_speed_tick = 0;          // 复位速度计时基准，首帧 UpdateSpeed 不计算速度
    kth7823_raw_fault_cnt = 0;    // 复位异常读数计数
    kth7823_unwrap_glitch_cnt = 0; // 复位野值剔除计数
    encoder_update_count = 0;

    // 建立流水线首拍：单帧协议下这一帧既完成上电通信初始化，也让下一帧能取到有效回读。
    // 其返回值是上电旧值/无效值，直接丢弃（首次 UpdateAngle 取到的才是本条命令对应的角度）。
    KTH7823_ReadAngleRaw();
    DWT_DelayMs(10);
}

// 更新当前角度
void KTH7823_UpdateAngle(void)
{
    uint16_t raw = KTH7823_ReadAngleRaw();
		float raw_rad = raw_to_rad(raw);

    // 异常读数统计：0x0000/0xFFFF 视为断线/无磁等无效读数，连续累加；正常读数清零
    if (raw == 0x0000u || raw == 0xFFFFu) {
        if (kth7823_raw_fault_cnt < 0xFFFFu) kth7823_raw_fault_cnt++;
    } else {
        kth7823_raw_fault_cnt = 0;
    }
	
    // 多圈解包裹：用 16 位带符号差值累积
    if (!raw_initialized) {
        last_raw_angle = raw;
        angle_unwrapped = raw_rad;
        raw_initialized = 1;
    } else {
        int16_t delta_raw = (int16_t)(raw - last_raw_angle);  // 自动处理跨 0xFFFF 边界
        // 野值剔除：单次刷新内跳变超物理可能上限（见 KTH7823_UNWRAP_MAX_DELTA），判为坏读数/
        // 解包裹野值，丢弃本次——不更新 last_raw_angle、不累积角度，等下一次好读数再接续，
        // 从而避免多圈角度阶跃与测速尖峰（后者会误触发超速保护）。
        if (delta_raw > KTH7823_UNWRAP_MAX_DELTA || delta_raw < -KTH7823_UNWRAP_MAX_DELTA) {
            if (kth7823_unwrap_glitch_cnt < 0xFFFFu) kth7823_unwrap_glitch_cnt++;
        } else {
            last_raw_angle = raw;
            angle_unwrapped += (float)delta_raw / KTH7823_ANGLE_RESOLUTION * 2.0f * PI;
        }
    }

    // sin/cos 一阶低通滤波
    if (!filter_initialized) {
        filtered_cos = cosf(raw_rad);
        filtered_sin = sinf(raw_rad);
        filter_initialized = 1;
    } else {
        filtered_cos += ANGLE_FILTER_ALPHA * (cosf(raw_rad) - filtered_cos);
        filtered_sin += ANGLE_FILTER_ALPHA * (sinf(raw_rad) - filtered_sin);

        float norm = sqrtf(filtered_cos * filtered_cos + filtered_sin * filtered_sin);
        if (norm > 1e-6f) {
            filtered_cos /= norm;
            filtered_sin /= norm;
        }
    }

    // 输出：单圈角度
    float filtered_angle = atan2f(filtered_sin, filtered_cos);
    if (filtered_angle < 0.0f) filtered_angle += 2.0f * PI;

    KTH7823_Angle_Rad = filtered_angle;
    KTH7823_Angle_Deg = filtered_angle * 180.0f / PI;

    // 输出：多圈总角度
    KTH7823_Angle_Multiturn = angle_unwrapped;

    // 采样心跳计数（主循环每拍递增），供 Protect 判断编码器是否在刷新
    encoder_update_count++;
}

/**
 * @brief 更新速度（通过角度差分计算）
 * @note  dt 用 DWT->CYCCNT @72MHz 测量（13.9ns 分辨率），取代 HAL_GetTick 的 1ms
 *        量化，避免 E[Δθ/Δt] 系统性偏高（实测约 +5%~10%）
 */
void KTH7823_UpdateSpeed(void)
{
    uint32_t now = DWT->CYCCNT;

    if (last_speed_tick == 0) {   // 首帧建基准
        last_speed_tick = now;
        last_unwrapped_angle = angle_unwrapped;
        KTH7823_Speed_Rads = 0.0f;
        KTH7823_Speed_RPM  = 0.0f;
        return;
    }

    uint32_t elapsed = now - last_speed_tick;
    if (elapsed == 0) return;                                  // 同周期重复调用，跳过（而非钳到 1e-6）

    float dt = (float)elapsed / (float)SystemCoreClock;        // 秒

    float delta_angle = angle_unwrapped - last_unwrapped_angle;
    if (delta_angle >  PI) delta_angle -= 2.0f * PI;
    if (delta_angle < -PI) delta_angle += 2.0f * PI;

    float speed_raw = delta_angle / dt;
    KTH7823_Speed_Rads += SPEED_FILTER_ALPHA * (speed_raw - KTH7823_Speed_Rads);
    KTH7823_Speed_RPM = KTH7823_Speed_Rads * 60.0f / (2.0f * PI);

    last_unwrapped_angle = angle_unwrapped;
    last_speed_tick = now;
}

/* 编码器刷新入口已收敛到主循环：main.c 的 while(1) 每轮依次调用
 * KTH7823_UpdateAngle() + KTH7823_UpdateSpeed()，并 DWT_DelayMs(1) 定 1ms 节拍。
 * 阻塞式测试命令（串口 ang=）内部也需自行调用这两个函数，见 uart_cmd.c。 */
