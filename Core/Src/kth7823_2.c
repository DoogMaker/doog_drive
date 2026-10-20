#include "KTH7823_2.h"
#include "arm_math.h"
#include "spi.h"
#include "dwt_delay.h"   // DWT_DelayMs
#include "main.h"        // ME2_CS_Pin / ME2_CS_GPIO_Port


// 全局变量
volatile float Encoder2_Angle_Rad = 0.0f;      // 单圈角度 [0, 2π)
float Encoder2_Angle_Deg        = 0.0f;        // 单圈角度（度）
float Encoder2_Angle_Multiturn  = 0.0f;        // 多圈总角度（连续无界，弧度）
volatile float Encoder2_Speed_Rads = 0.0f;     // 速度 (rad/s)
float Encoder2_Speed_RPM        = 0.0f;        // 速度 (RPM)

// 刷新心跳计数器：每次 Encoder2_UpdateAngle 调用 +1
volatile uint32_t encoder2_update_count = 0;

// 滤波变量
static float filtered_cos = 1.0f;
static float filtered_sin = 0.0f;
static uint8_t filter_initialized = 0;
static uint16_t last_raw_angle = 0;
static uint8_t  raw_initialized = 0;
static float    angle_unwrapped = 0.0f;        // 连续单圈角度
static float    last_unwrapped_angle = 0.0f;   // 上一帧连续角度（速度差分基准）
static uint32_t last_speed_tick = 0;           // 上次 UpdateSpeed 的真实时间戳(ms)

// SPI 收发一帧 16bit（uint16_t 单帧 + Size=1，避免 16 位模式下字节序/越界问题）
static uint16_t spi_exchange_frame(uint16_t tx)
{
    uint16_t txw = tx;
    uint16_t rxw = 0;

    HAL_GPIO_WritePin(ME2_CS_GPIO_Port, ME2_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(ENCODER2_SPI, (uint8_t *)&txw, (uint8_t *)&rxw, 1, 100);
    HAL_GPIO_WritePin(ME2_CS_GPIO_Port, ME2_CS_Pin, GPIO_PIN_SET);

    return rxw;   // HAL 16 位模式已写入完整 16 位
}

// TODO: 读取原始角度（芯片协议待定，占位实现）
uint16_t Encoder2_ReadAngleRaw(void)
{
    // TODO: 芯片确定后，按数据手册补全命令帧/寄存器地址/数据位解析。
    // 占位实现：发读指令（命令字待定），返回 16 位原始角度值。
    spi_exchange_frame(0x0000);           // 第一帧：发读指令（返回的是上一次旧值，丢弃）
    return spi_exchange_frame(0x0000);    // 返回本次原始角度
}

// 角度弧度转换
static float raw_to_rad(uint16_t raw)
{
    return (float)raw / ENCODER2_ANGLE_RESOLUTION * 2.0f * PI;  // 65536 个刻度覆盖 2π
}

// 初始化（芯片相关配置待补）
void Encoder2_Init(void)
{
    Encoder2_Angle_Rad = 0.0f;
    Encoder2_Angle_Deg = 0.0f;
    Encoder2_Angle_Multiturn = 0.0f;
    Encoder2_Speed_Rads = 0.0f;
    Encoder2_Speed_RPM = 0.0f;

    filtered_cos = 1.0f;
    filtered_sin = 0.0f;
    filter_initialized = 0;
    raw_initialized = 0;
    angle_unwrapped = 0.0f;
    last_unwrapped_angle = 0.0f;  // 复位速度差分基准
    last_speed_tick = 0;          // 复位速度计时基准

    // TODO: 芯片确定后补全初始化（寄存器配置/自检/分辨率设置等）

    // 丢弃首次无效值 + 建立通信
    Encoder2_ReadAngleRaw();
    DWT_DelayMs(10);
}

// 更新当前角度（应在主循环每个节拍调用）
void Encoder2_UpdateAngle(void)
{
    uint16_t raw = Encoder2_ReadAngleRaw();
    float raw_rad = raw_to_rad(raw);

    // 多圈解包裹：用 16 位带符号差值累积
    if (!raw_initialized) {
        last_raw_angle = raw;
        angle_unwrapped = raw_rad;
        raw_initialized = 1;
    } else {
        int16_t delta_raw = (int16_t)(raw - last_raw_angle);  // 自动处理跨 0xFFFF 边界
        last_raw_angle = raw;
        angle_unwrapped += (float)delta_raw / ENCODER2_ANGLE_RESOLUTION * 2.0f * PI;
    }

    // sin/cos 一阶低通滤波
    if (!filter_initialized) {
        filtered_cos = cosf(raw_rad);
        filtered_sin = sinf(raw_rad);
        filter_initialized = 1;
    } else {
        filtered_cos += ENCODER2_FILTER_ALPHA * (cosf(raw_rad) - filtered_cos);
        filtered_sin += ENCODER2_FILTER_ALPHA * (sinf(raw_rad) - filtered_sin);

        float norm = sqrtf(filtered_cos * filtered_cos + filtered_sin * filtered_sin);
        if (norm > 1e-6f) {
            filtered_cos /= norm;
            filtered_sin /= norm;
        }
    }

    // 输出：单圈角度
    float filtered_angle = atan2f(filtered_sin, filtered_cos);
    if (filtered_angle < 0.0f) filtered_angle += 2.0f * PI;

    Encoder2_Angle_Rad = filtered_angle;
    Encoder2_Angle_Deg = filtered_angle * 180.0f / PI;

    // 输出：多圈总角度
    Encoder2_Angle_Multiturn = angle_unwrapped;

    // 心跳计数
    encoder2_update_count++;
}

// 更新速度（通过角度差分 + 真实时间间隔计算，不依赖固定控制周期）
void Encoder2_UpdateSpeed(void)
{
    uint32_t now = HAL_GetTick();

    // 首帧：仅建立基准，不计算速度
    if (last_speed_tick == 0) {
        last_speed_tick = now;
        last_unwrapped_angle = angle_unwrapped;
        Encoder2_Speed_Rads = 0.0f;
        Encoder2_Speed_RPM  = 0.0f;
        return;
    }

    // 真实时间间隔（ms -> s）
    float dt = (float)(now - last_speed_tick) / 1000.0f;
    if (dt <= 1e-6f) dt = 1e-6f;   // 防除零/异常

    float delta_angle = angle_unwrapped - last_unwrapped_angle;

    // 处理角度溢出（0 -> 2π 跳变）
    if (delta_angle > PI) delta_angle -= 2.0f * PI;
    if (delta_angle < -PI) delta_angle += 2.0f * PI;

    Encoder2_Speed_Rads = delta_angle / dt;
    Encoder2_Speed_RPM = Encoder2_Speed_Rads * 60.0f / (2.0f * PI);

    last_unwrapped_angle = angle_unwrapped;
    last_speed_tick = now;
}
