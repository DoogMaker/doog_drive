#include "uart_cmd.h"
#include "usart.h"
#include "DRV8353.h"
#include "foc.h"
#include "sense.h"
#include "kth7823.h"
#include "Protect.h"
#include "fdcan_app.h"  
#include "dwt_delay.h"   // [TEST] DWT_DelayMs（开环角度运动阻塞延时）
#include "iwdg.h"        // [TEST] HAL_IWDG_Refresh / hiwdg
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>    // fabsf（对齐结果自检用）



//外部变量
extern float manual_voltage;	
extern float manual_speed;
extern float control_dt;                 // 控制周期 (秒)，由 motor_control.c 提供
extern float KTH7823_Angle_Deg;
extern volatile float KTH7823_Angle_Rad;
extern volatile float KTH7823_Speed_Rads;
extern float KTH7823_Speed_RPM;

// 串口接收缓冲区
#define UART_RX_BUF_SIZE  64
static  char rx_buf[UART_RX_BUF_SIZE];
static  uint8_t rx_index = 0;
static  uint8_t cmd_ready = 0;  // 收到完整命令标志

/* ==================== 非阻塞发送缓冲（TX 环形缓冲 + DMA 发送）====================
 * UART_SendString() 把数据拷进环形缓冲、启动 DMA 发送后立即返回；
 *   数据搬运由 DMA1_Channel3(DMAMUX 请求 29) 完成，发送完成中断里续传下一段，
 *   CPU 只在"每段"首尾各介入一次，而不是逐字节进中断。
 *
 * 并发约定：
 *   - tx_head 只由生产者（主循环 / 接收中断）写；
 *   - tx_tail 只由消费者（DMA 发送完成中断）写；
 *   - "入队 + 启动发送" 放在同一临界区内，避免与发送完成中断竞争。
 *   - 空间不足时整条消息丢弃（绝不输出半行），并累加丢弃计数。
 */
#define UART_TX_BUF_SIZE   2048u                 // 必须是 2 的幂
#define UART_TX_BUF_MASK   (UART_TX_BUF_SIZE - 1u)

static uint8_t  tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t tx_head    = 0;   // 生产者写入位置
static volatile uint16_t tx_tail    = 0;   // 消费者读出位置
static volatile uint16_t tx_chunk   = 0;   // 当前 DMA 发送的字节数
static volatile uint8_t  tx_active  = 0;   // 1 = DMA 正在发送
static volatile uint32_t tx_dropped = 0;   // 缓冲满被整条丢弃的消息数

// 开环点动自动归零：输入 v= 后启动计时
#define AUTO_STOP_MS  3000   // 自动归零延时 (ms)
static uint32_t v_start_tick= 0;   // 自动归零截止时间戳 (ms)
static uint8_t  v_active = 0;       // 是否已启动自动归零

// 自动上报开关：log=1 开启
static uint8_t auto_report = 0;
static uint16_t report_fre = 200; //主循环发送实时状态间隔时间

// 帮助信息
static const char* help_msg = 
    "\r\n========== Motor Control Commands ==========\r\n"
    "help           - Show this help\r\n"
    "fault          - Read DRV8353 fault status\r\n"
    "v=<float>      - Set open-loop voltage (V), e.g. v=0.5\r\n"
    "s=<float>      - Set target speed (rad/s), e.g. s=10\r\n"
    "t=<float>      - Set target torque (Nm), e.g. t=1.0\r\n"
    "mode=<0..5>    - Set control mode (0:IDLE 1:OPENLOOP 2:SPEED 3:ALIGN 4:POSITION 5:TORQUE)\r\n"
    "dir=<0|1>      - Set direction (0:forward, 1:reverse)\r\n"
    "status         - Print current status (Mode uses mode= numbering: 6=FAULT)\r\n"
    "log=<0|1>      - Auto report realtime status every 200ms (Ia/Ib/Ic, Vbus, temp, speed, mode)\r\n"
    "pid            - Print all PID gains (Kp/Ki/Kd)\r\n"
    "pid <ring> <Kp> <Ki> <Kd> - Set PID (ring: speed|id|iq|pos)\r\n"
    "enable         - Enable DRV8353 output\r\n"
    "disable        - Disable DRV8353 output\r\n"
    "ang=<deg>      - [TEST] Open-loop rotate given reducer-output angle (deg)\r\n"
    "==============================================\r\n";

//串口命令处理初始化
void UART_Cmd_Init(void)
{
    // 清空缓冲区
    memset(rx_buf, 0, UART_RX_BUF_SIZE);
    rx_index = 0;
    cmd_ready = 0;
    
    // 启动串口接收中断（空闲中断或单字节中断）
    // 使用 HAL_UART_Receive_IT 接收单个字符
    HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buf, 1);
}

//串口接收完成回调（由 HAL 库调用）每次收到一个字符都会进入
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        char c = rx_buf[rx_index];
        if (c == '\n') {
            // 命令结束符：\n 触发（\r 不递增索引，会被随后的 \n 覆盖，故只认 \n）
            rx_buf[rx_index] = '\0';
            if (rx_index > 0) {
                cmd_ready = 1;   // 有内容才标记，避免 \r\n 连续触发空命令
            }
            rx_index = 0;        // 重置，准备接收下一条命令
        } else if (c == '\r') {
            // 忽略 \r：索引不递增，随后到达的 \n 会覆盖此字节
        } else {
            // 普通字符，继续接收
            rx_index++;
            if (rx_index >= UART_RX_BUF_SIZE - 1) {
                rx_index = 0;  // 缓冲区溢出，从头开始
                UART_SendString("\r\nError: Command too long!\r\n");
            }
        }
        // 继续接收下一个字符
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_buf[rx_index], 1);
    }
}

// 把内部 FOC 状态映射为串口统一的模式编号（与 mode= 命令、status 的 Mode 字段完全一致）：
//   0=IDLE 1=OPENLOOP 2=SPEED 3=ALIGN 4=POSITION 5=TORQUE 6=FAULT
// 注意：内部枚举 FOC_STATE_* 的取值与上述编号不同（如开环内部=2），必须经此函数转换后再对外显示。
static int UART_GetModeNumber(void)
{
    switch (foc.mode) {
        case FOC_STATE_IDLE:       return 0;   // 空闲
        case FOC_STATE_OPENLOOP:   return 1;   // 开环
        case FOC_STATE_ALIGN:      return 3;   // 对齐
        case FOC_STATE_CLOSEDLOOP:             // 闭环：按子模式细分
            if      (foc.closedloop_mode == CL_MODE_POSITION) return 4;  // 闭环位置
            else if (foc.closedloop_mode == CL_MODE_TORQUE)   return 5;  // 闭环力矩
            else                                             return 2;  // 闭环速度
        case FOC_STATE_FAULT:      return 6;   // 故障
        default:                   return -1;
    }
}

/* ==================== BEGIN TEST: 开环角度运动（可整段删除）====================
 * 用途：开环模式下按“减速器输出端机械角度”转动指定角度，到位自动停。
 * 原理：开环无位置反馈，靠对“命令电角度速率(target_speed_ramp)”按控制节拍积分判到位。
 *       输出端机械角 -> 电机轴机械角(×减速比) -> 电角度(×极对数)。
 * 说明：
 *   - 角度为“相对当前点”的相对量，非绝对零点；打滑/负载会导致偏差（故不要求精度）。
 *   - 阻塞式：动作期间不处理其他串口命令，带超时保护。
 *   - 转速/电压优先取当前 s=/v= 设定，未设定则用下方默认值。
 * 删除方法：删除本段全部内容 + UART_Cmd_Process 中标注 [TEST] 的那一行
 *          + help_msg 中标注 [TEST] 的那一行 + 头部两个 [TEST] include（可选）。
 * ============================================================================ */
#define TEST_GEAR_RATIO_OUT2MOTOR   12.5f   // 减速器输出端 -> 电机轴 的减速比
#define TEST_OL_DEFAULT_SPEED       5.0f   // 未设转速时的默认开环转速 (rad/s，电机轴)
#define TEST_OL_DEFAULT_VOLTAGE     0.6f    // 未设电压时的默认开环电压 (V)
#define TEST_OL_MOVE_TIMEOUT_MS     5000    // 单次动作超时 (ms)

extern volatile uint32_t control_tick;      // 控制节拍计数（motor_control.c 定义，20kHz）

// 开环转动指定“输出端”机械角度（°，正=正转）。返回 0=完成，-1=参数非法，-2=超时
int UART_Test_OpenLoopMoveAngle(float out_deg)
{
    float out_rad = out_deg * 0.0174532925f;
    if (fabsf(out_rad) < 1e-5f) return -1;

    float spd = fabsf(manual_speed);   if (spd < 1.0f) spd = TEST_OL_DEFAULT_SPEED;
    float vol = fabsf(manual_voltage); if (vol < 0.1f) vol = TEST_OL_DEFAULT_VOLTAGE;
    float dir = (out_rad >= 0.0f) ? 1.0f : -1.0f;

    // 输出端机械角 -> 电机轴机械角 -> 电角度 (rad)
    float target_elec = fabsf(out_rad) * TEST_GEAR_RATIO_OUT2MOTOR * (float)MOTOR_POLE_PAIRS;

    // 进入开环并给定转速/电压
    FOC_SetMode(FOC_STATE_OPENLOOP);
    manual_speed     = dir * spd;
    manual_voltage   = vol;
    foc.target_speed = manual_speed;
    FOC_SetOpenLoopVoltage(vol);

    // 用控制节拍对“命令电角度速率”积分：与中断内推进 openloop_angle 同源，判转够
    uint32_t t0        = HAL_GetTick();
    uint32_t last_tick = control_tick;
    float    moved     = 0.0f;

    while (fabsf(moved) < target_elec) {
        uint32_t now      = control_tick;
        uint32_t dt_ticks = now - last_tick;
        last_tick = now;
        moved += foc.target_speed_ramp * (float)MOTOR_POLE_PAIRS * control_dt * (float)dt_ticks;

        KTH7823_UpdateAngle();     // 维持编码器/失速检测节奏（开环不用其值判位）
        KTH7823_UpdateSpeed();
        UART_Tx_Poll();
        HAL_IWDG_Refresh(&hiwdg);  // 阻塞期间喂狗，防 IWDG 复位
        if ((HAL_GetTick() - t0) > TEST_OL_MOVE_TIMEOUT_MS) break;
        DWT_DelayMs(1);
    }

    // 到位即停（清零给定，保持开环静止）
    manual_voltage   = 0.0f;
    manual_speed     = 0.0f;
    foc.target_speed = 0.0f;
    FOC_SetOpenLoopVoltage(0.0f);

    return ((HAL_GetTick() - t0) > TEST_OL_MOVE_TIMEOUT_MS) ? -2 : 0;
}

// 命令解析：ang=<deg>。返回 1=已处理，0=非本命令
static int UART_Test_AngleCmd(const char* cmd_buf, char* response)
{
    if (strncmp(cmd_buf, "ang=", 4) != 0) return 0;

    float deg = atof(cmd_buf + 4);
    if (deg == 0.0f) {
        UART_SendString("Usage: ang=<deg> (reducer output shaft, nonzero)\r\n");
        return 1;
    }

    UART_SendString("Angle move start...\r\n");   // 先回显，避免阻塞后才提示
    int r = UART_Test_OpenLoopMoveAngle(deg);
    if      (r == 0)  sprintf(response, "Angle move done (out): %.1f deg\r\n", deg);
    else if (r == -2) sprintf(response, "Angle move TIMEOUT (out): %.1f deg (partial)\r\n", deg);
    else              sprintf(response, "Angle move invalid\r\n");
    UART_SendString(response);
    return 1;
}
/* ==================== END TEST: 开环角度运动 ==================== */

//解析并执行命令（在主循环中调用）
void UART_Cmd_Process(void)
{
    if (!cmd_ready) return;
    cmd_ready = 0;  // 清空标志
    
    // 保存当前命令（复制一份，因为 rx_buf 会被覆盖）
    char cmd_buf[UART_RX_BUF_SIZE];
    strcpy(cmd_buf, rx_buf);
    
    // 重置接收缓冲区
    rx_index = 0;
    memset(rx_buf, 0, UART_RX_BUF_SIZE);
    HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buf, 1);
    
    // 解析命令。
    // 说明：response 放静态区而非栈上——主栈只有 1KB，且 UART_Cmd_Process 仅在主循环
    //       调用、不可重入；UART_SendString 是非阻塞的，拷进发送环形缓冲后本缓冲即可复用。
    static char response[384];

    // [TEST] 开环角度运动命令（删除测试功能时连同本行一起删）
    if (UART_Test_AngleCmd(cmd_buf, response)) return;
    
    // help
    if (strcmp(cmd_buf, "help") == 0) {
        UART_SendString(help_msg);
        return;
    }
    
    // fault - 读取故障状态
    if (strcmp(cmd_buf, "fault") == 0) {
        uint16_t status1 = DRV8353_ReadReg(DRV8353_REG_STATUS1);
        uint16_t status2 = DRV8353_ReadReg(DRV8353_REG_STATUS2);
        uint16_t ctrl = DRV8353_ReadReg(DRV8353_REG_DRIVER_CTRL);
        sprintf(response, 
                "STATUS1=0x%04X STATUS2=0x%04X CTRL=0x%04X\r\n",
                status1, status2, ctrl);
        UART_SendString(response);
        
        // 解析具体故障位 (STATUS1 地址 0x00h, DRV8353xS 表 8-11)
        if (status1 & 0x0400) UART_SendString("  FAULT: nFAULT logic OR!\r\n");
        if (status1 & 0x0200) UART_SendString("  VDS_OCP: any phase VDS overcurrent!\r\n");
        if (status1 & 0x0100) UART_SendString("  GDF: Gate driver fault!\r\n");
        if (status1 & 0x0080) UART_SendString("  UVLO: Under voltage lockout!\r\n");
        if (status1 & 0x0040) UART_SendString("  OTSD: Over temperature shutdown!\r\n");
        if (status1 & 0x0020) UART_SendString("  VDS_HA: A high-side VDS!\r\n");
        if (status1 & 0x0010) UART_SendString("  VDS_LA: A low-side VDS!\r\n");
        if (status1 & 0x0008) UART_SendString("  VDS_HB: B high-side VDS!\r\n");
        if (status1 & 0x0004) UART_SendString("  VDS_LB: B low-side VDS!\r\n");
        if (status1 & 0x0002) UART_SendString("  VDS_HC: C high-side VDS!\r\n");
        if (status1 & 0x0001) UART_SendString("  VDS_LC: C low-side VDS!\r\n");
        return;
    }
    
    // status - 打印当前状态
    if (strcmp(cmd_buf, "status") == 0) {
        sprintf(response,
                "Speed=%.2f rad/s (%.2f RPM) | Angle=%.2f deg\r\n"
                "Ia=%.3f A, Ib=%.3f A, Ic=%.3f A\r\n"
                "Id=%.3f A, Iq=%.3f A | Mode=%d\r\n"
                "TargetSpeed=%.2f rad/s | Voltage=%.2f V | V_Bus=%.2f V\r\n"
                "Align: offset=%.4f rad (%.2f deg) | dir=%+d\r\n"
                "ADC_A=%u(%.3fV) ADC_B=%u(%.3fV)\r\n",
                KTH7823_Speed_Rads, KTH7823_Speed_RPM,
                KTH7823_Angle_Deg,
                current_A, current_B, current_C,
                foc.Id, foc.Iq, UART_GetModeNumber(),
                foc.target_speed, manual_voltage, bus_voltage,
                encoder_offset, encoder_offset * 57.2957795f, encoder_direction,
                adc_values[0], (float)adc_values[0] / 4095.0f * 3.3f,
                adc_values[1], (float)adc_values[1] / 4095.0f * 3.3f);
        UART_SendString(response);
        return;
    }
    
    // ==================== PID 在线调参 ====================
    //
    // 功能：通过串口在电机运行时实时查看/修改 4 个 PID 环的比例、积分、微分系数。
    //       参数直接写入 PID_Handle_t 的 params 副本，控制中断（20kHz）内的 PID_Calculate
    //       下一个周期即采用新值，无需复位 MCU。
    //
    // 命令格式（两条）：
    //   1) 查询：pid
    //      打印全部 4 个环（速度/d轴电流/q轴电流/位置）当前的 Kp/Ki/Kd。
    //   2) 设置：pid <ring> <Kp> <Ki> <Kd>
    //      <ring> 取值：speed=速度环、id=d轴电流环、iq=q轴电流环、pos=位置环。
    //      后跟 3 个浮点数为新系数，均以空格分隔。
    //
    // 示例：
    //   pid                    -> 查询所有环参数
    //   pid speed 0.2 1.0 0.0  -> 速度环 Kp=0.2, Ki=1.0, Kd=0.0
    //   pid iq 3.0 0.5 0.0     -> q轴电流环 Kp=3.0, Ki=0.5, Kd=0.0
    //
    // 说明：
    //   - 修改后仅影响运行中的 RAM 参数，掉电/复位后恢复为 FOC.c 中的硬编码初始值；
    //   - 本命令直接写 params（不经过 PID_SetParams），因为 PID_SetParams 只改 Kp/Ki/Kd，
    //     此处直接写等价；写完后调用 PID_Reset 清空积分/微分历史，避免旧累积值造成输出突变。
    // ====================================================
    if (strcmp(cmd_buf, "pid") == 0) {
        sprintf(response,
                "PID speed: Kp=%.4f Ki=%.4f Kd=%.4f\r\n"
                "PID id   : Kp=%.4f Ki=%.4f Kd=%.4f\r\n"
                "PID iq   : Kp=%.4f Ki=%.4f Kd=%.4f\r\n"
                "PID pos  : Kp=%.4f Ki=%.4f Kd=%.4f\r\n",
                pid_speed.params.Kp, pid_speed.params.Ki, pid_speed.params.Kd,
                pid_id.params.Kp, pid_id.params.Ki, pid_id.params.Kd,
                pid_iq.params.Kp, pid_iq.params.Ki, pid_iq.params.Kd,
                pid_position.params.Kp, pid_position.params.Ki, pid_position.params.Kd);
        UART_SendString(response);
        return;
    }

    if (strncmp(cmd_buf, "pid ", 4) == 0) {
        // 解析：环名字符串（最长 7 字符，防溢出）+ 3 个浮点系数
        char ring[8];
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        // sscanf 返回成功解析的字段数，必须恰好等于 4，否则参数个数不对，提示用法
        if (sscanf(cmd_buf + 4, "%7s %f %f %f", ring, &kp, &ki, &kd) != 4) {
            UART_SendString("Usage: pid <speed|id|iq|pos> <Kp> <Ki> <Kd>\r\n");
            return;
        }

        PID_Handle_t *pid = NULL;   // 待设置的 PID 句柄
        const char *name = "";      // 环名（用于回显）
        if      (strcmp(ring, "speed") == 0) { pid = &pid_speed;    name = "speed"; }
        else if (strcmp(ring, "id")    == 0) { pid = &pid_id;       name = "id"; }
        else if (strcmp(ring, "iq")    == 0) { pid = &pid_iq;       name = "iq"; }
        else if (strcmp(ring, "pos")   == 0) { pid = &pid_position; name = "pos"; }
        else {
            UART_SendString("Unknown ring! Use speed|id|iq|pos\r\n");
            return;
        }

        pid->params.Kp = kp;
        pid->params.Ki = ki;
        pid->params.Kd = kd;
        PID_Reset(pid);

        // 回显本次设置结果
        sprintf(response, "PID %s updated: Kp=%.4f Ki=%.4f Kd=%.4f\r\n", name, kp, ki, kd);
        UART_SendString(response);
        return;
    }

    // v=<voltage> - 设置开环电压（设为非0 时启动自动停止定时器）
    if (strncmp(cmd_buf, "v=", 2) == 0) {
        float val = atof(cmd_buf + 2);
        manual_voltage = val;
        FOC_SetOpenLoopVoltage(val);
        if (val != 0.0f) {
            v_start_tick = HAL_GetTick();   // 启动 AUTO_STOP_MS 倒计时
            v_active = 1;
            sprintf(response, "Voltage set to %.3f V (auto stop in %d ms)\r\n", val, AUTO_STOP_MS);
        } else {
            v_active = 0;                   // 手动归零则取消定时器
            sprintf(response, "Voltage set to 0 (auto stop cancelled)\r\n");
        }
        UART_SendString(response);
        return;
    }

    // s=<speed> - 设置目标速度
    if (strncmp(cmd_buf, "s=", 2) == 0) {
        float val = atof(cmd_buf + 2);
        foc.target_speed = val;
        manual_speed = val;
        sprintf(response, "Target speed set to %.2f rad/s\r\n", val);
        UART_SendString(response);
        return;
    }

    // log=<0|1> - 自动实时上报状态
    if (strncmp(cmd_buf, "log=", 4) == 0) {
        int on = atoi(cmd_buf + 4);
        auto_report = (on != 0) ? 1 : 0;
        sprintf(response, "Auto report %s (every %d ms)\r\n", auto_report ? "ON" : "OFF",report_fre);
        UART_SendString(response);
        return;
    }

    // mode=<mode> - 切换控制模式
    if (strncmp(cmd_buf, "mode=", 5) == 0) {
        int mode = atoi(cmd_buf + 5);
        switch(mode) {
            case 0: FOC_SetMode(FOC_STATE_IDLE); break;
            case 1: FOC_SetMode(FOC_STATE_OPENLOOP); break;
            case 2: FOC_SetMode(FOC_STATE_CLOSEDLOOP); FOC_SetClosedLoopMode(CL_MODE_SPEED); break;
            case 3: FOC_SetMode(FOC_STATE_ALIGN); break;
            case 4: FOC_SetMode(FOC_STATE_CLOSEDLOOP); FOC_SetClosedLoopMode(CL_MODE_POSITION); break;  // 闭环位置子模式（需输出端编码器）
            case 5: FOC_SetMode(FOC_STATE_CLOSEDLOOP); FOC_SetClosedLoopMode(CL_MODE_TORQUE); break;   // 闭环力矩子模式
            default:
                UART_SendString("Invalid mode! (0:IDLE, 1:OPENLOOP, 2:SPEED, 3:ALIGN, 4:POSITION, 5:TORQUE)\r\n");
                return;
        }
        sprintf(response, "Mode switched to %d\r\n", mode);
        UART_SendString(response);
        return;
    }

    // t=<torque> - 设置目标转矩 (Nm)，力矩子模式使用
    if (strncmp(cmd_buf, "t=", 2) == 0) {
        float val = atof(cmd_buf + 2);
        FOC_SetTargetTorque(val);
        sprintf(response, "Target torque set to %.3f Nm\r\n", val);
        UART_SendString(response);
        return;
    }
    
    // dir=<0|1> - 设置方向
    if (strncmp(cmd_buf, "dir=", 4) == 0) {
        int dir = atoi(cmd_buf + 4);
        if (dir == 0) {
            foc.target_speed = fabs(foc.target_speed);
            manual_speed = foc.target_speed;
            UART_SendString("Direction: Forward\r\n");
        } else if (dir == 1) {
            foc.target_speed = -fabs(foc.target_speed);
            manual_speed = foc.target_speed;
            UART_SendString("Direction: Reverse\r\n");
        } else {
            UART_SendString("Invalid direction! Use 0 (forward) or 1 (reverse)\r\n");
        }
        return;
    }
    
    // enable / disable
    if (strcmp(cmd_buf, "enable") == 0) {
        DRV8353_Recover();           // 清故障 + 恢复PWM + 使能 + 重启三路
        FOC_SetMode(FOC_STATE_OPENLOOP);
        Protect_CommDisarm();        // 进入本地调试态：关闭 CAN 失联保护，避免串口 enable 被失联检测打回 FAULT
        UART_SendString("DRV8353 output enabled (fault cleared)\r\n");
        return;
    }
    if (strcmp(cmd_buf, "disable") == 0) {
        DRV8353_Disable();
        UART_SendString("DRV8353 output disabled\r\n");
        return;
    }
    
    // 未知命令
    UART_SendString("Unknown command. Type 'help' for available commands.\r\n");
}

// 发送缓冲剩余可写入字节数（保留 1 字节用于区分空/满）
uint16_t UART_TxFree(void)
{
    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    uint16_t used = (uint16_t)((head - tail) & UART_TX_BUF_MASK);
    return (uint16_t)(UART_TX_BUF_MASK - used);
}

// 因发送缓冲满而被整条丢弃的消息数（正常调试不应增长）
uint32_t UART_TxDropCount(void)
{
    return tx_dropped;
}

// 空闲则把缓冲里的一段连续数据挂到 DMA 发送。
// 调用前必须已关中断（或在中断上下文），以免与发送完成中断竞争。
static void UART_Tx_Kick(void)
{
    if (tx_active) return;                     // 已有传输在进行，回调会续传
    if (tx_head == tx_tail) return;            // 无待发数据

    uint16_t tail = tx_tail;
    uint16_t head = tx_head;
    uint16_t len  = (head > tail) ? (uint16_t)(head - tail)
                                  : (uint16_t)(UART_TX_BUF_SIZE - tail);

    tx_chunk  = len;
    tx_active = 1;
    /* DMA_NORMAL：一段发完自动停，由 TxCplt 回调再挂下一段 */
    if (HAL_UART_Transmit_DMA(&huart3, &tx_buf[tail], len) != HAL_OK) {
        tx_chunk  = 0;
        tx_active = 0;                         // 启动失败：交由 UART_Tx_Poll / 下次发送重试
    }
}

// 主循环调用：兜底重试。正常情况下数据由 UART_SendString 与 DMA 发送完成回调推进，
// 此函数用于 HAL 启动失败或 DMA 出错后恢复，保证发送链路不会永久卡死。
void UART_Tx_Poll(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    UART_Tx_Kick();
    __set_PRIMASK(primask);
}

//发送字符串（非阻塞：拷入发送环形缓冲后立即返回，可在中断中调用）
void UART_SendString(const char* str)
{
    if (str == NULL) return;

    uint16_t len = (uint16_t)strlen(str);
    if (len == 0u) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t head = tx_head;
    uint16_t tail = tx_tail;
    uint16_t used = (uint16_t)((head - tail) & UART_TX_BUF_MASK);

    if ((uint16_t)(UART_TX_BUF_MASK - used) < len) {
        tx_dropped++;                          // 空间不足：整条丢弃，避免输出半行
    } else {
        for (uint16_t i = 0u; i < len; i++) {
            tx_buf[head] = (uint8_t)str[i];
            head = (uint16_t)((head + 1u) & UART_TX_BUF_MASK);
        }
        tx_head = head;
        UART_Tx_Kick();
    }

    __set_PRIMASK(primask);
}

// DMA 发送完成回调（在 USART3 的 TC 中断里执行，此时最后一位已移出）：
// 推进读指针，并续传下一段
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        tx_tail   = (uint16_t)((tx_tail + tx_chunk) & UART_TX_BUF_MASK);
        tx_chunk  = 0;
        tx_active = 0;
        UART_Tx_Kick();                        // 已在中断上下文，可直接续传
    }
}

// 串口出错回调：释放发送状态，并在接收被中止时重新挂上接收中断
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        // 发送侧：DMA 出错时 HAL 已把通道置回 READY（见 HAL_DMA_IRQHandler），
        // 这里只需释放状态，交由 UART_Tx_Poll 重新发起；本段 tx_tail 未推进，会重发。
        tx_chunk  = 0;
        tx_active = 0;

        // 接收侧：过载(ORE)等错误会让 HAL 中止接收，这里重新挂上，避免命令通道永久失效
        if (huart->RxState == HAL_UART_STATE_READY) {
            HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_buf[rx_index], 1);
        }
    }
}

//发送状态反馈（主循环调用；需先用串口 log=1 开启）
void UART_SendStatus(void)
{
    if (!auto_report) return;   // 未开启自动上报则直接返回

    // 如果频繁调用，建议加个时间限制，防止刷屏
    static uint32_t last_send = 0;
    if (HAL_GetTick() - last_send < report_fre) return;  // 每report_fre ms发送一次

    // 背压：发送缓冲快满（上位机没在读/终端卡顿）就跳过本次上报，
    // 避免把命令回显等消息挤掉；缓冲一空出来就会立即补发。
    if (UART_TxFree() < 160u) return;

    last_send = HAL_GetTick();

    char buf[192];
    sprintf(buf,
            "Ia=%+.2f Ib=%+.2f Ic=%+.2f A | Vbus=%.1f V | T=%.1f C | S=%.1f rad/s | Mode=%d | ADC_A=%u ADC_B=%u\r\n",
            current_A, current_B, current_C,
            bus_voltage,
            ntc_temperature,
            KTH7823_Speed_Rads,
            UART_GetModeNumber(),
            adc_values[0], adc_values[1]);
    UART_SendString(buf);
}

// 自动停止（主循环调用）：v= 设为非0 后经过 AUTO_STOP_MS，自动把电压归零、停止输出。
// 用于开环测试，防止来不及手动发 v=0 造成电机/驱动器持续大电流过热。
void UART_Cmd_AutoStop(void)
{
    if (!v_active) return;                                    // 定时器未启动
    if (HAL_GetTick() - v_start_tick < AUTO_STOP_MS) return;   // 未到时间

    manual_voltage = 0.0f;
    FOC_SetOpenLoopVoltage(0.0f);   // 电压上限归零 → V/F 算出的 Vq 被限到 0，立即停止输出
    v_active = 0;
    UART_SendString("\r\n[AUTO STOP] voltage -> 0 (timeout)\r\n");
}

// 对齐完成结果上报（主循环调用）
// 对齐逻辑跑在控制中断（20kHz）里，那里只存数值 + 置标志；本函数负责格式化与发送。
void UART_ReportAlignResult(void)
{
    if (!align_report_pending) return;
    align_report_pending = 0;      // 先清标志，避免打印期间又被置位造成重复上报

    float  dlt = align_report_delta;
    float  off = align_report_offset;
    int8_t dir = align_report_direction;

    // 理论机械转角 = 90°(电) / 极对数：0° 与 90° 两步之间转子应转过的角度
    float expect = 1.5707963f / (float)MOTOR_POLE_PAIRS;   // π/2 / 21 ≈ 0.0748 rad

    char buf[192];
    sprintf(buf,
            "\r\n[ALIGN] done: delta=%+.4f rad (%+.2f deg), expect=%.4f rad, dir=%+d, offset=%.4f rad (%.2f deg), nsamp=%u\r\n",
            dlt, dlt * 57.2957795f, expect,
            dir,
            off, off * 57.2957795f,
            (unsigned)align_report_samples);
    UART_SendString(buf);

    // 自检：转角明显偏小 -> 励磁不足，转子基本没动，本次结果不可信
    if (fabsf(dlt) < 0.5f * expect) {
        UART_SendString("[ALIGN] WARN: delta too small, rotor likely did NOT move."
                        " Increase ALIGN_VOLTAGE (0.6V is a good starting point;"
                        " higher is NOT always better, >3V trips stall protection)\r\n");
    }
    // 自检：转角明显偏大 -> 转子滑过一极或被外力带动，编码器读数存疑
    else if (fabsf(dlt) > 1.5f * expect) {
        UART_SendString("[ALIGN] WARN: delta too large, rotor slipped or was disturbed"
                        " (check load / encoder mount)\r\n");
    }		    
    // 发送帮助信息
    UART_SendString(help_msg);
}

// ==================== 上电参数校验打印（启动序列调用）====================
// 逐项检查控制参数与运行条件，输出数值 + PASS/WARN，便于上电一眼确认配置是否可信。
// 复用既有非阻塞发送；纯打印，不改变任何运行状态。
void UART_PrintSystemReport(void)
{
    static char buf[128];   // 静态区，避免占用主循环小栈（不可重入，仅主循环/启动序列调用）

    UART_SendString("\r\n===== SYSTEM PARAM CHECK =====\r\n");

    // ---- PID 四环（增益应非负，负值会改变环的极性）----
    sprintf(buf, "PID speed Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_speed.params.Kp, pid_speed.params.Ki, pid_speed.params.Kd);
    UART_SendString(buf);
    sprintf(buf, "PID id    Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_id.params.Kp, pid_id.params.Ki, pid_id.params.Kd);
    UART_SendString(buf);
    sprintf(buf, "PID iq    Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_iq.params.Kp, pid_iq.params.Ki, pid_iq.params.Kd);
    UART_SendString(buf);
    sprintf(buf, "PID pos   Kp=%+.4f Ki=%+.4f Kd=%+.4f\r\n",
            pid_position.params.Kp, pid_position.params.Ki, pid_position.params.Kd);
    UART_SendString(buf);

    uint8_t pid_ok = (pid_speed.params.Kp    >= 0.0f && pid_speed.params.Ki    >= 0.0f && pid_speed.params.Kd    >= 0.0f &&
                     pid_id.params.Kp       >= 0.0f && pid_id.params.Ki       >= 0.0f && pid_id.params.Kd       >= 0.0f &&
                     pid_iq.params.Kp       >= 0.0f && pid_iq.params.Ki       >= 0.0f && pid_iq.params.Kd       >= 0.0f &&
                     pid_position.params.Kp >= 0.0f && pid_position.params.Ki >= 0.0f && pid_position.params.Kd >= 0.0f) ? 1u : 0u;
    sprintf(buf, "  -> PID gains          : %s\r\n", pid_ok ? "PASS" : "WARN (negative gain)");
    UART_SendString(buf);

    // ---- 位置软限位（输出端，rad；须 min < max）----
    uint8_t lim_ok = (soft_limit_min < soft_limit_max) ? 1u : 0u;
    sprintf(buf, "  -> SoftLimit [%.3f, %.3f] rad : %s\r\n",
            soft_limit_min, soft_limit_max, lim_ok ? "PASS" : "WARN (min >= max)");
    UART_SendString(buf);

    // ---- 位置跟随误差阈值（rad；须 > 0）----
    uint8_t pos_ok = (pos_error_limit > 0.0f) ? 1u : 0u;
    sprintf(buf, "  -> PosErrLimit %.4f rad   : %s\r\n",
            pos_error_limit, pos_ok ? "PASS" : "WARN (> 0 required)");
    UART_SendString(buf);

    // ---- 转矩常数 Kt（Nm/A；须按实测设置，0.1 为代码中的占位值）----
    const char *kt_msg = (foc.torque_constant <= 0.0f) ? "WARN (> 0 required)"
                       : (foc.torque_constant == 0.1f) ? "WARN (placeholder, set actual Kt)"
                       : "PASS";
    sprintf(buf, "  -> TorqueConst %.4f Nm/A : %s\r\n", foc.torque_constant, kt_msg);
    UART_SendString(buf);

    // ---- 母线电压（对比 Protect.h 阈值区间）----
    uint8_t bus_ok = (bus_voltage > BUS_UNDERVOLTAGE_THRESHOLD &&
                      bus_voltage < BUS_OVERVOLTAGE_THRESHOLD) ? 1u : 0u;
    sprintf(buf, "  -> BusVoltage %.2f V (%.1f~%.1f) : %s\r\n",
            bus_voltage, BUS_UNDERVOLTAGE_THRESHOLD, BUS_OVERVOLTAGE_THRESHOLD,
            bus_ok ? "PASS" : "WARN (out of range)");
    UART_SendString(buf);

    // ---- 线圈温度（NTC；低于关断阈值）----
    uint8_t temp_ok = (ntc_temperature < OVERTEMP_SHUTDOWN) ? 1u : 0u;
    sprintf(buf, "  -> CoilTemp %.1f C (limit %.0f) : %s\r\n",
            ntc_temperature, OVERTEMP_SHUTDOWN, temp_ok ? "PASS" : "WARN (over temp)");
    UART_SendString(buf);

    // ---- 控制周期（TIM1 20kHz 更新中断 -> 期望 50µs）----
    float dt_us = control_dt * 1e6f;
    uint8_t dt_ok = (dt_us > 49.0f && dt_us < 51.0f) ? 1u : 0u;
    sprintf(buf, "  -> ControlDt %.2f us (expect 50) : %s\r\n",
            dt_us, dt_ok ? "PASS" : "WARN (check TIM1)");
    UART_SendString(buf);

    // ---- 关键编译期常量 ----
    sprintf(buf, "  -> PolePairs=%d  ClosedLoopEn=%d  NodeID=%d\r\n",
            MOTOR_POLE_PAIRS, FOC_CLOSEDLOOP_ENABLE, NODE_ID);
    UART_SendString(buf);

    UART_SendString("===== END PARAM CHECK =====\r\n");
}