#include "motor_control.h"
#include "drv8353.h"
#include "kth7823.h"
#include "sense.h"
#include "foc.h"
#include "tim.h"
#include "adc.h"
#include "stm32g4xx_hal_rcc.h"
#include "iwdg.h"        // HAL_IWDG_Refresh / hiwdg（启动序列阻塞期间喂狗）
#include "dwt_delay.h"   // DWT_DelayMs（自检重试间隔）
#include "uart_cmd.h"    // UART_SendString / UART_PrintSystemReport 等
#include <stdio.h>       // sprintf（启动序列自检报告格式化）

//电机参数
//线电阻为145mΩ±10%，线电感为59.7uH±15%，接法为星型，电机额定转速1812，额定电压24V

#define BOOT_SC_ATTEMPT_MAX    3      // 参数自检总尝试次数（计入第一次，共 3 次）
#define BOOT_SC_RETRY_DELAY_MS 20     // 自检重试之间的短延时(ms)，让驱动/SPI 稳定
#define BOOT_DEV_ERR_BLINK_MS  200    // 硬件错误(锁死)时 LED 闪烁半周期(ms)

extern float manual_voltage;   // 手动给定电压 (V)（定义于 main.c）
extern float manual_speed;     // 手动给定速度 (rad/s)（定义于 main.c）

static uint8_t boot_hw_error = 0;   // 1 = 上电自检三次仍不通过，硬件错误（锁死控制命令，需检修）

volatile uint32_t control_tick = 0;
float control_dt = 0.01f;  // 控制周期 (秒)，由 MotorControl_Init 按 TIM1 频率计算
volatile uint8_t motor_ready = 0;   // 电流偏移校准完成前为 0，禁止中断里跑 FOC/保护
extern volatile float KTH7823_Angle_Rad;
extern volatile float KTH7823_Speed_Rads;

//电机控制初始化
void MotorControl_Init(void)
{
    // 计算实际控制周期
    // TIM1 为中心对齐模式，计数时钟 = tim_clk：
    // PWM 频率     f_pwm = tim_clk / (2*(ARR+1))
    // 更新中断频率 f_isr = f_pwm * 2/(RCR+1) = tim_clk / ((ARR+1)*(RCR+1))
		uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
		uint32_t hclk  = HAL_RCC_GetHCLKFreq();
		uint32_t tim_clk = (pclk2 == hclk) ? pclk2 : pclk2 * 2;  // TIM1 时钟：APB2 预分频≠1 时倍频
    uint32_t tim_freq = tim_clk / ((htim1.Init.Period + 1) * (htim1.Init.RepetitionCounter + 1));
    control_dt = 1.0f / tim_freq;
	
    //DRV8353F 初始化
    DRV8353_Init();

    //KTH7823 初始化（电机轴编码器）
    KTH7823_Init();

    //第二编码器初始化（减速器输出端，SPI3，芯片待定）
    Encoder2_Init();

    //FOC 算法初始化
    FOC_Init();

    // 位置环软限位/位置跟随误差阈值初始化
    FOC_SetSoftLimits(-3.14159f, 3.14159f);   // ±180°（输出端），按机械行程调整
    FOC_SetPositionErrorLimit(0.05f);          // 0.05 rad ≈ 2.9°，按需调整

    //三路占空比清零
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    //使能 TIM1 更新中断
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
		
		//启动 TIM1 基本计数
		HAL_TIM_Base_Start_IT(&htim1); 

    // 启动 TIM3 更新中断（编码器刷新，10kHz，与电流采样同相位）
    // [TEST] 主循环刷新模式下不启动 TIM3：编码器改由 main.c 的 while(1) 轮询（约 1kHz）
#if !ENCODER_REFRESH_IN_MAIN_LOOP
    HAL_TIM_Base_Start_IT(&htim3);
#endif
		
    //ADC自校准
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_DIFFERENTIAL_ENDED);
		HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    //采样初始化
    CurrentSense_Init();

    //启动 TIM1 PWM
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    //使能 TIM1 三路输出
    DRV8353_PWM_ctrl_ON(); 

    // 初始化全部完成，允许中断里执行 FOC 控制循环
    motor_ready = 1;
		
}

//中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {

    CurrentSense_Update();	

    // 执行 FOC 算法（电流偏移校准完成前不执行，避免初始化期误触发保护）
    if (motor_ready) {
        FOC_Update();
    }

    control_tick++;
  }
#if !ENCODER_REFRESH_IN_MAIN_LOOP
  else if (htim->Instance == TIM3) {
	
    KTH7823_UpdateISR();
  }
#endif
}


// 上电自动执行启动序列
// 重新参数设置 + 驱动自检：使能 + 重写全部配置 + 清故障 + 重启 PWM，再读回比对
static uint16_t Boot_ParamSetAndSelfCheck(void)
{
    DRV8353_Recover();            // 重新设置参数
    return DRV8353_SelfCheck();   // 自检（读回比对 + STATUS 检查）
}

// 打印自检读回明细（与期望设置值对照）
static void Boot_PrintSelfCheck(uint16_t sc)
{
    static char line[256];   // 静态区：主栈仅 1KB，避免占用（仅启动序列调用，不可重入）
    sprintf(line,
            "[BOOT] DRV8353 self-check: %s\r\n"
            "       readback CTRL=0x%03X(exp 0x%03X) HS=0x%03X(exp 0x%03X) LS=0x%03X(exp 0x%03X)\r\n"
            "                OCP=0x%03X(exp 0x%03X) CSA=0x%03X(exp 0x%03X)\r\n"
            "                STATUS1=0x%03X STATUS2=0x%03X\r\n",
            (sc == DRV8353_SC_OK) ? "PASS" : "FAIL",
            drv8353_selfcheck.driver_ctrl, (DRV8353_PWM_MODE_3X | DRV8353_OCP_ACT_ALL),
            drv8353_selfcheck.gate_hs,     DRV8353_DRIVE_STRENGTH_HS,
            drv8353_selfcheck.gate_ls,     DRV8353_DRIVE_STRENGTH_LS,
            drv8353_selfcheck.ocp_ctrl,    DRV8353_OCP_CONFIG,
            drv8353_selfcheck.csa_ctrl,    DRV8353_CSA_DISABLE,
            drv8353_selfcheck.status1,     drv8353_selfcheck.status2);
    UART_SendString(line);

    if (sc & 0x001F)                    UART_SendString("[BOOT]   config readback mismatch\r\n");
    if (sc & DRV8353_SC_STATUS1_FAULT)  UART_SendString("[BOOT]   STATUS1 reports fault bits\r\n");
    if (sc & DRV8353_SC_STATUS2_FAULT)  UART_SendString("[BOOT]   STATUS2 reports fault bits\r\n");
}

// 控制命令门控（uart_cmd.c / fdcan_app.c 调用）：1=放行 0=拒绝
uint8_t MotorControl_CommandGate(void)
{
    if (boot_hw_error) {
        UART_SendString("[DEVICE] ERROR: HARDWARE ERROR - control disabled. Maintenance required.\r\n");
        return 0;
    }
    return 1;
}

/* 硬件错误 LED 指示（主循环调用）：boot_hw_error=1 时闪烁提示需检修，
 * 需在 LED_Update() 之后调用以覆盖其 IDLE 熄灭指示 */
void MotorControl_BootErrorLed(void)
{
    static uint32_t tick = 0;
    if (!boot_hw_error) return;
    if ((HAL_GetTick() - tick) >= BOOT_DEV_ERR_BLINK_MS) {
        tick = HAL_GetTick();
        Run_Toggle();
    }
}

// 上电自动执行启动序列
void MotorControl_BootSequence(void)
{
    UART_SendString("\r\n\r\n===== BOOT SEQUENCE START =====\r\n");

    // 参数自检：第 1 次直接检；不通过则重新参数设置 + 自检，最多共 3 次
    uint16_t sc = DRV8353_SC_OK;
    uint8_t  attempt;
    for (attempt = 1u; attempt <= BOOT_SC_ATTEMPT_MAX; attempt++) {
        if (attempt == 1u) {
            sc = DRV8353_SelfCheck();              // MotorControl_Init 已完成参数设置
        } else {
            UART_SendString("[BOOT] self-check failed -> retry param-set + self-check...\r\n");
            sc = Boot_ParamSetAndSelfCheck();      // 重新设置参数 + 自检
        }
        Boot_PrintSelfCheck(sc);

        if (sc == DRV8353_SC_OK) {
            UART_SendString("[BOOT] self-check PASS.\r\n");
            break;
        }

        {
            char line[96];
            sprintf(line, "[BOOT] self-check FAILED (attempt %u/%u), code=0x%03X\r\n",
                    (unsigned)attempt, (unsigned)BOOT_SC_ATTEMPT_MAX, sc);
            UART_SendString(line);
        }

        HAL_IWDG_Refresh(&hiwdg);                  // 重试期间喂狗，防 IWDG(约1s) 复位
        if (attempt < BOOT_SC_ATTEMPT_MAX) DWT_DelayMs(BOOT_SC_RETRY_DELAY_MS);
    }

    // 参数校验打印（信息展示，不参与判定）
    UART_PrintSystemReport();

    // 三次仍不通过：硬件错误，进 IDLE 并锁死控制命令
    if (sc != DRV8353_SC_OK) {
        boot_hw_error = 1;

        manual_voltage    = 0.0f;
        manual_speed      = 0.0f;
        foc.target_speed  = 0.0f;
        foc.target_torque = 0.0f;
        FOC_SetMode(FOC_STATE_IDLE);
        LED_Update();

        UART_SendString("[BOOT] ERROR: HARDWARE ERROR - driver self-check failed 3 times. "
                        "Entered IDLE. Maintenance required.\r\n");
        UART_SendString("===== BOOT SEQUENCE END =====\r\n\r\n");
        return;
    }

    // 自检通过：停止输出、清零给定，进入 IDLE 等待控制（不做上电自动对齐）
    manual_voltage    = 0.0f;
    manual_speed      = 0.0f;
    foc.target_speed  = 0.0f;
    foc.target_torque = 0.0f;
    FOC_SetMode(FOC_STATE_IDLE);
    LED_Update();

    UART_SendString("[BOOT] Driver param check PASS. Entered IDLE, waiting for control.\r\n");
    UART_SendString("===== BOOT SEQUENCE END =====\r\n\r\n");
		
    UART_SendString("Please send “help” for more information.\r\n\r\n");		
}

