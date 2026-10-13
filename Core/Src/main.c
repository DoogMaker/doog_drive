/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "foc.h"
#include "DRV8353.h"
#include "fdcan_app.h"
#include "uart_cmd.h"
#include "dwt_delay.h"
#include "encoder2.h"
#include "dbg_monitor.h"
#include <stdio.h>   // sprintf（启动序列自检报告格式化）


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ==================== 上电自动执行启动序列配置 ====================
#define AUTO_ALIGN_ON_BOOT     1      // 1 = 上电自动执行一次转子对齐；0 = 跳过（整机装配/带载场景可关闭）
#define BOOT_ALIGN_TIMEOUT_MS  3000   // 自动对齐等待超时(ms)，超时则放弃并进入 IDLE

float manual_voltage = 0.0f;	// 手动给定电压 (V)，默认 0，上电不转（安全）
float manual_speed = 0.0f;	  // 手动给定速度 (rad/s)，默认 0
volatile uint8_t fault_triggered = 0;	// 驱动故障标志
volatile uint8_t  fault_code = FAULT_CODE_NONE;  // 故障诊断码（锁存），主循环上报后清零
volatile uint16_t fault_status1 = 0;             // 硬件故障 STATUS1 锁存（Disable 前读取）
volatile uint16_t fault_status2 = 0;             // 硬件故障 STATUS2 锁存

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Boot_Sequence_Run(void);   // 上电自动执行启动序列（实现见 USER CODE BEGIN 0）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// 驱动故障回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_12){
			// 先锁存硬件故障详情（此时 EN 尚未拉低、SPI 可用，读到的才是真实故障位）
			fault_status1 = DRV8353_ReadReg(DRV8353_REG_STATUS1);
			fault_status2 = DRV8353_ReadReg(DRV8353_REG_STATUS2);
			fault_code = FAULT_CODE_HARDWARE;
			// 立即停止 PWM 输出（硬件保护，必须快！）
			DRV8353_Disable();
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			// 设置标志，通知 FOC 状态机进入退避
			fault_triggered = 1;
		} 
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== 上电自动执行启动序列 ====================
 * 顺序：驱动自检配置 → 参数校验打印 → 自动对齐(可选) → 进入 IDLE 等待串口/CAN 指令
 *
 * 关键约束：
 *   1) 对齐依赖驱动已完成配置并使能（DRV8353_Init 已在 MotorControl_Init 内完成），
 *      故"驱动自检配置"必须排在对齐之前。
 *   2) 对齐逻辑跑在 20kHz 中断内，其角度源 KTH7823_Angle_Rad 由主循环 SPI 读取刷新；
 *      阻塞等待期间必须持续调用 KTH7823_UpdateAngle()/KTH7823_UpdateSpeed()，
 *      否则角度不刷新会导致对齐结果错误。
 *   3) 启动序列阻塞约 1.6s（对齐两步），必须持续 HAL_IWDG_Refresh()，否则 IWDG(约1s) 复位。
 *   4) 任何异常（自检告警/对齐超时）仅打印提示，不阻断，最终仍进入 IDLE。
 */
static void Boot_Sequence_Run(void)
{
    UART_SendString("\r\n\r\n===== BOOT SEQUENCE START =====\r\n");

    /* ---- 1) 驱动自检配置：读回配置寄存器 + 检查 STATUS ---- */
    uint16_t sc = DRV8353_SelfCheck();
    {
        char line[256];
        sprintf(line,
                "[BOOT] DRV8353 self-check: %s\r\n"
                "       readback CTRL=0x%03X(exp 0x%03X) HS=0x%03X(exp 0x%03X) LS=0x%03X(exp 0x%03X)\r\n"
                "                OCP=0x%03X(exp 0x%03X) CSA=0x%03X(exp 0x%03X)\r\n"
                "                STATUS1=0x%03X STATUS2=0x%03X\r\n",
                (sc == DRV8353_SC_OK) ? "PASS" : "WARN",
                drv8353_selfcheck.driver_ctrl, (DRV8353_PWM_MODE_3X | DRV8353_OCP_ACT_ALL),
                drv8353_selfcheck.gate_hs,     DRV8353_DRIVE_STRENGTH_HS,
                drv8353_selfcheck.gate_ls,     DRV8353_DRIVE_STRENGTH_LS,
                drv8353_selfcheck.ocp_ctrl,    DRV8353_OCP_CONFIG,
                drv8353_selfcheck.csa_ctrl,    DRV8353_CSA_DISABLE,
                drv8353_selfcheck.status1,     drv8353_selfcheck.status2);
        UART_SendString(line);

        if (sc & 0x001F) {
            UART_SendString("[BOOT] WARN: DRV8353 config readback mismatch\r\n");
        }
        if (sc & DRV8353_SC_STATUS1_FAULT) {
            UART_SendString("[BOOT] WARN: DRV8353 STATUS1 reports fault bits\r\n");
        }
        if (sc & DRV8353_SC_STATUS2_FAULT) {
            UART_SendString("[BOOT] WARN: DRV8353 STATUS2 reports fault bits\r\n");
        }
    }

    /* ---- 2) 参数校验打印 ---- */
    UART_PrintSystemReport();

    /* ---- 3) 自动对齐（可用 AUTO_ALIGN_ON_BOOT 关闭）---- */
#if (AUTO_ALIGN_ON_BOOT)
    UART_SendString("[BOOT] Auto-align start (two-step, ~1.6s)...\r\n");
    FOC_SetMode(FOC_STATE_ALIGN);        // 触发对齐（内部会重置对齐步骤与窗口累积）

    {
        uint32_t t0 = HAL_GetTick();
        while (foc.mode == FOC_STATE_ALIGN) {
            /* 对齐角度源必须持续刷新：SPI 读取在主循环，不在 20kHz 中断内做 */
            KTH7823_UpdateAngle();
            KTH7823_UpdateSpeed();
            /* 对齐完成时置 pending，由既有函数完成格式化上报（含 delta 过小/过大 WARN） */
            UART_ReportAlignResult();
            UART_Tx_Poll();
            HAL_IWDG_Refresh(&hiwdg);    // 阻塞期间喂狗，防 IWDG(约1s) 复位

            if ((HAL_GetTick() - t0) > BOOT_ALIGN_TIMEOUT_MS) {
                UART_SendString("[BOOT] WARN: auto-align TIMEOUT, abort\r\n");
                break;
            }
            DWT_DelayMs(1);
        }
        UART_ReportAlignResult();        // 兜底：确保最后一步结果被上报
    }
#else
    UART_SendString("[BOOT] Auto-align disabled (AUTO_ALIGN_ON_BOOT=0)\r\n");
#endif

    /* ---- 4) 进入 IDLE：停止输出、清零给定，等待指令 ---- */
    manual_voltage    = 0.0f;
    manual_speed      = 0.0f;
    foc.target_speed  = 0.0f;
    foc.target_torque = 0.0f;
    FOC_SetMode(FOC_STATE_IDLE);
    LED_Update();                        // 立即刷新 LED 到 IDLE 指示（熄灭）

    UART_SendString("[BOOT] Sequence complete. Entered IDLE, waiting for UART commands.\r\n");
    UART_SendString("===== BOOT SEQUENCE END =====\r\n\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_Init();   // 使能内核 DWT  Cycle 计数器，供 DWT_Delay us/ms 使用

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_ADC2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
	FDCAN_App_Init();
	UART_Cmd_Init();
	MotorControl_Init();

	// 上电自动执行：驱动自检配置 → 参数校验打印 → 自动对齐 → 进入 IDLE 等待串口/CAN 指令
	Boot_Sequence_Run();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	FOC_SetOpenLoopVoltage(0.0f);  // 开环初始电压 0V（速度默认 0，上电不转）
	
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// 调试监控快照：集中刷新关键运行变量到 dbg，供 KEIL Watch 观察（放最前，捕捉上报清零前的 fault_code）
		DBG_Monitor_Update();
		
		// LED 状态指示：正常运行常亮，故障低频闪烁
		LED_Update();

		// 故障监测：仅做诊断上报，真正的退避/恢复由 FOC_Update 顶部的FOC_STATE_FAULT 状态机接管
    // （fault_triggered 在此不清除，留给 FOC 消费）
		if (fault_code != FAULT_CODE_NONE) {
      // 硬件故障：上报 Disable 前锁存的 STATUS1/2；软件故障：上报软件码
      uint16_t status1 = fault_status1;
      uint16_t status2 = fault_status2;
      if (fault_code == FAULT_CODE_STALL) {
          UART_SendString("FAULT: STALL\r\n");// 软件堵转/过流
      } else if (fault_code == FAULT_CODE_ENCODER_STALL) {
          UART_SendString("FAULT: ENCODER_STALL\r\n");// 编码器失速/断线
      } else if (fault_code == FAULT_CODE_OVERTEMP) {
          UART_SendString("FAULT: OVERTEMP\r\n");// MOSFET/线圈过温关断
      } else if (fault_code == FAULT_CODE_OVERVOLT) {
          UART_SendString("FAULT: OVERVOLT\r\n");// 母线过压（回馈泵升）
      } else if (fault_code == FAULT_CODE_UNDERVOLT) {
          UART_SendString("FAULT: UNDERVOLT\r\n");// 母线欠压
      } else if (fault_code == FAULT_CODE_OVERSPEED) {
          UART_SendString("FAULT: OVERSPEED\r\n");// 机械超速
      } else if (fault_code == FAULT_CODE_PHASE_OC) {
          UART_SendString("FAULT: PHASE_OC\r\n");// 单相瞬时峰值过流
      } else if (fault_code == FAULT_CODE_I2T) {
          UART_SendString("FAULT: I2T\r\n");// I²t 热积累过流
      } else if (fault_code == FAULT_CODE_POS_ERROR) {
          UART_SendString("FAULT: POS_ERROR\r\n");// 位置跟随误差
      } else if (fault_code == FAULT_CODE_SOFT_LIMIT) {
          UART_SendString("FAULT: SOFT_LIMIT\r\n");// 软限位持续被撞
      } else if (fault_code == FAULT_CODE_COMM_TIMEOUT) {
          UART_SendString("FAULT: COMM_TIMEOUT\r\n");// 上位机通讯失联（CAN 心跳超时）
      } else {
      if (status1 & 0x0400) UART_SendString("FAULT: FAULT\r\n");// 故障状态，对应nFault
      if (status1 & 0x0200) UART_SendString("FAULT: VDS_OCP\r\n");// VDS监视器过流
      if (status1 & 0x0100) UART_SendString("FAULT: GDF\r\n");// 栅极驱动故障
      if (status1 & 0x0080) UART_SendString("FAULT: UVLO\r\n");// 欠压锁定
      if (status1 & 0x0040) UART_SendString("FAULT: OTSD\r\n");// 过温关断
      if (status1 & 0x0020) UART_SendString("FAULT: VDS_HA\r\n");// A 相高侧VDS过流
      if (status1 & 0x0010) UART_SendString("FAULT: VDS_LA\r\n");// A 相低侧VDS过流
      if (status1 & 0x0008) UART_SendString("FAULT: VDS_HB\r\n");// B 相高侧VDS过流
      if (status1 & 0x0004) UART_SendString("FAULT: VDS_LB\r\n");// B 相低侧VDS过流
      if (status1 & 0x0002) UART_SendString("FAULT: VDS_HC\r\n");// C 相高侧VDS过流
      if (status1 & 0x0001) UART_SendString("FAULT: VDS_LC\r\n");// C 相低侧VDS过流

      // STATUS2：逐相/子系统细分诊断（只读）
      if (status2 & 0x0400) UART_SendString("FAULT: SA_OC\r\n");// A 相电流采样放大器（CSA）过流
      if (status2 & 0x0200) UART_SendString("FAULT: SB_OC\r\n");// B 相 CSA 过流
      if (status2 & 0x0100) UART_SendString("FAULT: SC_OC\r\n");// C 相 CSA 过流
      if (status2 & 0x0080) UART_SendString("FAULT: OTW\r\n");// 过温警告
      if (status2 & 0x0040) UART_SendString("FAULT: GDUV\r\n");// 电荷泵 VCP / VGLS 欠压故障
      if (status2 & 0x0020) UART_SendString("FAULT: VGS_HA\r\n");// A 相高侧 MOSFET 栅源（VGS）驱动故障
      if (status2 & 0x0010) UART_SendString("FAULT: VGS_LA\r\n");// A 相低侧 MOSFET VGS 驱动故障
      if (status2 & 0x0008) UART_SendString("FAULT: VGS_HB\r\n");// B 相高侧 MOSFET VGS 驱动故障
      if (status2 & 0x0004) UART_SendString("FAULT: VGS_LB\r\n");// B 相低侧 MOSFET VGS 驱动故障
      if (status2 & 0x0002) UART_SendString("FAULT: VGS_HC\r\n");// C 相高侧 MOSFET VGS 驱动故障
      if (status2 & 0x0001) UART_SendString("FAULT: VGS_LC\r\n");// C 相低侧 MOSFET VGS 驱动故障
      }
      fault_code = FAULT_CODE_NONE;   // 上报完成后清除锁存
		}

		// 处理串口命令
    UART_Cmd_Process();

		// 自动实时上报状态
		UART_SendStatus();

		// 对齐完成结果上报（对齐在 20kHz 控制中断里只置标志，这里做格式化+发送）
		UART_ReportAlignResult();

		// 非阻塞发送兜底：发送链路异常（HAL 启动失败/出错）时恢复，正常无开销
		UART_Tx_Poll();

		// 自动停止：v= 设为非0 后 3 秒自动把电压归零（测试保护，防忘记停机过热）
		UART_Cmd_AutoStop();

		// 处理 CAN 消息（多节点：只处理发给本节点 NODE_ID 的帧，过滤器已按 ID 区分）
    FDCAN_App_Process();
		
		
		// 开环电压/速度更新
		FOC_SetOpenLoopVoltage(manual_voltage);
		foc.target_speed = manual_speed;

		// 编码器 SPI 读取放到主循环（不在 20kHz 控制中断内阻塞 SPI）
		KTH7823_UpdateAngle();
		KTH7823_UpdateSpeed();
//		Encoder2_UpdateAngle();   // 第二编码器（输出端，SPI3）
//		Encoder2_UpdateSpeed();

		DWT_DelayMs(1);   // 缩短延时，提高角度刷新率（~1kHz）

		// 喂独立看门狗：主循环 1ms 周期喂狗，远小于 1s 超时
		HAL_IWDG_Refresh(&hiwdg);
		 
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
