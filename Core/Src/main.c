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
#include "Protect.h"     // Protect_StopPWM（故障时关断输出但保持 TIM1 计数）


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 注：上电自动执行启动序列已迁移至 motor_control.c（MotorControl_BootSequence）

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
      Protect_StopPWM();   // 关断三路输出但补回 CEN，避免 TIM1 停摆冻结控制中断与故障状态机
			// 设置标志，通知 FOC 状态机进入退避
			fault_triggered = 1;
		} 
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 上电自动执行启动序列已迁移至 motor_control.c（MotorControl_BootSequence） */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	FDCAN_App_Init();
	UART_Cmd_Init();
	MotorControl_Init();

	MotorControl_BootSequence();   // 上电自动执行启动序列
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	FOC_SetOpenLoopVoltage(0.0f);  // 开环初始电压 0V（速度默认 0，上电不转）
	
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		// 故障恢复：20kHz 中断只推到 FOC_RECOVER_REQ，本函数执行阻塞式 DRV8353_Recover()
		FOC_FaultRecoverProcess();

		// 调试监控快照：集中刷新关键运行变量到 dbg，供 KEIL Watch 观察
		DBG_Monitor_Update();

		// [TEST] 故障/状态自动串口上报（删除本模块时一并删除本行）
		UartLog_Poll();
		
		// LED 状态指示：正常运行常亮，故障低频闪烁
		LED_Update();

		// 上电自检硬件错误(锁死)指示：覆盖 IDLE 熄灭，闪烁提示需检修
		MotorControl_BootErrorLed();

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
//		UART_SendStatus();

		UART_ReportAlignResult();
		UART_ReportAlignStep();   // 对齐步进事件（[ALIGN] stepN start/end）

		// 非阻塞发送兜底：发送链路异常（HAL 启动失败/出错）时恢复，正常无开销
		UART_Tx_Poll();

		// 自动停止：v= 设为非0 后 3 秒自动把电压归零（测试保护，防忘记停机过热）
		UART_Cmd_AutoStop();

		// 处理 CAN 消息（多节点：只处理发给本节点 NODE_ID 的帧，过滤器已按 ID 区分）
    FDCAN_App_Process();

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
