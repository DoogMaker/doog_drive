#include "motor_control.h"
#include "drv8353.h"
#include "kth7823.h"
#include "sense.h"
#include "foc.h"
#include "tim.h"
#include "adc.h"
#include "stm32g4xx_hal_rcc.h"

//电机参数
//线电阻为145mΩ±10%，线电感为59.7uH±15%，接法为星型，电机额定转速1812，额定电压24V

volatile uint32_t control_tick = 0;
float control_dt = 0.01f;  // 控制周期 (秒)，由 MotorControl_Init 按 TIM1 频率计算
volatile uint8_t motor_ready = 0;   // 电流偏移校准完成前为 0，禁止中断里跑 FOC/保护
extern volatile float KTH7823_Angle_Rad;
extern volatile float KTH7823_Speed_Rads;

//电机控制初始化
void MotorControl_Init(void)
{
    //计算实际控制周期
    // TIM1 为中心对齐模式，计数时钟 = tim_clk：
    //   PWM 频率     f_pwm = tim_clk / (2*(ARR+1))
    //   更新中断频率 f_isr = f_pwm * 2/(RCR+1) = tim_clk / ((ARR+1)*(RCR+1))
    //     RCR=0：上溢、下溢各触发一次 -> f_isr = 2*f_pwm（当前配置：10kHz PWM -> 20kHz 中断，
    //            即"双采样 + 每半周期更新占空比"）
    //     RCR=1：每个 PWM 周期只触发一次 -> f_isr = f_pwm
    // 注意：这里必须用"更新中断频率"，不是 PWM 频率；用错会让 control_dt 差 2/(RCR+1) 倍，
    //       进而导致 PID 等效增益、开环电角度推进速度、I²t 累积速率全部成比例偏离。
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

    // 位置环软限位/位置跟随误差阈值初始化（默认值见 Protect.c，后续可在 main 启动时根据机构行程覆盖）
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
    // 角度/速度 SPI 读取已在主循环完成，中断只消费缓存，避免阻塞 20kHz 控制节拍
    // 注：foc.angle / foc.speed 由 FOC_Update 内部各模式分支统一赋值，此处不再冗余写入

    // 更新采样
    CurrentSense_Update();	

    // 执行 FOC 算法（电流偏移校准完成前不执行，避免初始化期误触发保护）
    if (motor_ready) {
        FOC_Update();
    }

    // 控制节拍计数
    control_tick++;
  }
}

