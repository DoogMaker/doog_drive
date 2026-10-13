#include "sense.h"
#include "adc.h"
#include "arm_math.h"
#include "dwt_delay.h"   // DWT_DelayMs

// ADC 采样值存储（由 DMA 自动填充）
volatile uint16_t adc_values[4];  // A相, B相, 母线电压，MOSFET温度
volatile uint16_t adc2_values[1];  // NTC
// 采样就绪标志
volatile uint8_t adc_dma_ready = 0;

volatile float current_A = 0.0f;     // 三相电流
volatile float current_B = 0.0f;
volatile float current_C = 0.0f;
volatile float bus_voltage = 0.0f;		// 母线电压
volatile float mos_temperature = 0.0f;		// MOSFET温度
volatile float ntc_temperature = 0.0f;		// 线圈温度（中断采样，主循环/保护读取）
static float current_A_offset = 0.0f;
static float current_B_offset = 0.0f;


//电流采样初始化
void CurrentSense_Init(void)
{
	// 启动 DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 4);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_values, 1);   
    // 等第一个数据到来
    uint32_t start_tick = HAL_GetTick();
    while (!adc_dma_ready) {
        if (HAL_GetTick() - start_tick > 50) {
            Error_Handler();   // 50ms 无数据：TIM1 未触发或 ADC 配置错误
        }
    }
		// 再稳定一个周期
		DWT_DelayMs(10);
		
	// 采样 64 次求平均偏移（电机必须静止）
    float sum_A = 0.0f, sum_B = 0.0f;
    for (int i = 0; i < 64; i++) {
        sum_A += CurrentSense_Calculate(adc_values[0]);
        sum_B += CurrentSense_Calculate(adc_values[1]);
        DWT_DelayMs(1);
    }
    current_A_offset = sum_A / 64.0f;
    current_B_offset = sum_B / 64.0f;		
}

//根据 ADC 原始值计算电流
float CurrentSense_Calculate(uint16_t adc_raw)
{
    // ADC 值转电压
    float v_out = (float)adc_raw / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
	
    // 计算采样电阻压降（减去 VREF 偏置）
    float v_sense = v_out - INA240_VREF;
	
    // 计算实际电流
    float current = v_sense / (INA240_GAIN * R_SHUNT);		
		
    return current;
}

//更新三相电流值（应在每个控制周期调用）
void CurrentSense_Update(void)
{
    // ADC 值由 DMA 自动更新
    current_A = CurrentSense_Calculate(adc_values[0]) - current_A_offset;
    current_B = CurrentSense_Calculate(adc_values[1]) - current_B_offset;
    // C相电流由基尔霍夫电流定律计算
    current_C = -(current_A + current_B);

    // 母线电压（adc_values[2]）
    static uint16_t vbus_hist[3] = {0, 0, 0};
    {
        vbus_hist[0] = vbus_hist[1];
        vbus_hist[1] = vbus_hist[2];
        vbus_hist[2] = adc_values[2];

        uint16_t a = vbus_hist[0], b = vbus_hist[1], c = vbus_hist[2], med;
        if ((a <= b && b <= c) || (c <= b && b <= a))      med = b;
        else if ((b <= a && a <= c) || (c <= a && a <= b)) med = a;
        else                                                med = c;

        bus_voltage = ((float)med / ADC_MAX_VALUE) * ADC_REF_VOLTAGE * VOLTAGE_DIVIDER_RATIO;
    }
    
	// NTC温度（adc_values[3]）—— 上拉接法：VCC→NTC→采样点→下拉电阻→GND
    uint16_t mos_raw = adc_values[3];
    if (mos_raw == 0) mos_raw = 1;  // 防除零（低温时 NTC 阻值大，采样电压趋近 0）
    float mos_ntc = PULL_DOWN_RES * (ADC_MAX_VALUE - (float)mos_raw) / (float)mos_raw;
    if (mos_ntc < 1.0f) mos_ntc = 1.0f;   // 防 logf(0)（NTC 短路时）
    float temp_k = 1.0f / ((1.0f / NTC_T0) + (1.0f / MOS_B) * logf(mos_ntc / NTC_R0));
    mos_temperature = temp_k - 273.15f;
	
    // NTC温度（adc2_values[0]）—— 上拉接法：VCC→NTC→采样点→下拉电阻→GND
    uint16_t ntc_raw = adc2_values[0];
    if (ntc_raw == 0) ntc_raw = 1;  // 防除零
    float r_ntc = PULL_DOWN_RES * (ADC_MAX_VALUE - (float)ntc_raw) / (float)ntc_raw;
    if (r_ntc < 1.0f) r_ntc = 1.0f;
    float temp_k_2 = 1.0f / ((1.0f / NTC_T0) + (1.0f / NTC_B) * logf(r_ntc / NTC_R0));
    ntc_temperature = temp_k_2 - 273.15f;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) // 仅 Init 期探测用，置位后不再清零，不代表"每帧就绪"
{
    if (hadc->Instance == ADC1) {
        adc_dma_ready = 1;  // ADC1 一轮(4通道)转换完成
    }
}
