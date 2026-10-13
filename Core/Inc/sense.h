#ifndef __SENSE_H
#define __SENSE_H

#include "stm32g4xx_hal.h"

// 硬件参数
#define INA240_GAIN          20.0f     // INA240A1 增益
#define R_SHUNT              0.001f    // 1mΩ 采样电阻
#define ADC_REF_VOLTAGE      3.3f      // ADC 参考电压
#define ADC_MAX_VALUE        4095.0f   // 12位 ADC 最大量化值 (0~4095)，电流/母线/NTC 统一使用
#define INA240_VREF          (ADC_REF_VOLTAGE / 2.0f)  // 1.65V

// 母线电压分压比
#define VOLTAGE_DIVIDER_RATIO  14.5f

// MOSFET NTC参数 
#define MOS_B       3450.0f    // B值, 单位: K

// NTC参数 
#define NTC_R0      10000.0f   // 25℃时的标称电阻, 单位: 欧姆
#define NTC_T0      298.15f    // 25℃对应的开尔文温度
#define NTC_B       3950.0f    // B值, 单位: K
#define PULL_DOWN_RES 10000.0f // 下拉电阻（NTC 上拉接法：VCC→NTC→采样点→下拉电阻→GND）, 单位: 欧姆

// ADC 原始采样值（DMA 填充，调试观察用）
extern volatile uint16_t adc_values[4];    // [0]A相电流 [1]B相电流 [2]母线电压 [3]MOSFET温度
extern volatile uint16_t adc2_values[1];   // [0]NTC 线圈温度

// 电流结果
extern volatile float current_A;   // A相电流 (A)
extern volatile float current_B;   // B相电流 (A)
extern volatile float current_C;   // C相电流 (A)
extern volatile float bus_voltage;      // 母线电压 (V)
extern volatile float mos_temperature;  // 线圈温度 (℃)
extern volatile float ntc_temperature;  // 线圈温度 (℃)

void CurrentSense_Init(void);
void CurrentSense_Update(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
float CurrentSense_Calculate(uint16_t adc_raw);

#endif