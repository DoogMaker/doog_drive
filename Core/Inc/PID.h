#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H


#include "main.h"

// PID 参数结构体
typedef struct {
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Kd;               // 微分系数
    float out_min;          // 输出下限（限幅）
    float out_max;          // 输出上限（限幅）
    float int_min;          // 积分项下限（抗积分饱和）
    float int_max;          // 积分项上限（抗积分饱和）
} PID_Params_t;

// PID 状态结构体
typedef struct {
    PID_Params_t params;    // 参数副本
    float integral;         // 积分累积值
    float prev_feedback;		// 微分先行（上一帧反馈值）
    float output;           // 当前输出值
} PID_Handle_t;

// 外部调用接口
void PID_Init(PID_Handle_t *pid, PID_Params_t *params);
void PID_Reset(PID_Handle_t *pid);
float PID_Calculate(PID_Handle_t *pid, float target, float feedback, float dt);
void PID_SetParams(PID_Handle_t *pid, PID_Params_t *params);

#endif