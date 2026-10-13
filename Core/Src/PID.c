#include "pid.h"
#include <math.h>

/**
 * @brief 初始化PID控制器
 * @param pid    句柄指针
 * @param params 参数结构体指针（包含Kp, Ki, Kd及限幅值）
 */
void PID_Init(PID_Handle_t *pid, PID_Params_t *params)
{
    // 拷贝参数
	pid->params = *params;	
    
    // 重置状态
    PID_Reset(pid);
}

//重置积分和误差（用于故障恢复或启动时清空历史）
void PID_Reset(PID_Handle_t *pid)
{
    pid->integral = 0.0f;
	pid->prev_feedback = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 执行PID计算（位置式）
 * @param target   目标值（如目标速度、目标Iq）
 * @param feedback 反馈值（如实际速度、实际Iq）
 * @return 计算后的输出值（已限幅）
 */
float PID_Calculate(PID_Handle_t *pid, float target, float feedback, float dt)
{
    // dt 合法性保护：防止除零/NaN 雪崩（上游 control_dt 异常时）
    if (dt <= 1e-6f) dt = 1e-6f;

    // 计算误差
    float error = target - feedback;

    // 比例项 (P)
    float P_out = pid->params.Kp * error;

    // 微分项 (D)：微分先行（对 feedback 微分，避免设定值阶跃引起微分冲击）
    float D_out = pid->params.Kd * (feedback - pid->prev_feedback) / dt;
    pid->prev_feedback = feedback;

    // 先更新积分（带积分限幅）
    pid->integral += error * dt;
    if (pid->integral > pid->params.int_max) pid->integral = pid->params.int_max;
    if (pid->integral < pid->params.int_min) pid->integral = pid->params.int_min;
    float I_out = pid->params.Ki * pid->integral;

    // 计算总输出（含本次新积分）
    float pre_output = P_out + I_out + D_out;

    // 抗积分饱和（clamping）：若输出已饱和且与积分作用方向同向，则回退本次积分
    if ((pre_output > pid->params.out_max && error > 0.0f) ||
        (pre_output < pid->params.out_min && error < 0.0f)) {
        pid->integral -= error * dt;   // 冻结积分，撤销本次累积
        if (pid->integral > pid->params.int_max) pid->integral = pid->params.int_max;
        if (pid->integral < pid->params.int_min) pid->integral = pid->params.int_min;
        I_out = pid->params.Ki * pid->integral;
        pre_output = P_out + I_out + D_out;
    }

    // 输出限幅（至关重要）
    if (pre_output > pid->params.out_max) pre_output = pid->params.out_max;
    if (pre_output < pid->params.out_min) pre_output = pid->params.out_min;

    pid->output = pre_output;

    return pid->output;
}

//运行时动态修改参数（可选）
void PID_SetParams(PID_Handle_t *pid, PID_Params_t *params)
{
    pid->params.Kp = params->Kp;
    pid->params.Ki = params->Ki;
    pid->params.Kd = params->Kd;
}