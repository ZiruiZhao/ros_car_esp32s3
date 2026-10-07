#include "speed_pid.h"

// PID 初始化（只设置参数，不清零状态）
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max, float min)
{
    // 1. 检查限幅有效性（防止 max==min 导致输出恒为常数）
    if (max == min) return;

    // 2. 保存参数
    pid->P = Kp;
    pid->I = Ki;
    pid->D = Kd;
    pid->OutputMax = max;
    pid->OutputMin = min;
}

// 重置 PID 状态（不清空参数，用于停车/换向/切换目标）
void PID_Clear(PID *pid)
{
    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->Output = 0.0f;
}

// 增量式 PID 计算（电机速度专用）
float PID_IncPIDCal(PID *pid, float NowValue, float AimValue)
{
    float iError;      // 当前误差
    float increment;   // 增量值

    // 1. 计算当前误差
    iError = AimValue - NowValue;

    // 2. 增量式 PID 公式（标准工业版）
    increment = pid->P * (iError - pid->LastError)                         // 比例项
              + pid->I * iError                                            // 积分项
              + pid->D * (iError - 2 * pid->LastError + pid->PrevError);   // 微分项

    // 3. 输出 = 上次输出 + 增量
    pid->Output += increment;

    // 4. 输出限幅（防 PWM 饱和）
    if (pid->Output > pid->OutputMax) pid->Output = pid->OutputMax;
    if (pid->Output < pid->OutputMin) pid->Output = pid->OutputMin;

    // 5. 滚动更新历史误差
    pid->PrevError = pid->LastError;
    pid->LastError = iError;

    return pid->Output;
}
