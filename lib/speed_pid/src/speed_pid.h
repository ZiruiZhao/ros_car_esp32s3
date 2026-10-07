#pragma once

#include <Arduino.h>

// ============================================================================
// 增量式 PID（电机速度环专用）
//
// 增量式与位置式区别：
//   u(k) = u(k-1) + P·(e(k)-e(k-1)) + I·e(k) + D·(e(k)-2e(k-1)+e(k-2))
// 优点：输出天然平滑、无需累加全部历史误差、目标跳变不打冲击，适合电机调速。
// ============================================================================

typedef struct
{
    // PID 参数
    float P;
    float I;
    float D;

    // 输出限幅（防止 PWM 饱和导致积分饱和/电机过载）
    float OutputMax;
    float OutputMin;

    // 历史误差（增量式需要前两拍误差）
    float LastError;
    float PrevError;

    // 当前输出
    float Output;
} PID;

// 初始化参数（只设置参数与限幅，不清零状态）
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max, float min);

// 清零历史误差与输出（停车 / 换向 / 改目标时必须调用）
void PID_Clear(PID *pid);

// 增量式 PID 计算（NowValue=实测速度，AimValue=目标速度，返回控制量）
float PID_IncPIDCal(PID *pid, float NowValue, float AimValue);
