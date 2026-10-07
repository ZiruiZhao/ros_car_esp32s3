#pragma once

#include <Arduino.h>

// ============================================================================
// 差速底盘运动层 —— 「前进速度 v + 旋转速度 w」→ 左右轮速度闭环
//
//   接口形状有意与 geometry_msgs/Twist(linear.x, angular.z) 对齐 ——
//   (v, w) 就是 (linear.x, angular.z)，中间不需要再垫一层换算。
//
//   单位：归一化油门
//     v ∈ [-1, +1]   正 = 前进，负 = 后退
//     w ∈ [-1, +1]   正 = 左转（俯视逆时针），负 = 右转
//     满油门对应 CHASSIS_MAX_SPEED_CM_S
//
//   标准差速混速：  left = v - w ,  right = v + w
//   混速后按最大分量等比缩放到 ±1，保证「边走边转」给满时仍保持转向比例。
//
//   调速链路（全封装在本模块内，上层只管给 (v, w)）：
//     归一化油门 → 左右轮目标速度(cm/s) → 目标斜坡爬升
//     → 增量式 PID（50ms）→ PWM 死区 / 单步限幅 → motor_driver
// ============================================================================

static constexpr float CHASSIS_THROTTLE_MAX = 1.0f;           // 归一化油门满量程
// 满油门对应的线速度 —— 实测值，不是拍的：油门 15% 时 PWM 已到 234、
// 20% 时 PWM 顶死 255 且只跑到 28 cm/s，说明真实极速就在 30 cm/s 附近。
// 定得比极速高，多出来的油门全是白给（目标达不到，PID 只会把 PWM 顶到饱和）。
static constexpr float CHASSIS_MAX_SPEED_CM_S = 30.0f;
static constexpr uint32_t CHASSIS_CONTROL_PERIOD_US = 50000U; // 闭环周期 50ms

// 底盘状态快照（供串口上报 / 调参观察）
struct ChassisState {
  int32_t enc_left;
  int32_t enc_right;
  float speed_left_cm_s;
  float speed_right_cm_s;
  float target_left_cm_s;
  float target_right_cm_s;
  int pwm_left;
  int pwm_right;
};

void chassis_init();                  // 电机 + 编码器 + PID 初始化
void chassis_drive(float v, float w); // 设定期望运动（归一化油门），下一周期生效
void chassis_stop();                  // 立即停车：清目标 + 清 PID + PWM 归零
void chassis_update();                // 闭环周期调用：读编码器 → PID → 输出 PWM

ChassisState chassis_get_state();                              // 取状态快照
void chassis_set_pid(float kp, float ki, float kd);            // 在线整定 PID
void chassis_get_pid(float *kp, float *ki, float *kd);         // 读当前 PID 参数

// ---- 运动基元（瞬时动作，持续时长由调用方用定时器控制）----
inline void chassis_forward(float throttle) { chassis_drive(+throttle, 0.0f); }
inline void chassis_backward(float throttle) { chassis_drive(-throttle, 0.0f); }
inline void chassis_spin_left(float throttle) { chassis_drive(0.0f, +throttle); }
inline void chassis_spin_right(float throttle) { chassis_drive(0.0f, -throttle); }
