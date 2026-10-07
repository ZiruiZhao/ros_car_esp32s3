#include <Arduino.h>
#include <math.h>

#include "chassis.h"
#include <motor_driver.h>
#include <speed_pid.h>
#include <wheel_encoder.h>

// ---- PID 与输出整形参数 ----
static constexpr float PID_KP_DEFAULT = 2.5f;
static constexpr float PID_KI_DEFAULT = 0.65f;
static constexpr float PID_KD_DEFAULT = 0.5f;
static constexpr float PID_OUTPUT_MAX = 255.0f; // 对应 PWM 满量程
static constexpr float PID_OUTPUT_MIN = -255.0f;

static constexpr int PWM_DEADBAND = 5;              // 死区：|PWM| 低于此值直接给 0
static constexpr int PWM_STEP_MAX = 12;             // 单周期 PWM 最大变化量，防电流冲击
static constexpr float TARGET_STEP_MAX_CM_S = 8.0f; // 单周期目标速度最大爬升

// ---- 模块状态 ----
static PID g_left_pid;
static PID g_right_pid;

static float g_target_left_cm_s = 0.0f; // chassis_drive 设定的最终目标
static float g_target_right_cm_s = 0.0f;
static float g_ramp_left_cm_s = 0.0f;   // 斜坡爬升中的当前目标
static float g_ramp_right_cm_s = 0.0f;

static float g_speed_left_cm_s = 0.0f;  // 编码器实测速度
static float g_speed_right_cm_s = 0.0f;
static int g_pwm_left = 0;              // 当前 PWM 输出
static int g_pwm_right = 0;

static inline float clamp_throttle(float x)
{
  if (x > CHASSIS_THROTTLE_MAX) return CHASSIS_THROTTLE_MAX;
  if (x < -CHASSIS_THROTTLE_MAX) return -CHASSIS_THROTTLE_MAX;
  return x;
}

// 按步长限制变化量（斜坡 / 单步限幅共用）
static inline float step_limit(float current, float target, float max_step)
{
  const float delta = target - current;
  if (delta > max_step) return current + max_step;
  if (delta < -max_step) return current - max_step;
  return target;
}

static inline int step_limit_int(int current, int target, int max_step)
{
  const int delta = target - current;
  if (delta > max_step) return current + max_step;
  if (delta < -max_step) return current - max_step;
  return target;
}

// 单轮闭环：目标速度 → PWM
static int update_one_wheel(PID *pid, float target_cm_s, float actual_cm_s, int current_pwm)
{
  // 目标为 0：不进 PID，直接断电。否则累积输出会让轮子停不干净（蠕动）
  if (fabsf(target_cm_s) < 0.01f) {
    PID_Clear(pid);
    return 0;
  }

  const int raw = static_cast<int>(lroundf(PID_IncPIDCal(pid, actual_cm_s, target_cm_s)));

  // 死区：占空比太低时电机不转，只会发热，直接归零
  const int shaped = (abs(raw) < PWM_DEADBAND) ? 0 : raw;

  // 单步限幅：目标突变时限制电流冲击
  return step_limit_int(current_pwm, shaped, PWM_STEP_MAX);
}

void chassis_init()
{
  motor_init();
  wheel_encoder_init();
  wheel_encoder_speed_init();

  PID_Init(&g_left_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, PID_OUTPUT_MAX, PID_OUTPUT_MIN);
  PID_Init(&g_right_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT, PID_OUTPUT_MAX, PID_OUTPUT_MIN);

  chassis_stop();
}

void chassis_drive(float v, float w)
{
  v = clamp_throttle(v);
  w = clamp_throttle(w);

  float left = v - w;
  float right = v + w;

  // 混速后会超出 ±1（例如 v=1,w=1 → right=2）。按最大分量等比缩放，
  // 既不会把目标顶到饱和后丢掉差速，也保持了 v 与 w 的比例（转向半径不变）。
  const float peak = fmaxf(fabsf(left), fabsf(right));
  if (peak > CHASSIS_THROTTLE_MAX) {
    const float scale = CHASSIS_THROTTLE_MAX / peak;
    left *= scale;
    right *= scale;
  }

  const float new_left = left * CHASSIS_MAX_SPEED_CM_S;
  const float new_right = right * CHASSIS_MAX_SPEED_CM_S;

  // 目标换向（含归零）时清 PID：换向前累积的输出和误差历史都无意义，
  // 留着会让轮子在过零瞬间窜一下
  if ((new_left > 0.0f) != (g_target_left_cm_s > 0.0f)) PID_Clear(&g_left_pid);
  if ((new_right > 0.0f) != (g_target_right_cm_s > 0.0f)) PID_Clear(&g_right_pid);

  g_target_left_cm_s = new_left;
  g_target_right_cm_s = new_right;
}

void chassis_stop()
{
  g_target_left_cm_s = 0.0f;
  g_target_right_cm_s = 0.0f;
  g_ramp_left_cm_s = 0.0f;
  g_ramp_right_cm_s = 0.0f;

  PID_Clear(&g_left_pid);
  PID_Clear(&g_right_pid);

  g_pwm_left = 0;
  g_pwm_right = 0;

  // 急停不走斜坡，直接断电
  set_left_motor_pwm(0);
  set_right_motor_pwm(0);
}

void chassis_update()
{
  // 测速：未到采样周期时函数返回 false 且不写输出，沿用上一次的值
  float speed_l = g_speed_left_cm_s;
  float speed_r = g_speed_right_cm_s;
  if (wheel_encoder_get_speed_cm_s(&speed_l, &speed_r, CHASSIS_CONTROL_PERIOD_US)) {
    g_speed_left_cm_s = speed_l;
    g_speed_right_cm_s = speed_r;
  }

  // 目标斜坡爬升，避免突变时拉冲击电流
  g_ramp_left_cm_s = step_limit(g_ramp_left_cm_s, g_target_left_cm_s, TARGET_STEP_MAX_CM_S);
  g_ramp_right_cm_s = step_limit(g_ramp_right_cm_s, g_target_right_cm_s, TARGET_STEP_MAX_CM_S);

  g_pwm_left = update_one_wheel(&g_left_pid, g_ramp_left_cm_s, g_speed_left_cm_s, g_pwm_left);
  g_pwm_right = update_one_wheel(&g_right_pid, g_ramp_right_cm_s, g_speed_right_cm_s, g_pwm_right);

  set_left_motor_pwm(g_pwm_left);
  set_right_motor_pwm(g_pwm_right);
}

ChassisState chassis_get_state()
{
  ChassisState state;
  wheel_encoder_get_counts(&state.enc_left, &state.enc_right);

  state.speed_left_cm_s = g_speed_left_cm_s;
  state.speed_right_cm_s = g_speed_right_cm_s;
  state.target_left_cm_s = g_ramp_left_cm_s; // 上报爬升中的实际目标，便于观察跟随
  state.target_right_cm_s = g_ramp_right_cm_s;
  state.pwm_left = g_pwm_left;
  state.pwm_right = g_pwm_right;
  return state;
}

void chassis_set_pid(float kp, float ki, float kd)
{
  g_left_pid.P = kp;
  g_left_pid.I = ki;
  g_left_pid.D = kd;
  g_right_pid.P = kp;
  g_right_pid.I = ki;
  g_right_pid.D = kd;
}

void chassis_get_pid(float *kp, float *ki, float *kd)
{
  if (kp != nullptr) *kp = g_left_pid.P;
  if (ki != nullptr) *ki = g_left_pid.I;
  if (kd != nullptr) *kd = g_left_pid.D;
}
