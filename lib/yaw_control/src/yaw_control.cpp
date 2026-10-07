#include "yaw_control.h"

#include <math.h>

#include <attitude.h>
#include <chassis.h>

// 调用周期（秒），与 YAW_CONTROL_PERIOD_US 对应。只在 dt 读取异常时当兜底值用。
static constexpr float YAW_CONTROL_PERIOD_S = 0.05f;

// dt 上限：loop 卡顿时别让积分项一次跳一大截
static constexpr float YAW_DT_MAX_S = 0.2f;

// ---- 模块状态 ----
static bool g_active = false;
static float g_target_deg = 0.0f;  // 本轮目标（绝对角）
static float g_error_deg = 0.0f;   // 最近一次误差；本轮结束后就是最终残差
static float g_w = 0.0f;           // 最近一次输出
static float g_integral = 0.0f;    // 积分累积量
static uint32_t g_start_ms = 0;
static uint32_t g_elapsed_ms = 0; // 本轮已耗时；结束后冻结在最终值
static uint32_t g_last_us = 0;

// PID 参数（可在运行期整定）
static float g_kp = YAW_KP_DEFAULT;
static float g_ki = YAW_KI_DEFAULT;
static float g_kd = YAW_KD_DEFAULT;

// 卡死检测基准：上一次结算时的角度和时刻
static float g_stall_ref_yaw = 0.0f;
static uint32_t g_stall_ref_ms = 0;

static float clampf(float v, float lo, float hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

void yaw_control_init()
{
  g_active = false;
  g_target_deg = 0.0f;
  g_error_deg = 0.0f;
  g_w = 0.0f;
  g_integral = 0.0f;
  g_start_ms = 0;
  g_elapsed_ms = 0;
  g_last_us = micros();
  g_stall_ref_yaw = 0.0f;
  g_stall_ref_ms = 0;

  g_kp = YAW_KP_DEFAULT;
  g_ki = YAW_KI_DEFAULT;
  g_kd = YAW_KD_DEFAULT;
}

bool yaw_control_start(float delta_deg)
{
  const Attitude a = attitude_get();
  if (!a.valid) {
    // IMU 还没攒够数据，此刻的 yaw 没意义。宁可不启动，也不要拿 0 当基准。
    return false;
  }

  // 夹到上限：转 250 圈没有意义，而且必然中途没电或者撞墙
  delta_deg = clampf(delta_deg, -YAW_TARGET_MAX_DEG, YAW_TARGET_MAX_DEG);

  // 目标是「现在朝向 + delta」，相对角。
  // 不提供「转到绝对 90°」是因为 yaw 的零点随 attitude_zero_yaw() 和静止修正
  // 的累积而变，绝对角没有物理意义；「转 90°」才是自然语义。
  g_target_deg = a.yaw_deg + delta_deg;

  g_integral = 0.0f;  // 新目标，上一轮积的东西全作废
  g_error_deg = delta_deg;
  g_w = 0.0f;

  const uint32_t now_ms = millis();
  g_start_ms = now_ms;
  g_elapsed_ms = 0;
  g_last_us = micros();

  g_stall_ref_yaw = a.yaw_deg;
  g_stall_ref_ms = now_ms;

  g_active = true;
  return true;
}

YawResult yaw_control_update()
{
  if (!g_active) {
    return YawResult::IDLE;
  }

  // ---- dt：用实际经过时间，而不是写死 50ms ----
  const uint32_t now_us = micros();
  float dt = static_cast<float>(now_us - g_last_us) * 1e-6f;
  g_last_us = now_us;
  if (dt > YAW_DT_MAX_S) {
    dt = YAW_DT_MAX_S;
  }
  if (dt <= 0.0f) {
    dt = YAW_CONTROL_PERIOD_S;
  }

  const uint32_t now_ms = millis();
  g_elapsed_ms = now_ms - g_start_ms;

  const Attitude a = attitude_get();
  if (!a.valid) {
    // 姿态数据失效（正常不会发生，valid 一旦为真就不再回落）。
    // 但真发生了就绝不能拿一个无效的角度继续闭环 —— 立刻停车。
    chassis_stop();
    g_active = false;
    return YawResult::ABORTED;
  }

  g_error_deg = g_target_deg - a.yaw_deg;

  // ---- ① 容差：到位 ----
  if (fabsf(g_error_deg) <= YAW_TOLERANCE_DEG) {
    chassis_stop();
    g_active = false;
    return YawResult::ARRIVED;
  }

  // ---- ② 超时 ----
  if (g_elapsed_ms >= YAW_TIMEOUT_MS) {
    chassis_stop();
    g_active = false;
    return YawResult::TIMEOUT;
  }

  // ---- PID ----
  const float p_term = g_kp * g_error_deg;
  // D 项：d(误差)/dt = -dyaw/dt，而 gyro_z 就是 dyaw/dt 的直接测量。
  // 符号因此是负的 —— 「正在往目标方向转」时，这一项在往回拉，起阻尼作用。
  const float d_term = -g_kd * a.gyro_z_dps;
  const float pd_term = p_term + d_term;

  // 积分：只在容差外积（进了容差上面就直接到位了，用不着）。
  //
  // 抗积分饱和用**条件积分**：输出已经饱和、且误差还在往饱和的同一个方向推
  // → 这一拍不积。反过来，误差方向与饱和方向相反（积分能帮输出退饱和）时
  // 照积不误，否则会卡在饱和里出不来。
  //
  // 诚实记录：仿真实测下来，在 YAW_INTEGRAL_MAX=5 这个限幅下，条件积分相比
  // 「只夹幅值」**没有可测量的收益**（两者过冲完全一致）。保留它是因为它是
  // 标准做法、只有 4 行，且一旦有人把 I_MAX 或 W_MAX 调大，只夹幅值就会失效
  // —— 那时候这里能兜住。不要因为它现在看不出效果就删掉。
  const float w_unsat = pd_term + g_ki * g_integral;
  const bool pushing_into_limit =
      (fabsf(w_unsat) > YAW_W_MAX) && ((w_unsat > 0.0f) == (g_error_deg > 0.0f));

  if (!pushing_into_limit) {
    g_integral += g_error_deg * dt;
    g_integral = clampf(g_integral, -YAW_INTEGRAL_MAX, YAW_INTEGRAL_MAX);
  }

  float w = pd_term + g_ki * g_integral;
  w = clampf(w, -YAW_W_MAX, YAW_W_MAX);
  g_w = w;

  // ---- ③ 卡死：使劲了，但车没动 ----
  // 放在算完 w 之后，因为要靠这一拍的输出判断「我到底有没有在使劲」。
  if (now_ms - g_stall_ref_ms >= YAW_STALL_WINDOW_MS) {
    const float moved = fabsf(a.yaw_deg - g_stall_ref_yaw);
    if (moved < YAW_STALL_MIN_DEG && fabsf(w) > YAW_STALL_MIN_W) {
      chassis_stop();
      g_active = false;
      return YawResult::STALLED;
    }
    // 没卡住：把基准挪到当前，进入下一个观察窗口
    g_stall_ref_yaw = a.yaw_deg;
    g_stall_ref_ms = now_ms;
  }

  // ---- 只转不前进 ----
  // v=0 → 左右轮等速反向，绕车体中心原地转。w 的符号约定（正=左转）
  // 与 attitude 的 yaw 正方向一致，这里不需要任何符号转换。
  chassis_drive(0.0f, w);
  return YawResult::RUNNING;
}

void yaw_control_abort()
{
  if (!g_active) {
    return;
  }
  chassis_stop();
  g_active = false;
}

bool yaw_control_busy()
{
  return g_active;
}

void yaw_control_set_pid(float kp, float ki, float kd)
{
  g_kp = kp;
  g_ki = ki;
  g_kd = kd;
}

void yaw_control_get_pid(float *kp, float *ki, float *kd)
{
  if (kp != nullptr) {
    *kp = g_kp;
  }
  if (ki != nullptr) {
    *ki = g_ki;
  }
  if (kd != nullptr) {
    *kd = g_kd;
  }
}

float yaw_control_get_target_deg()
{
  return g_target_deg;
}

float yaw_control_get_error_deg()
{
  return g_error_deg;
}

float yaw_control_get_w()
{
  return g_w;
}

uint32_t yaw_control_get_elapsed_ms()
{
  return g_elapsed_ms;
}
