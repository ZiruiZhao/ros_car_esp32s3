#include "odometry.h"

#include <math.h>

#include <attitude.h>
#include <chassis.h>
#include <wheel_encoder.h>

// 度数 ↔ 弧度。名字带前缀是必须的 —— Arduino.h 里有同名的 DEG_TO_RAD / RAD_TO_DEG
// 宏，直接叫那个名字会被预处理器替换掉，编译直接报错（步骤 5 踩过）。
// 末尾坚持加 f：ESP32-S3 的 double 是软件模拟的，比硬件 float 慢一个数量级。
static constexpr float ODOM_DEG_TO_RAD = 0.01745329252f;

// ---- 模块状态 ----
static OdomState g_odom = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

// 上一拍的原始输入。里程计算的是**增量**，所以必须留着上一次的值做差。
static int32_t g_last_enc_l = 0;
static int32_t g_last_enc_r = 0;
static float g_last_yaw_deg = 0.0f;

static float g_dist_scale = ODOM_DIST_SCALE_DEFAULT;
static uint32_t g_last_us = 0;

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

void odom_init()
{
  g_odom.x_cm = 0.0f;
  g_odom.y_cm = 0.0f;
  g_odom.theta_deg = 0.0f;
  g_odom.dist_cm = 0.0f;
  g_odom.v_cm_s = 0.0f;
  g_odom.w_dps = 0.0f;
  g_odom.valid = false;

  g_last_enc_l = 0;
  g_last_enc_r = 0;
  g_last_yaw_deg = 0.0f;

  g_dist_scale = ODOM_DIST_SCALE_DEFAULT;
  g_last_us = micros();
}

bool odom_update()
{
  // ---- 节拍门限 ----
  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - g_last_us;
  if (elapsed_us < ODOM_PERIOD_US) {
    return false;
  }
  g_last_us = now_us;

  const Attitude a = attitude_get();
  const ChassisState s = chassis_get_state();

  // ---- 首帧：只记基准，不积分 ----
  // 没有「上一拍」就算不出增量。这一拍的作用是把起点对齐，从下一拍开始才真正推进。
  // （和 attitude 首帧用加速度计定初值是同一个套路）
  if (!g_odom.valid) {
    if (!a.valid) {
      return false;  // 姿态还没就绪，等它
    }
    g_last_enc_l = s.enc_left;
    g_last_enc_r = s.enc_right;
    g_last_yaw_deg = a.yaw_deg;
    g_odom.valid = true;
    return false;
  }

  // ---- 本拍增量 ----
  const float d_left_cm =
      wheel_encoder_counts_to_distance_cm(s.enc_left - g_last_enc_l) * g_dist_scale;
  const float d_right_cm =
      wheel_encoder_counts_to_distance_cm(s.enc_right - g_last_enc_r) * g_dist_scale;
  g_last_enc_l = s.enc_left;
  g_last_enc_r = s.enc_right;

  // 车体中心位移 = 左右轮的平均。原地转时两轮反向、平均值抵消为 0，位置不动 —— 对。
  const float d_center_cm = 0.5f * (d_left_cm + d_right_cm);

  // 航向增量取自 IMU 的 yaw 增量（为什么不用编码器差速，见头文件）。
  // 用增量而不是绝对值，`z` 指令把 yaw 归零时里程计不受任何影响。
  const float d_yaw_deg = a.yaw_deg - g_last_yaw_deg;
  g_last_yaw_deg = a.yaw_deg;

  // ---- 中点积分（见头文件：这不是精度优化，是「走圆弧会不会系统性偏」的区别）----
  const float theta_old_deg = g_odom.theta_deg;
  const float theta_mid_rad = (theta_old_deg + 0.5f * d_yaw_deg) * ODOM_DEG_TO_RAD;

  g_odom.x_cm += d_center_cm * cosf(theta_mid_rad);
  g_odom.y_cm += d_center_cm * sinf(theta_mid_rad);
  g_odom.theta_deg = theta_old_deg + d_yaw_deg;

  // 路程用绝对值累加：倒车是「走了路」，不该让累计路程变小。
  // 标定刻度系数时看的是这个量，必须是单调的。
  g_odom.dist_cm += fabsf(d_center_cm);

  // 速度估计。dt 有节拍门限保底（≥50ms），不会除到 0。
  const float dt_s = static_cast<float>(elapsed_us) * 1e-6f;
  g_odom.v_cm_s = d_center_cm / dt_s;
  g_odom.w_dps = d_yaw_deg / dt_s;

  return true;
}

OdomState odom_get()
{
  return g_odom;
}

void odom_reset()
{
  g_odom.x_cm = 0.0f;
  g_odom.y_cm = 0.0f;
  g_odom.theta_deg = 0.0f;
  g_odom.dist_cm = 0.0f;
  g_odom.v_cm_s = 0.0f;
  g_odom.w_dps = 0.0f;

  // ⚠ 关键：增量基准必须跟着挪到当前。
  // 只把 x/y/θ 清 0、不动基准的话，下一次更新会把「reset 之前累积的那段位移」
  // 当成新的一步算进去 —— 车明明没动，位置却凭空跳一下。
  // 这是里程计最容易踩的一个坑，且现象很迷惑（复位后第一帧就偏）。
  const ChassisState s = chassis_get_state();
  g_last_enc_l = s.enc_left;
  g_last_enc_r = s.enc_right;
  g_last_yaw_deg = attitude_get().yaw_deg;
}

void odom_set_dist_scale(float scale)
{
  g_dist_scale = clampf(scale, ODOM_DIST_SCALE_MIN, ODOM_DIST_SCALE_MAX);
}

float odom_get_dist_scale()
{
  return g_dist_scale;
}

bool odom_is_ready()
{
  return g_odom.valid;
}
