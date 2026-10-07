#pragma once

#include <Arduino.h>
#include <motor_driver.h>

// ============================================================================
// 轮速编码器 —— AB 双相正交编码（霍尔式，减速电机尾部引出）
//
//   A/B 两相信号相位相差 90°。两路都接 CHANGE 双边沿中断，配合状态机查表
//   实现 4 倍频计数：每转计数 = 磁环线数 × 减速比 × 4。
//   正转 +1、反转 -1，抖动（如单相回弹）不产生累计误差。
//
//   引脚定义（按整车接线图填写）：
//     左轮编码器 = A+/A-（GPIO4/5），右轮编码器 = B+/B-（GPIO6/7）
// ============================================================================

static constexpr uint8_t WHEEL_ENC_L_A_PIN = 4; // GPIO4 → 左轮编码器 A 相（A+）
static constexpr uint8_t WHEEL_ENC_L_B_PIN = 5; // GPIO5 → 左轮编码器 B 相（A-）
static constexpr uint8_t WHEEL_ENC_R_A_PIN = 6; // GPIO6 → 右轮编码器 A 相（B+）
static constexpr uint8_t WHEEL_ENC_R_B_PIN = 7; // GPIO7 → 右轮编码器 B 相（B-）

// 计数符号修正：跟随电机方向反相标志，保证「正转指令时编码器计数为正」。
// 自检时若某轮正转阶段 enc 为负，把对应项反过来即可。
static constexpr int8_t WHEEL_ENC_L_SIGN = LEFT_MOTOR_INVERT_DIR ? -1 : 1;
static constexpr int8_t WHEEL_ENC_R_SIGN = RIGHT_MOTOR_INVERT_DIR ? -1 : 1;

// ---- 轮子几何参数 ----
//
// 这两个是**标称值，不是卡尺量出来的**。它们只以乘积的形式进入下面的换算常数，
// 所以一个系数就能把两者的误差一起修掉 —— 那就是里程计的 ODOM_DIST_SCALE，
// 标定它比「量轮径 + 手数 10 圈计数」快得多（方法见 odometry.h 顶部）。
// 想从源头改也完全可以：卡尺量轮径、转 10 圈读总计数，直接填到这里。
static constexpr float WHEEL_DIAMETER_CM = 6.5f;       // 轮径（cm）
static constexpr float WHEEL_COUNTS_PER_REV = 1750.0f; // 每转计数（A/B 双相 4 倍频之后）

// 每个计数对应的行驶距离（cm）
static constexpr float WHEEL_CM_PER_COUNT =
    (WHEEL_DIAMETER_CM * PI) / WHEEL_COUNTS_PER_REV;

// 累计计数 → 距离（cm）
inline float wheel_encoder_counts_to_distance_cm(int32_t counts)
{
  return static_cast<float>(counts) * WHEEL_CM_PER_COUNT;
}

// 计数增量 + 时间间隔 → 速度（cm/s）
inline float wheel_encoder_delta_counts_to_speed_cm_s(int32_t delta_counts, float dt_s)
{
  return static_cast<float>(delta_counts) * WHEEL_CM_PER_COUNT / dt_s;
}

// 速度计算初始化（通常在 wheel_encoder_init 后调用一次）
void wheel_encoder_speed_init();

// 周期测速：到达采样周期（默认 100ms）才更新并返回 true，否则返回 false。
// PID 用 50ms 周期时传 sample_us = 50000。
bool wheel_encoder_get_speed_cm_s(float *left_cm_s,
                                  float *right_cm_s,
                                  uint32_t sample_us = 100000U);

// 初始化编码器（enable_pullups=true 时使用内部上拉，适配开漏输出的霍尔编码器）
void wheel_encoder_init(bool enable_pullups = true);

// 读取左右轮累计总计数（有符号，正负代表方向）
void wheel_encoder_get_counts(int32_t *left, int32_t *right);
int32_t wheel_encoder_get_left();
int32_t wheel_encoder_get_right();

// 清零计数 / 读取后清零（用于按周期计算增量里程）
void wheel_encoder_reset();
void wheel_encoder_get_and_reset(int32_t *left, int32_t *right);
