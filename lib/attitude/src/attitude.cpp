#include "attitude.h"

#include <math.h>

// 弧度转角度。名字必须带 ATTITUDE_ 前缀 —— Arduino.h 里已经有个同名的
// RAD_TO_DEG 宏，直接叫 RAD_TO_DEG 会被预处理器替换掉，编译直接报错。
// 另外这里坚持用 float（末尾的 f）：ESP32-S3 的 double 是软件模拟的，
// 比硬件 float 慢一个数量级，100Hz 的循环里没必要为那点精度付这个代价。
static constexpr float ATTITUDE_RAD_TO_DEG = 57.29577951f;

// 模块内部状态。全静态 = 只有这一个实例，不需要面向对象那套
static Attitude g_att = {0.0f, 0.0f, 0.0f, false, 0.0f};
static float g_alpha = ATTITUDE_ALPHA_DEFAULT;
static uint32_t g_last_us = 0;         // 上一拍的时刻（节拍门限基准）
static float g_last_dt_s = 0.0f;       // 上一拍实际用的 dt

// 实测频率统计：每满 1 秒结算一次
static uint32_t g_rate_count = 0;
static uint32_t g_rate_window_us = 0;
static uint16_t g_rate_hz = 0;

// 静止修正状态
static bool g_still = false;
static uint32_t g_still_us = 0; // 连续静止了多久（任何一次「在动」都清零）

void attitude_init()
{
  g_att.roll_deg = 0.0f;
  g_att.pitch_deg = 0.0f;
  g_att.yaw_deg = 0.0f;
  g_att.valid = false;

  g_alpha = ATTITUDE_ALPHA_DEFAULT;
  g_last_us = micros();
  g_last_dt_s = 0.0f;

  g_rate_count = 0;
  g_rate_window_us = g_last_us;
  g_rate_hz = 0;

  g_still = false;
  g_still_us = 0;
}

bool attitude_update(bool vehicle_stopped)
{
  // ---- 节拍门限：没到 10ms 直接走人，连 I2C 都不碰 ----
  const uint32_t now_us = micros();
  const uint32_t elapsed_us = now_us - g_last_us;
  if (elapsed_us < ATTITUDE_PERIOD_US) {
    return false;
  }
  g_last_us = now_us;

  // ---- 读一帧六轴数据 ----
  ImuSample s;
  if (!imu_read(&s)) {
    return false;
  }

  // dt 正常是 10ms。loop 卡顿时 elapsed 会变大，夹到上限：
  // 陀螺积分靠 dt 乘以角速度，dt 突然变成 100ms 会让角度一次跳一大截。
  float dt = static_cast<float>(elapsed_us) * 1e-6f;
  if (dt > ATTITUDE_DT_MAX_S) {
    dt = ATTITUDE_DT_MAX_S;
  }
  g_last_dt_s = dt;

  // 顺便把这一拍的 Z 轴角速度带出去（转向环的阻尼项要用，见 attitude.h）。
  // 放在首帧判断之前，保证任何一个有效的 Attitude 快照里它都是新鲜的。
  g_att.gyro_z_dps = s.gyro_z_dps;

  // ---- 加速度计：由重力方向算出绝对角度 ----
  const float ax = s.accel_x_g;
  const float ay = s.accel_y_g;
  const float az = s.accel_z_g;

  // roll：车身侧倾时重力在 Y/Z 上分量变化
  const float roll_acc_deg = atan2f(ay, az) * ATTITUDE_RAD_TO_DEG;

  // pitch：分母用 sqrt(ay²+az²) 而不是直接 az —— 大俯仰角时 az 趋近 0，
  // 直接除会失稳。开方后分母始终是「水平面外的合分量」，不会退化。
  const float pitch_acc_deg =
      atan2f(ax, sqrtf(ay * ay + az * az)) * ATTITUDE_RAD_TO_DEG;

  // |a| 偏离 1g 太多说明车体正在加减速或受撞击，此刻的「重力方向」是假的。
  const float a_mag = sqrtf(ax * ax + ay * ay + az * az);
  const bool accel_ok =
      (a_mag > ATTITUDE_ACCEL_MAG_MIN) && (a_mag < ATTITUDE_ACCEL_MAG_MAX);

  // ---- 首帧：直接用加速度计定初值，省得从 0 慢慢爬上来 ----
  if (!g_att.valid) {
    if (!accel_ok) {
      return false;  // 车还在动，等它稳下来再定基准
    }
    g_att.roll_deg = roll_acc_deg;
    g_att.pitch_deg = pitch_acc_deg;
    g_att.yaw_deg = 0.0f;
    g_att.valid = true;
    return false;  // 本拍只是定初值，不算一次解算
  }

  // ---- 陀螺积分（预测）----
  //   pitch 这里要取负：陀螺仪 +Y 为正表示车头下俯，而本工程定义抬头为正，
  //   两者反向。roll 和 yaw 不需要，符号天然一致。
  const float roll_gyro_deg = g_att.roll_deg + s.gyro_x_dps * dt;
  const float pitch_gyro_deg = g_att.pitch_deg - s.gyro_y_dps * dt;

  // ---- 互补滤波（校正）----
  if (accel_ok) {
    g_att.roll_deg =
        g_alpha * roll_gyro_deg + (1.0f - g_alpha) * roll_acc_deg;
    g_att.pitch_deg =
        g_alpha * pitch_gyro_deg + (1.0f - g_alpha) * pitch_acc_deg;
  } else {
    // 加速度计不可信：这一拍只信陀螺，等 |a| 回到 1g 附近再拉回来
    g_att.roll_deg = roll_gyro_deg;
    g_att.pitch_deg = pitch_gyro_deg;
  }

  // ---- 静止判定：车轮没转（调用方告诉）+ 陀螺安静（没人用手转它）----
  const float gyro_mag = sqrtf(s.gyro_x_dps * s.gyro_x_dps +
                               s.gyro_y_dps * s.gyro_y_dps +
                               s.gyro_z_dps * s.gyro_z_dps);
  const bool quiet = gyro_mag < ATTITUDE_STILL_GYRO_DPS;

  if (vehicle_stopped && quiet) {
    // 防溢出：静态车连续放 71 分钟以上就不再累加，反正早超阈值了
    if (g_still_us < 4000000000U) {
      g_still_us += elapsed_us;
    }
  } else {
    g_still_us = 0;
  }
  g_still = (g_still_us >= ATTITUDE_STILL_DELAY_US);

  // ---- yaw：纯积分，没有校正项（没磁力计，见头文件说明）----
  // 唯一的例外是静止时冻结 —— 车都没转，yaw 就不该动。
  if (!g_still) {
    g_att.yaw_deg += s.gyro_z_dps * dt;
  }

  // ---- 零偏在线跟踪：只在不该有转角的时刻学，学的才是真零偏 ----
  // s.gyro_* 已经是扣过当前零偏的值，所以它此刻读数就是「零偏估计的误差」；
  // 按比例把这个误差补回零偏里，估计值就会指数收敛到真值（时间常数 TAU）。
  // 注意用的是 s.gyro_*（扣过的）而不是原始值 —— 否则会自己咬自己尾巴，不收敛。
  if (g_still) {
    const float k = dt / ATTITUDE_BIAS_TRACK_TAU_S;
    const ImuGyroBias b = imu_get_bias();
    imu_set_bias(b.x_dps + k * s.gyro_x_dps,
                 b.y_dps + k * s.gyro_y_dps,
                 b.z_dps + k * s.gyro_z_dps);
  }

  // ---- 实测频率统计 ----
  g_rate_count++;
  if (now_us - g_rate_window_us >= 1000000U) {
    g_rate_hz = static_cast<uint16_t>(g_rate_count);
    g_rate_count = 0;
    g_rate_window_us = now_us;
  }

  return true;
}

Attitude attitude_get()
{
  return g_att;
}

bool attitude_is_still()
{
  return g_still;
}

void attitude_zero_yaw()
{
  g_att.yaw_deg = 0.0f;
}

void attitude_set_alpha(float alpha)
{
  if (alpha < ATTITUDE_ALPHA_MIN) {
    alpha = ATTITUDE_ALPHA_MIN;
  }
  if (alpha > ATTITUDE_ALPHA_MAX) {
    alpha = ATTITUDE_ALPHA_MAX;
  }
  g_alpha = alpha;
}

float attitude_get_alpha()
{
  return g_alpha;
}

float attitude_get_dt_s()
{
  return g_last_dt_s;
}

uint16_t attitude_get_rate_hz()
{
  return g_rate_hz;
}
