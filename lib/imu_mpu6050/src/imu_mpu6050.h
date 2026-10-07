#pragma once

#include <Arduino.h>

// ============================================================================
// IMU —— MPU6050 六轴（3 轴加速度 + 3 轴陀螺仪），I2C 寄存器级驱动
//
//   零第三方库：I2C 走 Arduino 核心自带的 Wire，寄存器全部手写。
//   本模块只负责「把芯片里的 14 字节原始数据变成物理量 + 扣掉零偏」，
//   姿态解算（互补滤波出 roll/pitch/yaw）是下一层的事，不混在这里。
//
//   接线（模块 GY-521）：
//     VCC → 3V3       GND → GND
//     SDA → GPIO8     SCL → GPIO9
//     AD0 → 接地或悬空（决定 I2C 地址：低 = 0x68，高 = 0x69）
//     INT → 不接（本项目不跑片上 DMP，不需要「数据就绪」中断）
//   GY-521 板载 4.7k 上拉，无需外接；换成裸芯片才要自己加上拉。
//
//   一条 I2C 连读拿全六轴（见下方 IMU_REG_ACCEL_XOUT_H）：
//     加速度(6B) + 温度(2B) + 陀螺仪(6B) = 14B
//   分 6 次读会慢 6 倍，且 6 轴不是同一时刻采样的 —— 对姿态解算致命。
//
//   ⚠ 局限：MPU6050 没有磁力计。加速度计只能在静止时提供重力方向，
//     那是 roll/pitch 的绝对参考，但给不了 yaw 的绝对参考。
//     所以 roll/pitch 能用互补滤波做到不漂，yaw 只能靠陀螺仪积分，
//     必须靠静态零偏标定把漂移压到最小。
// ============================================================================

// ---- I2C 硬件配置 ----
static constexpr uint8_t IMU_I2C_SDA_PIN = 8;        // GPIO8
static constexpr uint8_t IMU_I2C_SCL_PIN = 9;        // GPIO9
static constexpr uint32_t IMU_I2C_FREQ_HZ = 400000U; // 400kHz 快速模式（实测可用）
// 若出现读数据偶发失败 / WHO_AM_I 时好时坏：先把这里降到 100000（100kHz 标准模式）。
// 杜邦线偏长、走线靠近电机线时，400kHz 容易被干扰。

// ---- 从机地址 ----
// AD0 引脚接地（或悬空，GY-521 板上有下拉）→ 0x68；AD0 接 3V3 → 0x69
static constexpr uint8_t IMU_I2C_ADDR = 0x68;

// ---- 芯片身份 ----
static constexpr uint8_t IMU_REG_WHO_AM_I = 0x75;    // 只读，MPU6050 固定返回 0x68
static constexpr uint8_t IMU_WHO_AM_I_VALUE = 0x68;  // 用来验「芯片到底在不在」

// ---- 量程档位（改这里换量程，灵敏度换算值必须跟着改！）----
// 陀螺仪 ±250°/s：小车原地转最快约 200°/s，这档分辨率最高（131 LSB/(°/s)）。
//   选更大档位 = 白丢分辨率，除非实测发现有削顶（转最快时读数顶死在 ±32767）。
static constexpr uint8_t IMU_GYRO_FS_SEL = 0;  // 0=±250 1=±500 2=±1000 3=±2000 °/s
// 加速度计 ±4g：留出撞击/颠簸余量；±2g 遇到减速带容易削顶。
static constexpr uint8_t IMU_ACCEL_FS_SEL = 1; // 0=±2g 1=±4g 2=±8g 3=±16g

// ---- 采样率与滤波 ----
// DLPF_CFG 档位 → (加速度带宽 / 陀螺仪带宽 / 群延时)：
//   0: 260/256Hz  0ms      4: 21/20Hz  8.3ms
//   1: 184/188Hz  1.9ms    5: 10/10Hz  13.8ms
//   2:  94/98Hz   2.8ms    6:  5/5Hz   19ms
//   3:  44/42Hz   4.8ms
// 取 3（44Hz）—— 车体振动主要在 20~100Hz，滤掉大部分噪声，延时 4.8ms 还能接受。
// 想让角度更平滑就调到 4（21Hz，延时涨到 8.3ms）；想跟手就调到 2（94Hz，噪声大）。
static constexpr uint8_t IMU_DLPF_CFG = 3;

// 期望采样率。注意下面这个耦合关系（写错采样率就不对）：
//   DLPF 使能时（CFG=0~6），陀螺仪内部输出率 = 1kHz
//   DLPF 关闭时（CFG=7），陀螺仪内部输出率 = 8kHz
//   实际采样率 = 内部输出率 / (1 + SMPLRT_DIV)
static constexpr uint16_t IMU_SAMPLE_RATE_HZ = 100;
static constexpr uint8_t IMU_SMPLRT_DIV =
    static_cast<uint8_t>(1000U / IMU_SAMPLE_RATE_HZ - 1U); // 1000/100 - 1 = 9

// ---- 换算系数（必须与上面的 FS_SEL 一一对应）----
static constexpr float IMU_GYRO_LSB_PER_DPS = 131.0f; // FS_SEL=0 → ±250°/s
static constexpr float IMU_ACCEL_LSB_PER_G = 8192.0f; // AFS_SEL=1 → ±4g

// 温度换算（数据手册公式，与量程无关）
static constexpr float IMU_TEMP_LSB_PER_DEG = 340.0f;
static constexpr float IMU_TEMP_OFFSET_DEG = 36.53f;

// ---- 零偏标定 ----
// 默认样本数：100Hz 下 500 个 = 5 秒。噪声被平均掉 √500 ≈ 22 倍，足够。
static constexpr uint16_t IMU_CAL_DEFAULT_SAMPLES = 500;
// 上限 3000 个 ≈ 30 秒：防误输入（如 c 99999）让标定跑成几十分钟
static constexpr uint16_t IMU_CAL_MAX_SAMPLES = 3000;
// 标定采样节拍：10ms = 100Hz，与芯片输出率对齐
static constexpr uint32_t IMU_CAL_SAMPLE_PERIOD_US = 10000U;
// 预热丢弃样本数：标定刚开始的芯片输出还不稳，先丢 20 个（200ms）不计入均值
static constexpr uint16_t IMU_CAL_WARMUP_SAMPLES = 20;

// ---- 数据结构 ----

// 原始 LSB 值（未换算、未扣零偏）—— 标定和排查接线时用
struct ImuRaw {
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

// 一帧物理量数据（陀螺仪已扣零偏）
struct ImuSample {
  float accel_x_g;    // 加速度，单位 g（1g = 9.8 m/s²）
  float accel_y_g;
  float accel_z_g;
  float gyro_x_dps;   // 角速度，单位 °/s
  float gyro_y_dps;
  float gyro_z_dps;
  float temp_c;       // 芯片温度（℃），仅供健康检查，不是环境温度
  bool bias_applied;  // 该帧是否已扣零偏（false = 从没标定过，gyro 值含零偏）
};

// 陀螺仪零偏（静止时三轴各自的输出，单位 °/s）
struct ImuGyroBias {
  float x_dps;
  float y_dps;
  float z_dps;
  uint16_t samples;   // 标定用了多少样本；0 = 从未标定
};

// ---- 初始化与状态 ----
// 初始化 I2C + 配置芯片；返回 false = 没找到芯片（接线/供电问题，不影响底盘）
bool imu_init();
bool imu_is_online();

// ---- 读数据 ----
bool imu_read_raw(ImuRaw *out);  // 读 14 字节原始值（标定 / 排查用）
bool imu_read(ImuSample *out);   // 读一帧物理量（陀螺仪扣零偏）

// ---- 零偏标定（非阻塞：绝不能在 loop 里用 delay 标定，会卡死底盘闭环）----
// 用法：imu_calibrate_start(); 然后在 loop 里每次调用 imu_calibrate_update()，
//       返回 true 表示标定完成、零偏已写入。标定期间车必须静止！
void imu_calibrate_start(uint16_t samples = IMU_CAL_DEFAULT_SAMPLES);
bool imu_calibrate_update();
bool imu_calibrate_busy();
uint8_t imu_calibrate_percent();  // 0~100

ImuGyroBias imu_get_bias();
void imu_set_bias(float x_dps, float y_dps, float z_dps); // 手动设零偏
void imu_clear_bias();

// ---- 排查工具 ----
// 扫描 I2C 总线，把发现的 7 位地址填进 found_addrs（最多 max_addrs 个），
// 返回实际发现数量。接线对不对、地址是不是 0x68，一跑就知道。
uint8_t imu_i2c_scan(uint8_t *found_addrs, uint8_t max_addrs);
