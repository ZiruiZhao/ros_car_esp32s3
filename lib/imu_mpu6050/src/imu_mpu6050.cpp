#include "imu_mpu6050.h"
#include <Wire.h>

// ============================================================================
// MPU6050 寄存器地址表（只用得到的，全部来自数据手册 Register Map）
// ============================================================================
static constexpr uint8_t REG_SMPLRT_DIV = 0x19;   // 采样率分频
static constexpr uint8_t REG_CONFIG = 0x1A;       // DLPF 数字低通滤波
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;  // 陀螺仪量程
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C; // 加速度计量程
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B; // 14 字节连读的起点
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;   // 电源管理 1（复位 / 唤醒 / 时钟源）

// 连读字节数：加速度(6) + 温度(2) + 陀螺仪(6)
static constexpr uint8_t IMU_BURST_LEN = 14;

// PWR_MGMT_1 的两个关键取值
static constexpr uint8_t PWR_DEVICE_RESET = 0x80; // bit7：写 1 触发软件复位（自动回 0）
static constexpr uint8_t PWR_CLKSEL_GYRO_PLL = 0x01; // CLKSEL=1：用 X 轴陀螺 PLL 做时钟源

// ============================================================================
// 模块内部状态
// ============================================================================
static bool g_wire_ready = false; // I2C 外设是否已初始化
static bool g_online = false;     // WHO_AM_I 校验是否通过

static ImuGyroBias g_bias = {0.0f, 0.0f, 0.0f, 0}; // 零偏，samples=0 表示未标定

// 零偏标定状态机（非阻塞）
static bool g_cal_busy = false;
static uint16_t g_cal_target = 0;      // 目标样本数
static uint16_t g_cal_count = 0;       // 已采有效样本数
static uint16_t g_cal_warmup_left = 0; // 还需丢弃的预热样本数
static double g_cal_sum_x = 0.0;       // 累加和用 double：int16 累加 500 次会溢出，double 不丢精度
static double g_cal_sum_y = 0.0;
static double g_cal_sum_z = 0.0;
static uint32_t g_cal_last_us = 0;     // 上次采样时刻（节拍控制）

// ============================================================================
// I2C 底层读写
// ============================================================================

// I2C 外设初始化（只做一次）
static void imu_wire_begin()
{
  if (g_wire_ready) return;
  Wire.begin(IMU_I2C_SDA_PIN, IMU_I2C_SCL_PIN);
  Wire.setClock(IMU_I2C_FREQ_HZ);
  g_wire_ready = true;
}

// 写一个寄存器。返回 false = 从机没应答（接线 / 供电 / 地址不对）
static bool imu_write_reg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0; // 0 = 收到 ACK
}

// 从 reg 起连读 len 字节到 buf
static bool imu_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  // endTransmission(false)：写完之后不发 STOP，直接接一个「重复起始」再读。
  // 这是 I2C 读寄存器的标准时序。
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(IMU_I2C_ADDR, len) != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

// 读单个寄存器（当前只在读 WHO_AM_I 时用）
static bool imu_read_reg(uint8_t reg, uint8_t *out)
{
  return imu_read_regs(reg, out, 1);
}

// ============================================================================
// 初始化
// ============================================================================
bool imu_init()
{
  g_online = false;

  // ---- 第 1 步：初始化 I2C 外设，设置好时钟 ----
  imu_wire_begin();

  // ---- 第 2 步：读 WHO_AM_I（0x75），确认硬件和通信正常 ----
  // 放在配置之前：先验「芯片在不在、通不通」，再谈怎么配它。
  // 不通的话后面的写入全是白费，还会把故障点搞模糊。
  uint8_t who = 0;
  if (!imu_read_reg(IMU_REG_WHO_AM_I, &who)) return false; // I2C 无应答
  if (who != IMU_WHO_AM_I_VALUE) return false;             // 应答了但不是 MPU6050

  // ---- 第 3 步：写 PWR_MGMT_1 = 0x80 触发软件复位，延时 100ms ----
  // 复位会把所有寄存器拉回默认值，所以必须在后面所有配置之前做。
  if (!imu_write_reg(REG_PWR_MGMT_1, PWR_DEVICE_RESET)) return false;
  delay(100);

  // 复位后芯片回到睡眠态，且时钟源是内部 8MHz RC 振荡器 —— 那东西温漂大，
  // 陀螺仪零偏会随温度乱跑。所以下一步必须显式唤醒 + 换时钟源。
  // 顺带一提：复位后要再读一次 WHO_AM_I 确认它活过来了吗？不必 ——
  // 数据手册规定复位后 WHO_AM_I 值不变，且第 4 步的写操作本身就是探活。

  // ---- 第 4 步：写 PWR_MGMT_1 = 0x01，唤醒 + 选 X 轴陀螺 PLL 时钟源 ----
  // bit6 SLEEP=0 退出睡眠；CLKSEL[2:0]=001 选陀螺仪 X 轴 PLL。
  // 这一字节是「零偏稳不稳」的关键，别图省事写 0x00。
  if (!imu_write_reg(REG_PWR_MGMT_1, PWR_CLKSEL_GYRO_PLL)) return false;
  delay(10); // 等 PLL 锁定

  // ---- 第 5 步：写 CONFIG（0x1A），设置 DLPF 低通滤波带宽 ----
  // 低 3 位是 DLPF_CFG，高 5 位保留（写 0）。
  if (!imu_write_reg(REG_CONFIG, IMU_DLPF_CFG & 0x07)) return false;

  // ---- 第 6 步：写 SMPLRT_DIV（0x19），设置采样分频 ----
  if (!imu_write_reg(REG_SMPLRT_DIV, IMU_SMPLRT_DIV)) return false;

  // ---- 第 7 步：写 GYRO_CONFIG（0x1B），设置陀螺仪量程 ----
  // FS_SEL 在 bit[4:3]，所以要左移 3 位；bit[7:5] 是自检位，写 0 关闭。
  if (!imu_write_reg(REG_GYRO_CONFIG, static_cast<uint8_t>(IMU_GYRO_FS_SEL << 3))) return false;

  // ---- 第 8 步：写 ACCEL_CONFIG（0x1C），设置加速度计量程 ----
  // AFS_SEL 同样在 bit[4:3]。
  if (!imu_write_reg(REG_ACCEL_CONFIG, static_cast<uint8_t>(IMU_ACCEL_FS_SEL << 3))) return false;

  delay(10); // 等滤波器稳定
  g_online = true;
  return true;
}

bool imu_is_online()
{
  return g_online;
}

// ============================================================================
// 读数据
// ============================================================================
bool imu_read_raw(ImuRaw *out)
{
  if (!g_online || out == nullptr) return false;

  // 一次连读 14 字节：既省 6 次 I2C 往返，又保证 6 轴是同一时刻的采样值。
  uint8_t buf[IMU_BURST_LEN];
  if (!imu_read_regs(REG_ACCEL_XOUT_H, buf, IMU_BURST_LEN)) return false;

  // 每轴 2 字节，大端（高字节在前）。先凑成 int 再截成 int16_t，
  // 这样负数的符号位能正确保留（buf[0] 是 uint8_t，左移 8 位前会提升为 int）。
  out->accel_x = static_cast<int16_t>((buf[0] << 8) | buf[1]);
  out->accel_y = static_cast<int16_t>((buf[2] << 8) | buf[3]);
  out->accel_z = static_cast<int16_t>((buf[4] << 8) | buf[5]);
  out->temp = static_cast<int16_t>((buf[6] << 8) | buf[7]);
  out->gyro_x = static_cast<int16_t>((buf[8] << 8) | buf[9]);
  out->gyro_y = static_cast<int16_t>((buf[10] << 8) | buf[11]);
  out->gyro_z = static_cast<int16_t>((buf[12] << 8) | buf[13]);
  return true;
}

bool imu_read(ImuSample *out)
{
  if (out == nullptr) return false;

  ImuRaw raw;
  if (!imu_read_raw(&raw)) return false;

  out->accel_x_g = static_cast<float>(raw.accel_x) / IMU_ACCEL_LSB_PER_G;
  out->accel_y_g = static_cast<float>(raw.accel_y) / IMU_ACCEL_LSB_PER_G;
  out->accel_z_g = static_cast<float>(raw.accel_z) / IMU_ACCEL_LSB_PER_G;

  out->gyro_x_dps = static_cast<float>(raw.gyro_x) / IMU_GYRO_LSB_PER_DPS;
  out->gyro_y_dps = static_cast<float>(raw.gyro_y) / IMU_GYRO_LSB_PER_DPS;
  out->gyro_z_dps = static_cast<float>(raw.gyro_z) / IMU_GYRO_LSB_PER_DPS;

  // 扣零偏：必须在换算成 °/s 之后扣，量纲才对得上
  out->bias_applied = (g_bias.samples > 0);
  if (out->bias_applied) {
    out->gyro_x_dps -= g_bias.x_dps;
    out->gyro_y_dps -= g_bias.y_dps;
    out->gyro_z_dps -= g_bias.z_dps;
  }

  out->temp_c = static_cast<float>(raw.temp) / IMU_TEMP_LSB_PER_DEG + IMU_TEMP_OFFSET_DEG;
  return true;
}

// ============================================================================
// 零偏标定（非阻塞状态机）
//
//   为什么不做成阻塞函数（while 循环 delay 采样完再返回）？
//   因为标定要 5 秒，而 loop() 里同时跑着底盘 50ms 闭环。
//   一阻塞就是 5 秒不跑闭环 —— 万一标定时车没停住，轮子会以最后的 PWM 冲出去。
//   拆成 start/update 两步，标定期间串口还收得到 s 急停。
// ============================================================================
void imu_calibrate_start(uint16_t samples)
{
  // 限幅：0 = 用默认值，超上限按上限。放在驱动层做，
  // 这样任何调用方（包括以后的 ROS 桥接）都不会把标定跑飞。
  if (samples == 0U) {
    samples = IMU_CAL_DEFAULT_SAMPLES;
  } else if (samples > IMU_CAL_MAX_SAMPLES) {
    samples = IMU_CAL_MAX_SAMPLES;
  }

  g_cal_target = samples;
  g_cal_count = 0;
  g_cal_warmup_left = IMU_CAL_WARMUP_SAMPLES;
  g_cal_sum_x = 0.0;
  g_cal_sum_y = 0.0;
  g_cal_sum_z = 0.0;
  g_cal_last_us = micros();
  g_cal_busy = true;
}

bool imu_calibrate_update()
{
  if (!g_cal_busy) return false;

  // 按 100Hz 节拍采样 —— loop() 每秒跑几万次，不能每次调用都读 I2C
  const uint32_t now_us = micros();
  if (now_us - g_cal_last_us < IMU_CAL_SAMPLE_PERIOD_US) return false;
  g_cal_last_us = now_us;

  // 标定读的是「原始值」，不扣零偏 —— 标定的目的就是测出这个零偏
  ImuRaw raw;
  if (!imu_read_raw(&raw)) return false; // 读失败不计数，下个节拍自动重试

  // 预热期直接丢：芯片刚配置完、滤波器刚启动，头几个样本不代表真实零偏
  if (g_cal_warmup_left > 0) {
    g_cal_warmup_left--;
    return false;
  }

  g_cal_sum_x += static_cast<double>(raw.gyro_x);
  g_cal_sum_y += static_cast<double>(raw.gyro_y);
  g_cal_sum_z += static_cast<double>(raw.gyro_z);
  g_cal_count++;

  if (g_cal_count < g_cal_target) return false;

  // 采满 → 求均值 → 换算成 °/s 写入零偏
  g_bias.x_dps = static_cast<float>(g_cal_sum_x / g_cal_target) / IMU_GYRO_LSB_PER_DPS;
  g_bias.y_dps = static_cast<float>(g_cal_sum_y / g_cal_target) / IMU_GYRO_LSB_PER_DPS;
  g_bias.z_dps = static_cast<float>(g_cal_sum_z / g_cal_target) / IMU_GYRO_LSB_PER_DPS;
  g_bias.samples = g_cal_target;

  g_cal_busy = false;
  return true;
}

bool imu_calibrate_busy()
{
  return g_cal_busy;
}

uint8_t imu_calibrate_percent()
{
  if (!g_cal_busy || g_cal_target == 0U) return 0U;
  return static_cast<uint8_t>((static_cast<uint32_t>(g_cal_count) * 100U) / g_cal_target);
}

ImuGyroBias imu_get_bias()
{
  return g_bias;
}

void imu_set_bias(float x_dps, float y_dps, float z_dps)
{
  g_bias.x_dps = x_dps;
  g_bias.y_dps = y_dps;
  g_bias.z_dps = z_dps;
  g_bias.samples = 1U; // 非 0 即视为「已标定」，imu_read 会开始扣零偏
}

void imu_clear_bias()
{
  g_bias.x_dps = 0.0f;
  g_bias.y_dps = 0.0f;
  g_bias.z_dps = 0.0f;
  g_bias.samples = 0U;
}

// ============================================================================
// 排查工具
// ============================================================================
uint8_t imu_i2c_scan(uint8_t *found_addrs, uint8_t max_addrs)
{
  imu_wire_begin(); // 允许不调 imu_init 直接扫描（接线排查场景）

  uint8_t found = 0;
  for (uint8_t addr = 0x01; addr < 0x7FU; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue; // 没 ACK = 这个地址上没设备
    if (found_addrs != nullptr && found < max_addrs) {
      found_addrs[found] = addr;
    }
    found++; // 超出数组容量的也照常计数，返回值才是真实的「发现总数」
  }
  return found;
}
