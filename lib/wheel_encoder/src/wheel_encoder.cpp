#include "wheel_encoder.h"
#include <driver/gpio.h>

// ---- 全局状态 ----
static volatile int32_t g_left = 0;        // 左轮累计计数（中断中修改，必须 volatile）
static volatile int32_t g_right = 0;       // 右轮累计计数
static volatile uint8_t g_left_state = 0;  // 左轮当前 AB 电平状态（2bit）
static volatile uint8_t g_right_state = 0; // 右轮当前 AB 电平状态（2bit）
static int32_t g_speed_last_left = 0;      // 上次测速时的左轮计数
static int32_t g_speed_last_right = 0;     // 上次测速时的右轮计数
static uint32_t g_speed_last_us = 0;       // 上次测速时间（us）
static bool g_speed_inited = false;        // 测速是否已初始化
static bool g_irq_attached = false;        // 中断是否已挂载

// 正交编码 4 倍频状态机查表：
// 表下标 = (上一次AB状态 << 2) | 本次AB状态，值 = 本次跳变应加的计数。
// 合法的相邻状态跳变（顺时针/逆时针各一步）加 ±1，其余（保持/对角跳变）为 0，
// 从而天然滤除单相抖动。
static const int8_t g_quad_table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

// 中断里不能用 digitalRead（可能加锁），直接读 GPIO 寄存器
static inline uint8_t IRAM_ATTR fast_gpio_read(uint8_t pin)
{
  return static_cast<uint8_t>(gpio_get_level(static_cast<gpio_num_t>(pin)));
}

// 读取当前 AB 状态 → 查表累加 → 更新记录状态（左右轮对称）
static inline void IRAM_ATTR wheel_encoder_update_left()
{
  const uint8_t a = fast_gpio_read(WHEEL_ENC_L_A_PIN);
  const uint8_t b = fast_gpio_read(WHEEL_ENC_L_B_PIN);
  const uint8_t state = static_cast<uint8_t>((a << 1) | b);
  const uint8_t table_idx = static_cast<uint8_t>((g_left_state << 2) | state);
  g_left += g_quad_table[table_idx];
  g_left_state = state;
}

static inline void IRAM_ATTR wheel_encoder_update_right()
{
  const uint8_t a = fast_gpio_read(WHEEL_ENC_R_A_PIN);
  const uint8_t b = fast_gpio_read(WHEEL_ENC_R_B_PIN);
  const uint8_t state = static_cast<uint8_t>((a << 1) | b);
  const uint8_t table_idx = static_cast<uint8_t>((g_right_state << 2) | state);
  g_right += g_quad_table[table_idx];
  g_right_state = state;
}

// A/B 两相都接 CHANGE 中断（4 倍频：每相上升沿 + 下降沿都计数）
static void IRAM_ATTR wheel_encoder_isr_left_a()
{
  wheel_encoder_update_left();
}

static void IRAM_ATTR wheel_encoder_isr_left_b()
{
  wheel_encoder_update_left();
}

static void IRAM_ATTR wheel_encoder_isr_right_a()
{
  wheel_encoder_update_right();
}

static void IRAM_ATTR wheel_encoder_isr_right_b()
{
  wheel_encoder_update_right();
}

void wheel_encoder_init(bool enable_pullups)
{
  // 1. 引脚配置：霍尔编码器多为开漏输出，默认开启内部上拉
  pinMode(WHEEL_ENC_L_A_PIN, enable_pullups ? INPUT_PULLUP : INPUT);
  pinMode(WHEEL_ENC_L_B_PIN, enable_pullups ? INPUT_PULLUP : INPUT);
  pinMode(WHEEL_ENC_R_A_PIN, enable_pullups ? INPUT_PULLUP : INPUT);
  pinMode(WHEEL_ENC_R_B_PIN, enable_pullups ? INPUT_PULLUP : INPUT);

  // 2. 记录初始 AB 状态，避免首次中断查表时状态未初始化
  g_left_state = static_cast<uint8_t>(
      (digitalRead(WHEEL_ENC_L_A_PIN) << 1) | digitalRead(WHEEL_ENC_L_B_PIN));
  g_right_state = static_cast<uint8_t>(
      (digitalRead(WHEEL_ENC_R_A_PIN) << 1) | digitalRead(WHEEL_ENC_R_B_PIN));

  // 3. 重复初始化时先摘除旧中断，防止同一引脚被重复挂载
  if (g_irq_attached) {
    detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_A_PIN));
    detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_B_PIN));
    detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_A_PIN));
    detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_B_PIN));
  }

  // 4. 挂载四路 CHANGE 中断
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_A_PIN), wheel_encoder_isr_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_B_PIN), wheel_encoder_isr_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_A_PIN), wheel_encoder_isr_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_B_PIN), wheel_encoder_isr_right_b, CHANGE);
  g_irq_attached = true;
}

void wheel_encoder_speed_init()
{
  wheel_encoder_get_counts(&g_speed_last_left, &g_speed_last_right);
  g_speed_last_us = micros();
  g_speed_inited = true;
}

bool wheel_encoder_get_speed_cm_s(float *left_cm_s, float *right_cm_s, uint32_t sample_us)
{
  // 首次调用自动初始化基准点
  if (!g_speed_inited) {
    wheel_encoder_speed_init();
    return false;
  }

  // 采样周期未到，返回 false（不更新速度）
  const uint32_t now_us = micros();
  const uint32_t dt_us = now_us - g_speed_last_us;
  if (dt_us == 0U) {
    return false;
  }
  if (dt_us < sample_us) {
    return false;
  }

  // 读取累计计数，计算周期内增量 → 速度（cm/s）
  int32_t left = 0;
  int32_t right = 0;
  wheel_encoder_get_counts(&left, &right);

  const int32_t dl = left - g_speed_last_left;
  const int32_t dr = right - g_speed_last_right;
  const float dt_s = static_cast<float>(dt_us) * 1e-6f;
  const float v_left = wheel_encoder_delta_counts_to_speed_cm_s(dl, dt_s);
  const float v_right = wheel_encoder_delta_counts_to_speed_cm_s(dr, dt_s);

  if (left_cm_s) *left_cm_s = v_left;
  if (right_cm_s) *right_cm_s = v_right;

  // 滚动更新基准点
  g_speed_last_left = left;
  g_speed_last_right = right;
  g_speed_last_us = now_us;
  return true;
}

void wheel_encoder_get_counts(int32_t *left, int32_t *right)
{
  // 临界区保护：int32 读取在 32 位 MCU 上是原子的，但为兼容性仍关中断取数
  noInterrupts();
  const int32_t l = g_left;
  const int32_t r = g_right;
  interrupts();

  if (left) *left = l * static_cast<int32_t>(WHEEL_ENC_L_SIGN);
  if (right) *right = r * static_cast<int32_t>(WHEEL_ENC_R_SIGN);
}

int32_t wheel_encoder_get_left()
{
  noInterrupts();
  const int32_t l = g_left;
  interrupts();
  return l * static_cast<int32_t>(WHEEL_ENC_L_SIGN);
}

int32_t wheel_encoder_get_right()
{
  noInterrupts();
  const int32_t r = g_right;
  interrupts();
  return r * static_cast<int32_t>(WHEEL_ENC_R_SIGN);
}

void wheel_encoder_reset()
{
  noInterrupts();
  g_left = 0;
  g_right = 0;
  interrupts();
  wheel_encoder_speed_init();
}

void wheel_encoder_get_and_reset(int32_t *left, int32_t *right)
{
  noInterrupts();
  const int32_t l = g_left;
  const int32_t r = g_right;
  g_left = 0;
  g_right = 0;
  interrupts();

  if (left) *left = l * static_cast<int32_t>(WHEEL_ENC_L_SIGN);
  if (right) *right = r * static_cast<int32_t>(WHEEL_ENC_R_SIGN);
}
