#include <Arduino.h>
#include "motor_driver.h"

// 单个电机的「换向死区 + PWM/方向双线」核心逻辑（左右轮共用）
static void set_motor_pwm_with_dead_time(uint8_t dir_pin,
                                         uint8_t pwm_pin,
                                         int pwm,
                                         bool invert_dir,
                                         bool *last_forward,
                                         bool *has_last_forward);

void motor_init()
{
  // 1. 方向线与 PWM 线全部配置为输出
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);

  // 2. 上电即停：方向线拉低 + PWM 输出 0
  //    注意：PWM 线全程只用 analogWrite（LEDC 一旦接管引脚，digitalWrite 不生效），
  //    方向线全程只用 digitalWrite，两线职责固定，不混用。
  digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
  digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
}

void set_right_motor_pwm(int pwm)
{
  // static 记录上次方向，使左右轮各自独立记忆
  static bool last_forward = true;
  static bool has_last_forward = false;

  set_motor_pwm_with_dead_time(RIGHT_MOTOR_DIR_PIN,
                               RIGHT_MOTOR_PWM_PIN,
                               pwm,
                               RIGHT_MOTOR_INVERT_DIR,
                               &last_forward,
                               &has_last_forward);
}

void set_left_motor_pwm(int pwm)
{
  static bool last_forward = true;
  static bool has_last_forward = false;

  set_motor_pwm_with_dead_time(LEFT_MOTOR_DIR_PIN,
                               LEFT_MOTOR_PWM_PIN,
                               pwm,
                               LEFT_MOTOR_INVERT_DIR,
                               &last_forward,
                               &has_last_forward);
}

// 输入 pwm 为 -255~255：符号表示方向，绝对值表示占空比
static void set_motor_pwm_with_dead_time(uint8_t dir_pin,
                                         uint8_t pwm_pin,
                                         int pwm,
                                         bool invert_dir,
                                         bool *last_forward,
                                         bool *has_last_forward)
{
  // 1. 限幅，防止越界
  pwm = constrain(pwm, -255, 255);

  // 2. 计算期望方向（正 pwm = 正转），INVERT 位表示「正转时方向线为低」
  bool forward = (pwm >= 0);
  if (invert_dir) forward = !forward;

  // 3. 换向死区：如果本次方向与上次不同，先停 PWM，短暂延时让开关管完全截止，
  //    再翻方向线、恢复 PWM，避免换向瞬间上下桥臂直通
  if (*has_last_forward && forward != *last_forward) {
    analogWrite(pwm_pin, 0);
    delayMicroseconds(MOTOR_DIRECTION_SWITCH_DELAY_US);
  }

  // 4. 方向线输出数字电平：正转 = HIGH，反转 = LOW（invert 已在 forward 中体现）
  digitalWrite(dir_pin, forward ? HIGH : LOW);

  // 5. PWM 线输出占空比
  analogWrite(pwm_pin, abs(pwm));

  // 6. 记录本次方向，供下次换向判断
  *last_forward = forward;
  *has_last_forward = true;
}
