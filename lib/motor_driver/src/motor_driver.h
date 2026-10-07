#pragma once

#include <Arduino.h>

// ============================================================================
// 电机驱动 —— PWM + 方向 双线方案（每路电机 2 根信号线）
//
//   每路电机固定一根 PWM 线（全程 LEDC 输出占空比）+ 一根方向线（纯数字电平），
//   正转/反转只切换方向线电平，PWM 线始终输出当前占空比。
//   停车 = PWM 输出 0 + 方向线拉低。
//
//   引脚定义（实测接线，以代码数值为准）：
//     右轮：PWM=GPIO1，方向=GPIO2
//     左轮：PWM=GPIO42，方向=GPIO41
// ============================================================================

// 未填写引脚的占位值（0xFF 不是合法 GPIO，供上层做「未填写」判断）
static constexpr uint8_t PIN_UNSET = 0xFF;

// 右轮
static constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 42;  // PWM 线，全程 LEDC（analogWrite）
static constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 1;  // 方向线，纯数字电平（digitalWrite）

// 左轮
static constexpr uint8_t LEFT_MOTOR_PWM_PIN = 41;  // PWM 线，全程 LEDC（analogWrite）
static constexpr uint8_t LEFT_MOTOR_DIR_PIN = 2;  // 方向线，纯数字电平（digitalWrite）

// 方向反相标志：正转时方向线应为低的电机置 true（由整车接线决定）
static constexpr bool RIGHT_MOTOR_INVERT_DIR = false; // 右轮正转方向线=高
static constexpr bool LEFT_MOTOR_INVERT_DIR = true;   // 左轮正转方向线=低（反相）

// 换向死区时间：翻转方向前先停 PWM 若干微秒，防止 H 桥上下桥臂切换瞬间直通短路
static constexpr uint32_t MOTOR_DIRECTION_SWITCH_DELAY_US = 1000U;

void motor_init();
void set_right_motor_pwm(int pwm);
void set_left_motor_pwm(int pwm);
