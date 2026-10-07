# ros_car_esp32s3

差速底盘 ESP32-S3 固件：电机驱动、轮式编码器、IMU 姿态解算、航位推算里程计与串口通信协议。

## 模块

| 模块 | 说明 |
|---|---|
| `motor_driver` | 双路 PWM 电机驱动 |
| `wheel_encoder` | AB 双相正交编码器，4 倍频计数 |
| `imu_mpu6050` | MPU6050 I2C 寄存器级驱动，零偏标定 |
| `attitude` | 互补滤波姿态解算（roll/pitch/yaw），静止检测与零偏在线跟踪 |
| `chassis` | 轮速环：目标速度 → 增量式 PID → PWM，差速混速 |
| `yaw_control` | 转向环：目标角度 → yaw 闭环 |
| `odometry` | 航位推算：编码器路程 + IMU 航向 → x/y/θ，中点航向积分 |
| `serial_frame` | 帧协议：校验、指令分发、遥测上行 |

## 通信

- 串口：115200 8N1
- 上行：`[ODOM]` 5Hz、`[ATT]`、`[IMU]`、`[DATA]` 遥测
- 下行：运动控制帧（v/w）、PID 参数、距离刻度标定（0x2A）、陀螺零偏标定（0x22）

## 构建

PlatformIO，目标 `esp32-s3-devkitm-1`：

```bash
pio run -t upload
```
