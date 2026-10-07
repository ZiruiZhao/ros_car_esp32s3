#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS-CAR 上位机控制台（图形界面）

替代终端版串口助手：按钮直接发运动指令，速度/目标/PWM 实时数字显示 + 滚动曲线，
IMU 六轴数据同样有数字面板和陀螺仪曲线，不用再盯着一行行刷屏的 [DATA] 看。

只依赖 pyserial + Python 自带的 Tkinter，不需要额外安装 GUI 库。

运行：
    python test/car_upper.py
    python test/car_upper.py -p COM3 -b 115200

界面分区：
    连接      选端口、连接/断开
    运动控制  速度滑块 + 时长，前进/后退/左转/右转/掉头/急停
    底盘状态  左右轮速度、目标、PWM、计数 + 速度曲线 + PID 整定（标签页）
    IMU       加速度/角速度/温度数字面板 + 陀螺仪曲线 + 标定/扫描（标签页）
    里程计    XY 轨迹图 + 位姿读数 + 距离刻度标定（标签页）
    日志      板子回传的 [CMD] / [YAW] / [FRM] 等文本日志

下行只有二进制帧一条路（见 串口通信手册.md）。板子的回复仍然是文本日志行 ——
帧协议只统一了「上位机 → 下位机」这个方向。
"""

from __future__ import annotations

import argparse
import collections
import csv
import math
import queue
import re
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺少 pyserial，请先安装：pip install pyserial")

DATA_PREFIX = "[DATA]"
# 固件里 IMU 数据行的前缀。带上 "DATA" 是为了和「标定进度 / 扫描结果 / 报错」
# 那些纯文本行分开 —— 后者直接进日志，前者要送进面板和曲线。
IMU_DATA_PREFIX = "[IMU] DATA"
ATT_DATA_PREFIX = "[ATT] DATA"
ODOM_DATA_PREFIX = "[ODOM] DATA"
# 显式接受前导 '+'：固件现在的 DATA 行用 %.2f 不带正号，但哪天想让它对齐好看
# 改成 %+.2f，不带 [+-] 的正则会把 +0.50 整条漏掉 —— 值静默变成默认 0.0，
# 面板上显示成 +0.00 看着还挺合理，最难查。多一个字符堵掉这类静默失败。
KV_RE = re.compile(r"([A-Za-z_]\w*)=([+-]?\d+(?:\.\d+)?)")

PLOT_SAMPLES = 300          # 曲线保留的采样点数（[DATA] 5 行/秒 → 约 60 秒）
PLOT_REFRESH_MS = 100       # 重绘间隔
POLL_MS = 40                # 主线程取队列间隔
STARTUP_GRACE_MS = 3500     # 开串口后板子会复位，留出启动时间

C_BG = "#1e2229"
C_PANEL = "#272c35"
C_TEXT = "#d8dee9"
C_DIM = "#8b95a5"
C_ACCENT = "#4c9aff"
C_LEFT = "#4c9aff"
C_RIGHT = "#ffb454"
C_GZ = "#7ee787"
C_YAW = "#c792ea"
C_TARGET = "#6b7686"
C_GRID = "#343a45"
C_WARN = "#ff6b6b"
C_TRAIL = "#4c9aff"    # 走过的轨迹
C_ROBOT = "#7ee787"    # 当前位置 + 朝向箭头
C_ORIGIN = "#8b95a5"   # 原点十字（每次「归零」都会落在这里）

# 曲线画哪几条线：(键名, 颜色, 虚线样式)。列表顺序 = 绘制顺序，后画的盖在上面。
SPEED_SERIES = [
    ("tgt_l", C_TARGET, (4, 4)),
    ("tgt_r", C_TARGET, (4, 4)),
    ("speed_l", C_LEFT, None),
    ("speed_r", C_RIGHT, None),
]
GYRO_SERIES = [
    ("gx", C_LEFT, None),
    ("gy", C_RIGHT, None),
    ("gz", C_GZ, None),
]
# yaw 复用 gz 的绿色：物理上是同一个量的「积分前 / 积分后」，同色便于对着看
#
# ytgt 是定角度转向的目标角，灰虚线、画在最底层：定角度转向时能直接看到
# yaw 这条线朝目标线收敛的过程（冲没冲过头、最后停在哪，一眼就看得出来）。
# 不在转向时它保持上一轮的目标值，所以是一条平的灰线，不影响看 roll/pitch。
ATTITUDE_SERIES = [
    ("ytgt", C_TARGET, (4, 4)),
    ("roll", C_LEFT, None),
    ("pitch", C_RIGHT, None),
    ("yaw", C_GZ, None),
]

# CSV 固定表头：底盘行填前半段、IMU 行填中间、姿态行填最后，缺的留空。
# 写死表头而不是「按第一行推断」，是为了让三种数据源能混在同一个文件里
# 而不丢列 —— 记录中途打开 IMU/姿态上报也不会把后面的数据丢掉。
CSV_FIELDS = [
    "t",
    "enc_l", "enc_r", "speed_l", "speed_r", "tgt_l", "tgt_r", "pwm_l", "pwm_r",
    "ax", "ay", "az", "amag", "gx", "gy", "gz", "temp", "bias",
    "roll", "pitch", "yaw", "ytgt", "ybusy",
    # 里程计。x/y/th 是解算出的位姿，dist 是原始累计路程、标定刻度系数时只看它。
    # 这些键名和固件 [ODOM] DATA 行里的一致，也和 [DATA]/[ATT] 的键不冲突（查过）。
    "x", "y", "th", "dist", "v", "w", "scale",
]


# ============================================================================
#  指令协议层 —— 「界面要干什么」和「线上发什么字节」在这里分家
#
#    界面只调用 proto.move() / proto.stop() 这类语义接口，不拼字节。
#    FrameProtocol 把它翻成二进制帧，界面代码一行都不用改。
#
#    这一层以前有两个实现（TextProtocol / FrameProtocol），由界面二选一。
#    文本那条已经整个删掉了：固件侧的下行通道只剩数据帧，留着它就是个能编译
#    但一发过去就没人认的死路。要加新的下行能力，加一个 FrameProtocol 方法 +
#    到固件 handle_frame() 里加一个 case，两边对上功能码即可。
# ============================================================================


# ============================================================================
#  数据帧编码 —— 开发流程.md 附录「工业级串口帧标准结构」
#
#    偏移  字段        字节   说明
#     0    帧头        2      0xEB 0x90
#     2    设备地址     1      0x01~0xFE 单播，0xFF 广播
#     3    功能码       1      bit7 = 方向位：0 = 上位机→下位机，1 = 反向
#     4    帧序号       1      0~255 循环自增
#     5    数据长度     1      数据域字节数（0~255）
#     6    数据域       N      小端
#    6+N   CRC16       2      CRC16-Modbus，小端
#
#    全帧小端。CRC 范围 = 设备地址 .. 数据域末尾（**不含帧头**，也不含 CRC 自己）。
#
#    帧尾 0x7E 本工程**不启用**：接收端按「帧头 + 长度」就能定界，多一个字节反而
#    多一个要处理的转义问题 —— 数据域里只要出现一次 0x7E，接收端就会把它当成帧尾。
# ============================================================================

FRAME_HEAD = b"\xEB\x90"   # 0/1 跳变丰富，且不是 AA55 那类常见同步字，抗噪声误触发
FRAME_ADDR_DEFAULT = 0x01  # 单机场景。地址字段现在就占着位，是为了以后挂传感器节点

# ---- 功能码 ----
# 最高位是方向位，所以**请求码只占 0x00~0x7F**，应答码 = 请求码 | 0x80。
# 按高半字节分域（附录要求「划分功能码域…预留扩展位」）：
#
#     0x01~0x1F  底盘控制类       <- 本阶段全部实现
#     0x20~0x3F  传感器 / 配置类  <- 本阶段全部实现
#     0x40~0x5F  未分配           （预留）
#     0x60~0x6F  异常告警类       （预留）
#     0x70~0x7F  心跳保活类       （预留）
#
# 0x00 不用：全零的功能码在噪声里太容易凑出来。
#
# ⚠ 这张表是**上位机和下位机的共同契约**，改一个数两边都得改。
#   改完要重跑 串口通信手册.md 里那 13 条样例帧 —— 它们把功能码的完整字节钉死了，
#   对不上就说明两边对协议的理解决裂了（固件那边的同一张表在 src/main.cpp）。
FC_CHASSIS_DRIVE = 0x01  # 数据域: v(int16 mm/s), w(int16 mrad/s), 时长(uint16 ms)
FC_CHASSIS_STOP = 0x02   # 数据域: 空
FC_TURN_ANGLE = 0x03     # 数据域: 目标角(int16 mrad，带符号，正 = 左转)
FC_YAW_ABORT = 0x04      # 数据域: 空
FC_CHASSIS_PID = 0x05    # 数据域: Kp, Ki, Kd 各 uint16（增益 ×1000）
FC_IMU_READ = 0x20        # 数据域: 空
FC_IMU_REPORT = 0x21      # 数据域: uint8，0 = 关 / 1 = 开
FC_IMU_CALIBRATE = 0x22   # 数据域: uint16 采样数（0 = 用固件默认）
FC_I2C_SCAN = 0x23        # 数据域: 空
FC_ATT_READ = 0x24        # 数据域: 空
FC_ATT_REPORT = 0x25      # 数据域: uint8，0 = 关 / 1 = 开
FC_ATT_ZERO = 0x26        # 数据域: 空
FC_ODOM_READ = 0x27       # 数据域: 空
FC_ODOM_REPORT = 0x28     # 数据域: uint8，0 = 关 / 1 = 开
FC_ODOM_RESET = 0x29      # 数据域: 空
FC_ODOM_SET_SCALE = 0x2A  # 数据域: uint16 刻度（×10000）
FC_YAW_PID = 0x2B         # 数据域: Kp, Ki, Kd 各 uint16（增益 ×10000）—— 外环，别和 0x05 混
FC_FRAME_STATS = 0x2C     # 数据域: 空
FC_DATA_REPORT = 0x2D     # 数据域: uint8，0 = 关 / 1 = 开
FC_ODOM_GET_SCALE = 0x2E  # 数据域: 空

# ---- 物理量量化 ----
# 附录要求「物理量统一量化为整数传输（速度用 mm/s、角度用 mrad），避免浮点兼容性问题」。
# 为什么值得费这个事：浮点在两端都可能有差异（ESP32-S3 的 double 是软件模拟的、
# 编译器优化级别不同、结构体对齐不同、字长不同），传整数就是传整数 ——
# 同样的物理量，两边算出来的字节一定一样。
MRAD_PER_DEG = math.pi / 180.0 * 1000.0  # 1° = 17.4533 mrad

# 满油门对应的线速度，必须和固件 lib/chassis/src/chassis.h 的
# CHASSIS_MAX_SPEED_CM_S 保持一致 —— 协议里速度是**物理量**（mm/s），
# 总得有一处说清楚「界面的 100% 是多少 mm/s」。界面上那行「50% (15.0 cm/s)」
# 也走这个常数，两处不会各说各话。
CHASSIS_MAX_SPEED_CM_S = 30.0

# 「100% 转向」在协议里表示多少 mrad/s。
#
# 不是拍脑袋填的，是从一次实测反推的：定时开环转 2000ms、速度档 50%，
# 实测转了 284.5° → 142.25 °/s；满油门（100%）翻倍 = 284.5 °/s ≈ 4966 mrad/s，
# 取整 5000（≈286 °/s）。
#
# ⚠ 但它仍然是**标称值**，不是标定值。它只决定「100% 转向」写成多少 mrad/s，
#   固件用同一个常数反算，两边对上就行。想按物理量精确控角速度得先量出轮距 L
#   （odometry.h 里也还标着「轮距没标定」），到时候这个常数和固件一起改。
CHASSIS_SPIN_FULL_SCALE_MRAD_S = 5000.0


def crc16_modbus(data: bytes) -> int:
    """CRC16-Modbus：多项式 0xA001（0x8005 反射），初值 0xFFFF，无最终异或。

    逐位算，没查表 —— 一帧最多 261 字节 × 8 位，在 PC 上快到可以忽略，
    而查表版要带一张 256 项的常量表，读代码的人得先相信那张表是对的。
    这里 6 行代码自己就把算法说完了，而且**固件里可以写成一模一样的 6 行** ——
    两边照抄同一段，比两边各自查表更不容易岔开。

    自检值：crc16_modbus(b"123456789") == 0x4B37（CRC-16/MODBUS 的标准校验值）。
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _u8(flag) -> bytes:
    """1 字节开关量。只用来传 0 / 1 —— 不接数值，免得和 _u16 的用法混淆。"""
    return bytes((1 if flag else 0,))


def _i16(value: float) -> bytes:
    """有符号 16 位小端。超范围夹住，不抛异常。

    夹住而不是报错，是因为这里夹一次、固件还会再夹一次 —— 与其在界面上弹一个
    用户看不懂的溢出框，不如让他看到车按量程上限跑（然后自己把参数调回来）。
    """
    return int(max(-32768, min(32767, round(value)))).to_bytes(2, "little", signed=True)


def _u16(value: float) -> bytes:
    return int(max(0, min(65535, round(value)))).to_bytes(2, "little")


def arc_omega_mrad_s(percent: float, radius_cm: float) -> float:
    """走一段半径 radius_cm 的圆弧时，角速度该给多少 mrad/s。

    纯运动学，没有拟合常数：ω[rad/s] = v[m/s] / r[m]。把 v 换成 mm/s、
    r 换成 cm 代进去，两个 1000 约掉，就是

        w[mrad/s] = 1000 * v[mm/s] / r[mm]

    量一下就知道这个数很小：50% 档（150mm/s）走 30cm 半径，w 只有 500mrad/s
    —— 满油门的 10%。**「转弯」在协议里用的刻度比直觉小得多**，
    所以这里必须算，不能拍一个看着像的数。
    """
    if radius_cm <= 0:
        raise ValueError("圆弧半径必须大于 0")
    v_mm_s = percent / 100.0 * CHASSIS_MAX_SPEED_CM_S * 10.0
    return 1000.0 * v_mm_s / (radius_cm * 10.0)


def build_frame(func: int, payload: bytes = b"", seq: int = 0,
                addr: int = FRAME_ADDR_DEFAULT) -> bytes:
    """按附录结构拼一帧。"""
    if func & 0x80:
        raise ValueError(f"上位机只能发请求码（bit7 必须为 0），收到 0x{func:02X}")
    if len(payload) > 255:
        raise ValueError(f"数据域最长 255 字节，收到 {len(payload)}")

    body = bytes((addr & 0xFF, func, seq & 0xFF, len(payload))) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little")


class FrameProtocol:
    """工业级串口帧协议（结构见 串口通信手册.md）。

    没有「当前速度」这种跨帧状态。对比一下以前那条文本通道：`f 2000` 只说了
    「前进、两秒」，速度是上一条 `v 50` 留下来的 —— 所以每条运动指令都得配一条
    `v` 跟在后头，两条之间还可能被别的东西插队。一条帧就把「v 多少、w 多少、
    持续多久」全说完：没有跨帧状态，也就没有「忘了先设速度」这种错法。
    界面上的参数（速度、时长、角度、PID、刻度）全部直接进数据域，一次一键一帧。

    上报开关也是**置位**不是切换：0x21 / 0x25 / 0x28 / 0x2D 的数据域里直接带
    目标状态，发几次结果都一样，不用界面记着固件现在是什么状态。
    """

    def __init__(self, addr: int = FRAME_ADDR_DEFAULT) -> None:
        self.addr = addr
        self.seq = 0

    # ---- 帧骨架 ----

    def _next_seq(self) -> int:
        """帧序号，0~255 循环自增。

        附录还要求「同一指令**重发**时序号不变，下位机对同序号控制指令只执行 1 次」。
        重传功能还没做（现在发一次算一次），所以这里就是个单纯的自增计数器；
        以后加重传时注意：重发**不能**再调这个函数，得沿用上一次的号，
        否则下位机会当成新指令再执行一遍，序号防重放就白设了。
        """
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return seq

    def _frame(self, func: int, payload: bytes = b"") -> list[bytes]:
        return [build_frame(func, payload, self._next_seq(), self.addr)]

    # ---- 底盘控制类 ----

    def move(self, code: str, percent: int, duration_ms: int) -> list[bytes]:
        """方向码 → 一条底盘速度帧。

        前进/后退 = 纯线速度，原地左/右转 = 纯角速度，四个方向码是同一个功能码
        0x01 的四组 (v, w) 取值 —— 底层 chassis_drive(v, w) 本来就是这么定义的
        （lib/chassis/src/chassis.h），这里没有为它们多发明任何东西。

        所以「把 r 的时长从 2000 改成 3000」和「把 w 从 -143° /s 改到 -215° /s」
        在协议里是同一件事：都是改数据域里的数。转角由谁定，是上层的事。
        """
        v_mm_s = percent / 100.0 * CHASSIS_MAX_SPEED_CM_S * 10.0
        w_mrad_s = percent / 100.0 * CHASSIS_SPIN_FULL_SCALE_MRAD_S
        dirs = {
            "f": (+v_mm_s, 0.0),          # 前进：纯线速度
            "b": (-v_mm_s, 0.0),          # 后退
            "l": (0.0, +w_mrad_s),        # 原地左转：纯角速度
            "r": (0.0, -w_mrad_s),
        }
        if code not in dirs:
            raise ValueError(f"未知方向码 {code!r}")
        v, w = dirs[code]
        return self._frame(FC_CHASSIS_DRIVE,
                           _i16(v) + _i16(w) + _u16(duration_ms))

    def stop(self) -> list[bytes]:
        return self._frame(FC_CHASSIS_STOP)

    def set_pid(self, kp: float, ki: float, kd: float) -> list[bytes]:
        # 增益 ×1000 量化。固件默认 2.5 / 0.65 / 0.5 → 2500 / 650 / 500。
        # 三位小数够了：再细的位 PID 也分辨不出来，而且窗口里本来也只让填到三位。
        return self._frame(FC_CHASSIS_PID,
                           _u16(kp * 1000.0) + _u16(ki * 1000.0) + _u16(kd * 1000.0))

    def set_yaw_pid(self, kp: float, ki: float, kd: float) -> list[bytes]:
        """yaw 外环的 PID，和 set_pid 的轮速内环**不是一回事**。

        改哪个环要看现象：轮速跟不上目标调内环，转过头 / 停不准调这个外环。
        量纲也不同 —— yaw 环的 Kp 是「油门/度」，默认 0.008 这个量级，
        填成 2.5 那种数会直接把车甩出去。

        ⚠ 量化是 ×10000，**不是** set_pid 那个 ×1000。yaw 环的三个数小三个
        数量级（默认 0.008 / 0.020 / 0.0020），×1000 的话 Kd 只剩 2 一格，
        分辨率 0.001 等于默认值的一半 —— 那个旋钮就废了。
        """
        return self._frame(FC_YAW_PID,
                           _u16(kp * 10000.0) + _u16(ki * 10000.0) + _u16(kd * 10000.0))

    def data_report(self, on: bool) -> list[bytes]:
        """开关 [DATA] 底盘数据流（5Hz）。默认就是开的，关掉它串口会安静很多。"""
        return self._frame(FC_DATA_REPORT, _u8(on))

    def frame_stats(self) -> list[bytes]:
        """让固件打一遍收帧统计。

        ⚠ 有个循环依赖：**帧本身收不进来的时候，这条帧也发不进去**。所以固件那边
        补了一个主动告警 —— 错误计数器一涨就自己打 [FRM] 警告，不用谁来问。
        这个按钮是「我主动想看一眼」时用的，不是唯一手段。
        """
        return self._frame(FC_FRAME_STATS)

    # ---- 传感器 / 配置类 ----

    def imu_read(self) -> list[bytes]:
        return self._frame(FC_IMU_READ)

    def imu_report(self, on: bool) -> list[bytes]:
        return self._frame(FC_IMU_REPORT, _u8(on))

    def imu_calibrate(self, samples: int = 0) -> list[bytes]:
        # 0 = 用固件默认采样数（跟文本协议的裸 `c` 一个意思）
        return self._frame(FC_IMU_CALIBRATE, _u16(samples))

    def scan_i2c(self) -> list[bytes]:
        return self._frame(FC_I2C_SCAN)

    def att_read(self) -> list[bytes]:
        return self._frame(FC_ATT_READ)

    def att_report(self, on: bool) -> list[bytes]:
        return self._frame(FC_ATT_REPORT, _u8(on))

    def att_zero(self) -> list[bytes]:
        return self._frame(FC_ATT_ZERO)

    def turn_angle(self, deg: float) -> list[bytes]:
        # 度 → 毫弧度。带符号：正 = 左转，负 = 右转，与固件 yaw 的正方向一致。
        return self._frame(FC_TURN_ANGLE, _i16(deg * MRAD_PER_DEG))

    def arc(self, percent: int, radius_cm: float, duration_ms: int) -> list[bytes]:
        """圆弧 = 一条 v 和 w **同时非零**的 0x01。

        和 move() 的关系：move() 的四个方向码各有一维是 0（纯前进或纯原地转），
        圆弧是第五种拼法，功能码还是 0x01 —— 协议里本来就没有「圆弧」这个东西，
        有的只是 (v, w) 两个数。**这也是当初把 (v,w) 而不是左右轮速当接口的回报**：
        圆弧不需要新功能码，也不需要新固件逻辑。

        w 超满油门角速度就夹住。夹住之后实际半径会**比要的大**，这个差额
        由调用方报给用户（见 CarConsole.do_arc）—— 不夹的话 _i16 会把它裹成
        一个方向相反的数，车朝另一边拐，那是「静默的错误」。
        """
        w = arc_omega_mrad_s(percent, radius_cm)
        w = max(-CHASSIS_SPIN_FULL_SCALE_MRAD_S, min(CHASSIS_SPIN_FULL_SCALE_MRAD_S, w))
        v = percent / 100.0 * CHASSIS_MAX_SPEED_CM_S * 10.0
        return self._frame(FC_CHASSIS_DRIVE,
                           _i16(v) + _i16(w) + _u16(duration_ms))

    def yaw_abort(self) -> list[bytes]:
        return self._frame(FC_YAW_ABORT)

    def uturn(self) -> list[bytes]:
        """掉头 = 转到 +180°，就是 turn_angle(180)，不另占一个功能码。

        固件里 `u` 和 `g 180` 走的本来就是同一条路径（yaw 闭环到 +180°），
        协议层没必要为它多开一个码 —— 少一个码，固件就少一处要实现的逻辑，
        也少一处两边可能理解不一致的地方。
        """
        return self.turn_angle(180.0)

    def odom_read(self) -> list[bytes]:
        return self._frame(FC_ODOM_READ)

    def odom_report(self, on: bool) -> list[bytes]:
        return self._frame(FC_ODOM_REPORT, _u8(on))

    def odom_reset(self) -> list[bytes]:
        return self._frame(FC_ODOM_RESET)

    def odom_set_scale(self, scale: float) -> list[bytes]:
        # 刻度 ×10000 量化（1.0000 → 10000）。固件的合法范围是 0.2~5.0，
        # 也就是 2000~50000 —— 超了 int16 上限，所以这个字段用 uint16。
        # 不在这里夹范围：让固件去夹，上位机把用户输入原样发过去，
        # 免得以后固件放宽范围时这里成了看不见的天花板。回读值才是真相。
        return self._frame(FC_ODOM_SET_SCALE, _u16(scale * 10000.0))

    def odom_get_scale(self) -> list[bytes]:
        """只读当前刻度。标定时用来确认写进去的到底被夹成了多少。"""
        return self._frame(FC_ODOM_GET_SCALE)


class SerialLink:
    """串口收发 + 后台读取线程，收到的行丢进队列由主线程消费。"""

    def __init__(self, out_queue: queue.Queue):
        self.ser: serial.Serial | None = None
        self.queue = out_queue
        self.thread: threading.Thread | None = None
        self.stop = threading.Event()

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def open(self, port: str, baud: int) -> None:
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.stop.clear()
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self) -> None:
        while not self.stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                if not self.stop.is_set():
                    self.queue.put(("error", f"串口读取中断：{exc}"))
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self.queue.put(("line", line))

    def send(self, payload: bytes) -> None:
        """原样写字节。

        下行只有帧这一种东西，所以不做任何编码 —— 尤其**不能**补回车：帧的
        数据域里完全可能出现 0x0A，多插一个字节进去会让固件的 CRC 校验失败，
        而现象只是「发过去的指令没反应」，不报任何错。

        读取方向仍然是按行：板子回的始终是 [CMD] / [YAW] / [FRM] 这类文本日志，
        帧协议只统一了「上位机 → 下位机」这一个方向。
        """
        if not self.connected:
            raise RuntimeError("串口未连接")
        self.ser.write(payload)

    def close(self) -> None:
        self.stop.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


class RollingPlot(tk.Canvas):
    """滚动曲线：给一组 (键名, 颜色, 虚线) 就画，纵轴量程按采样峰值自适应。

    速度曲线和陀螺仪曲线共用这一个类，区别只在 series 传什么。
    """

    def __init__(self, master, series, min_span=5.0, height=180,
                 empty_text="等待数据…", **kw):
        super().__init__(master, bg=C_PANEL, highlightthickness=0, height=height, **kw)
        self.series = series
        self.min_span = min_span
        self.empty_text = empty_text
        self.samples: collections.deque = collections.deque(maxlen=PLOT_SAMPLES)
        self.span = min_span
        self.bind("<Configure>", lambda _e: self.redraw())

    def add(self, kv: dict) -> None:
        self.samples.append(kv)

    def clear(self) -> None:
        self.samples.clear()
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 10 or h < 10:
            return

        # 纵轴量程自适应：取所有采样里数值绝对值的峰值，向上取整到 5 的倍数
        peak = self.min_span
        for s in self.samples:
            for key, _color, _dash in self.series:
                peak = max(peak, abs(s.get(key, 0.0)))
        self.span = max(self.min_span, ((peak * 1.15) // 5 + 1) * 5)

        pad_l, pad_b, pad_t = 34, 16, 8
        x0, y0 = pad_l, pad_t
        x1, y1 = w - 8, h - pad_b

        def y_of(v: float) -> float:
            half = (y1 - y0) / 2
            return (y0 + y1) / 2 - v / self.span * half

        # 网格 + 纵轴刻度（±span、0 三条实线刻度，中间两条虚线）
        for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
            v = frac * self.span
            y = y_of(v)
            self.create_line(x0, y, x1, y, fill=C_GRID, dash=() if frac == 0 else (2, 4))
            self.create_text(x0 - 5, y, text=f"{v:.0f}", anchor="e",
                             fill=C_DIM, font=("Consolas", 8))

        n = len(self.samples)
        if n < 2:
            self.create_text(w / 2, h / 2, text=self.empty_text, fill=C_DIM,
                             font=("Microsoft YaHei", 10))
            return

        step = (x1 - x0) / (PLOT_SAMPLES - 1)

        def draw(key: str, color: str, dash=None) -> None:
            pts = []
            for i, s in enumerate(self.samples):
                if key in s:
                    pts.append(x0 + i * step)
                    pts.append(y_of(s[key]))
            if len(pts) >= 4:
                self.create_line(*pts, fill=color, width=2 if dash is None else 1, dash=dash)

        for key, color, dash in self.series:
            draw(key, color, dash)


class TrajectoryPlot(tk.Canvas):
    """XY 轨迹图：把里程计解算出的位姿画成一条走过的路。

    ---- 为什么两轴必须等比例 ----
    横轴纵轴共用同一个「每厘米多少像素」。否则走正方形会画成长方形、走圆会画成
    椭圆 —— 这跟数值对不对无关，是**看着像不像**的问题：一旦两轴比例不同，
    肉眼就没法判断「走回原点了吗」「这个弯是 90° 吗」，这张图就白画了。
    所以按包围盒的较大边定比例，两轴共用。

    ---- 视野 ----
    自适应该轨迹的包围盒，但**原点是硬塞进包围盒的**：不这么做的话车一跑远
    原点就被挤出画面，而「有没有回到起点」正是这张图最该回答的问题。
    另外有个最小视野，否则车没动时包围盒是 0，除下来比例无穷大、画面抖成一团。
    """

    MIN_SPAN_CM = 40.0   # 最小视野（cm）
    MAX_POINTS = 4000    # 轨迹点上限：5Hz 跑一小时约 18000 帧，不截会越跑越卡
    PAD = 20             # 画布内边距（像素）

    def __init__(self, master, height=330,
                 empty_text="等里程计数据…按「归零」把当前位置记为原点", **kw):
        super().__init__(master, bg=C_PANEL, highlightthickness=0, height=height, **kw)
        self.empty_text = empty_text
        self.trail: collections.deque = collections.deque(maxlen=self.MAX_POINTS)
        self.pose = None   # 最近一帧 (x, y, th)
        self.bind("<Configure>", lambda _e: self.redraw())

    def add(self, kv: dict) -> None:
        pose = (kv.get("x", 0.0), kv.get("y", 0.0), kv.get("th", 0.0))
        self.pose = pose
        # 车停着时 5Hz 上报会灌一堆完全重合的点，只更新朝向不追加
        if self.trail and self.trail[-1][:2] == pose[:2]:
            self.trail[-1] = pose
            return
        self.trail.append(pose)

    def clear(self) -> None:
        self.trail.clear()
        self.pose = None
        self.redraw()

    @staticmethod
    def _nice_step(span: float) -> float:
        """把视野切成几格，格宽取 1/2/5 × 10ⁿ —— 网格标签是 5/10/20 这种整数才好读。"""
        raw = span / 6.0
        mag = 10.0 ** math.floor(math.log10(raw))
        for m in (1.0, 2.0, 5.0):
            if raw <= m * mag:
                return m * mag
        return 10.0 * mag

    def redraw(self) -> None:
        self.delete("all")
        w = self.winfo_width()
        h = self.winfo_height()
        if w < 10 or h < 10:
            return

        if not self.trail:
            self.create_text(w / 2, h / 2, text=self.empty_text, fill=C_DIM,
                             font=("Microsoft YaHei", 10))
            return

        # 包围盒：轨迹 + 原点（原点必须在内，见类注释）
        xs = [p[0] for p in self.trail] + [0.0]
        ys = [p[1] for p in self.trail] + [0.0]
        cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
        span = max(self.MIN_SPAN_CM, max(xs) - min(xs), max(ys) - min(ys)) * 1.15

        # 两轴共用 k —— 等比例的落点就在这一行
        k = min((w - 2 * self.PAD) / span, (h - 2 * self.PAD) / span)

        def sx(x: float) -> float:
            return w / 2 + (x - cx) * k

        def sy(y: float) -> float:
            # 屏幕 y 向下、世界 y 向左，所以取负
            return h / 2 - (y - cy) * k

        # ---- 网格 ----
        step = self._nice_step(span)
        g = math.floor((cx - span / 2) / step) * step
        while g <= cx + span / 2 + 1e-9:
            x = sx(g)
            if self.PAD <= x <= w - self.PAD:
                self.create_line(x, self.PAD, x, h - self.PAD, fill=C_GRID)
                self.create_text(x, h - self.PAD + 8, text=f"{g:.0f}", fill=C_DIM,
                                 font=("Consolas", 8))
            g += step

        g = math.floor((cy - span / 2) / step) * step
        while g <= cy + span / 2 + 1e-9:
            y = sy(g)
            if self.PAD <= y <= h - self.PAD:
                self.create_line(self.PAD, y, w - self.PAD, y, fill=C_GRID)
                self.create_text(self.PAD - 5, y, text=f"{g:.0f}", anchor="e",
                                 fill=C_DIM, font=("Consolas", 8))
            g += step

        # ---- 原点十字（每次「归零」都落在这里，回来没回来一眼可见）----
        ox, oy = sx(0.0), sy(0.0)
        self.create_line(ox - 6, oy, ox + 6, oy, fill=C_ORIGIN, width=2, tags="origin")
        self.create_line(ox, oy - 6, ox, oy + 6, fill=C_ORIGIN, width=2, tags="origin")

        # ---- 轨迹 ----
        pts = []
        for x, y, _th in self.trail:
            pts.append(sx(x))
            pts.append(sy(y))
        if len(pts) >= 4:
            self.create_line(*pts, fill=C_TRAIL, width=2, capstyle="round", tags="trail")

        # ---- 当前位置 + 朝向箭头 ----
        x, y, th = self.pose
        px, py = sx(x), sy(y)
        rad = math.radians(th)
        ln = max(12.0, k * 8.0)   # 箭头约代表 8cm，太短看不见
        self.create_line(px, py, px + math.cos(rad) * ln, py - math.sin(rad) * ln,
                         fill=C_ROBOT, width=3, arrow="last", arrowshape=(11, 13, 4),
                         tags="robot")
        self.create_oval(px - 3, py - 3, px + 3, py + 3, fill=C_ROBOT, outline="",
                         tags="robot")

        # ---- 比例说明 ----
        self.create_text(w - self.PAD, self.PAD - 8,
                         text=f"1 格 = {step:g} cm", anchor="e", fill=C_DIM,
                         font=("Microsoft YaHei", 8))


class CarConsole(tk.Tk):
    def __init__(self, default_port: str, default_baud: int):
        super().__init__()
        self.title("ROS-CAR 上位机控制台")
        # 高度跟着屏幕走，不写死。这台机器逻辑分辨率只有 1536x864，任务栏一去
        # 剩约 780 —— 原来写死的 900 意味着底部 120px 永远在屏幕外面。
        # ⚠ Tk 对「窗口比内容矮」**不报任何错**，它只是把超出的部分裁掉：
        # 控件还在、还能点到，但你得先把窗口拖出去才看得见。
        # 所以「某个按钮找不到」时，先量 reqheight 和屏幕高度，别去猜控件。
        screen_h = self.winfo_screenheight()
        h = max(560, min(1000, screen_h - 80))
        self.geometry(f"940x{h}")
        # 最小高度跟上面那个 h 联动，不写死：整个界面实测要 672px 才装得下
        # （串口栏 55 + 运动面板 188 + Notebook 241 + 日志 62 + 标签和间距），
        # 写死 560 的话用户把窗口拖到最小就会裁掉日志。上限 700 是留一点余量，
        # 且保证 minsize 永远不会大于 geometry —— 那会让窗口一开就被撑出屏幕。
        self.minsize(860, min(700, h))
        self.configure(bg=C_BG)

        self.queue: queue.Queue = queue.Queue()
        self.link = SerialLink(self.queue)
        self.connected_at = 0.0
        # 界面发的全是协议帧。固件那边的下行通道也只有这一条 —— 以前还有个文本
        # 指令的对照物，已经删干净了（留着它就是个发过去没人认的死路）。
        self.proto = FrameProtocol()

        self.csv_file = None
        self.csv_writer = None
        self.csv_t0 = 0.0
        self.csv_rows = 0
        self.csv_path = ""

        # 界面自己记着「连续上报开着没」—— 只为了把按钮文字显示对（「开始」还是
        # 「停止」）。固件那边是置位的，不信这个变量也能工作，它错了只会让按钮
        # 文字难看，不会发错指令。
        self.imu_reporting = False
        self.att_reporting = False
        self.odom_reporting = False

        self._build_style()
        self._build_ui(default_port, default_baud)

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(POLL_MS, self.poll_queue)
        self.after(PLOT_REFRESH_MS, self.refresh_plot)

    # ---------------- 界面搭建 ----------------
    def _build_style(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=C_BG, foreground=C_TEXT, font=("Microsoft YaHei", 9))
        style.configure("TFrame", background=C_BG)
        style.configure("Panel.TFrame", background=C_PANEL)
        style.configure("TLabel", background=C_BG, foreground=C_TEXT)
        style.configure("Panel.TLabel", background=C_PANEL, foreground=C_TEXT)
        style.configure("Dim.TLabel", background=C_PANEL, foreground=C_DIM, font=("Microsoft YaHei", 8))
        style.configure("Value.TLabel", background=C_PANEL, foreground=C_TEXT, font=("Consolas", 15, "bold"))
        # 状态标签用独立样式：改它自己的 foreground 不会波及别的 Panel.TLabel
        style.configure("Status.TLabel", background=C_PANEL, foreground=C_DIM)
        style.configure("Title.TLabel", background=C_BG, foreground=C_DIM, font=("Microsoft YaHei", 9, "bold"))
        style.configure("TButton", padding=6)
        style.configure("Move.TButton", font=("Microsoft YaHei", 11, "bold"), padding=10)
        style.configure("Stop.TButton", font=("Microsoft YaHei", 11, "bold"), padding=10)
        style.configure("Imu.TButton", padding=6)
        style.configure("TCheckbutton", background=C_BG, foreground=C_TEXT)
        style.configure("TEntry", fieldbackground=C_PANEL, foreground=C_TEXT)
        style.configure("Horizontal.TScale", background=C_BG)
        # 标签页：深色主题下默认的灰白标签很刺眼，改成和面板同色系
        style.configure("TNotebook", background=C_BG, borderwidth=0, tabmargins=(0, 4, 0, 0))
        style.configure("TNotebook.Tab", background=C_PANEL, foreground=C_DIM, padding=(16, 6))
        style.map("TNotebook.Tab",
                  background=[("selected", C_BG)],
                  foreground=[("selected", C_ACCENT)])

    def _panel(self, parent, title: str) -> ttk.Frame:
        ttk.Label(parent, text=title, style="Title.TLabel").pack(anchor="w", pady=(10, 3))
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=10)
        frame.pack(fill="x")
        return frame

    def _scrollable_tab(self, nb: ttk.Notebook, text: str, min_height: int = 200):
        """加一个**可滚动**的标签页，返回该往里塞内容的容器。

        为什么非做不可：这台机器屏幕逻辑高度只有 864，任务栏一去，窗口最多到
        ~780。而 IMU 页的内容（操作面板 + 数字面板 + 两条曲线）自己就要 690 ——
        Notebook 的高度取**所有标签页里最高的那个**，再加顶上的串口/运动面板和
        底下的日志，一共要 1131。Tk 不会因此报错，它只是**把超出的部分裁掉**：
        控件还在、还能点，但你得把窗口拖出屏幕才够得着。

        这种「内容静默消失」没有任何提示，所以量 reqheight 和屏幕高度应该是
        排查「某个按钮找不到」的第一步 —— 别去猜是不是控件忘了 pack。

        min_height 是给画布要一个**小**的 reqheight 用的：Canvas 默认按内容
        撑高，那就等于没滚动。给个小值，Notebook 就只请求这么高，
        再用 expand 吃掉窗口的剩余空间，每个标签页各自滚。
        """
        outer = ttk.Frame(nb)
        nb.add(outer, text=text)

        canvas = tk.Canvas(outer, bg=C_BG, highlightthickness=0, height=min_height)
        vsb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = ttk.Frame(canvas)
        win = canvas.create_window((0, 0), window=inner, anchor="nw")

        # 内框宽度跟着画布走。不跟的话内容会按自己的 reqwidth 排，
        # 铺不满宽度、右边的控件被挤出可视区 —— 那是另一个「找不到按钮」。
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(win, width=e.width))
        inner.bind("<Configure>",
                   lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.configure(scrollregion=canvas.bbox("all"))

        # 滚轮只在指针停在这一页时接管。全局绑定会让鼠标在别的页面上
        # 也在滚这一页，而且会和日志区自己的滚动打架。
        def _wheel(e):
            canvas.yview_scroll(-1 if e.delta > 0 else 1, "units")
        canvas.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", _wheel))
        canvas.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))
        return inner

    def _readout(self, parent, name: str, key: str, width: int = 9,
                 store: dict | None = None) -> tk.Label:
        """一行「名字  值」。返回值那个 Label，后续直接 configure(text=)。

        store 指定把 Label 登记进哪个字典（默认 imu_lbls，姿态那栏传 att_lbls）。
        """
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(anchor="w", fill="x")
        tk.Label(row, text=name, bg=C_PANEL, fg=C_DIM, width=5, anchor="w",
                 font=("Microsoft YaHei", 9)).pack(side="left")
        value = tk.Label(row, text="—", bg=C_PANEL, fg=C_TEXT, width=width, anchor="w",
                         font=("Consolas", 11, "bold"))
        value.pack(side="left")
        (self.imu_lbls if store is None else store)[key] = value
        return value

    def _build_ui(self, default_port: str, default_baud: int) -> None:
        root = ttk.Frame(self, padding=12)
        root.pack(fill="both", expand=True)

        # ---- 串口连接 ----
        bar = self._panel(root, "串口连接")
        ttk.Label(bar, text="端口", style="Panel.TLabel").pack(side="left")
        self.port_var = tk.StringVar(value=default_port)
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=12,
                                     state="readonly")
        self.port_box.pack(side="left", padx=(6, 6))
        ttk.Button(bar, text="刷新", command=self.refresh_ports).pack(side="left")
        self.connect_btn = ttk.Button(bar, text="连接", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=8)
        self.status_var = tk.StringVar(value="未连接")
        self.status_lbl = ttk.Label(bar, textvariable=self.status_var, style="Status.TLabel")
        self.status_lbl.pack(side="left", padx=8)
        # 帧统计放在这一条上，不放标签页里：它诊断的是**链路**（帧到没到、CRC 对不对），
        # 和「下行协议」这句话是同一件事，挨着放才对。而且这条栏永远可见 ——
        # 帧收不到的时候最需要点它，把它埋进标签页底部等于在最需要的时候藏起来。
        ttk.Button(bar, text="帧统计", command=self.do_frame_stats).pack(side="right")
        ttk.Label(bar, text="下行协议：数据帧", style="Dim.TLabel").pack(side="right", padx=(0, 10))
        self.refresh_ports()

        # ---- 运动控制（常驻：开车的时候也要够得着急停）----
        ctrl = self._panel(root, "运动控制")

        row = ttk.Frame(ctrl, style="Panel.TFrame")
        row.pack(fill="x", pady=(0, 8))
        ttk.Label(row, text="速度", style="Panel.TLabel").pack(side="left")
        self.speed_var = tk.IntVar(value=50)
        self.speed_scale = ttk.Scale(row, from_=1, to=100, orient="horizontal")
        self.speed_scale.pack(side="left", fill="x", expand=True, padx=8)
        self.speed_lbl = ttk.Label(row, text="50%  (15.0 cm/s)", style="Panel.TLabel", width=20)
        self.speed_lbl.pack(side="left")
        # 回调必须在标签建好之后再挂：Scale.set() 会立刻触发一次回调
        self.speed_scale.configure(command=lambda v: self.on_speed_change(float(v)))
        self.speed_scale.set(50)

        row = ttk.Frame(ctrl, style="Panel.TFrame")
        row.pack(fill="x", pady=(0, 10))
        ttk.Label(row, text="时长", style="Panel.TLabel").pack(side="left")
        self.dur_var = tk.StringVar(value="2000")
        ttk.Entry(row, textvariable=self.dur_var, width=8).pack(side="left", padx=6)
        # 时长管前进/后退/圆弧。转向和掉头都走 yaw 闭环，转多少由角度定、不由时间定。
        ttk.Label(row, text="ms（前进/后退/圆弧）", style="Dim.TLabel").pack(side="left")
        # 圆弧半径和时长同一行，不另起一行 —— 竖向空间已经很紧（见 _scrollable_tab），
        # 能不加行就不加。
        ttk.Label(row, text="半径", style="Panel.TLabel").pack(side="left", padx=(20, 0))
        self.arc_r_var = tk.StringVar(value="30")
        ttk.Entry(row, textvariable=self.arc_r_var, width=6,
                  justify="right").pack(side="left", padx=6)
        ttk.Label(row, text="cm（圆弧）", style="Dim.TLabel").pack(side="left")

        grid = ttk.Frame(ctrl, style="Panel.TFrame")
        grid.pack(fill="x")
        moves = [
            ("▲  前进", lambda: self.move("f", "前进")),
            ("▼  后退", lambda: self.move("b", "后退")),
            # 圆弧：v 和 w 同时非零的唯一入口。上面那些按钮全是「纯前进 / 纯原地转」，
            # 一个圆弧都没有 —— 而里程计的中点积分只有在**边走边转**时才和欧拉积分
            # 分开（原地转时 d_center = 0，两种积分法给出同样的结果）。
            # 所以「走正方形」验不到积分方法，这条路才是里程计真正的验证。
            ("◔  圆弧", self.do_arc),
            # 转向走 yaw 闭环（固件的 0x03），不是定时转 —— 这三个按钮都是「转到一个
            # 角度」，没有一个是「转一段时间」，所以它们都不看上面的时长框。
            #
            # 这不是「闭环更准」的偏好问题，是定时转**根本答不出**「转了多少度」：
            # 转出来的角度由速度档和电池电压决定，跟 90° 毫无关系（实测 50% 档下
            # 按一次约 285°，四次累计 1138°，走正方形画出来是个星形）。
            # 所以凡是「转到某个角度」的需求一律走闭环，没有例外。
            ("↺  左转 90°", lambda: self.do_turn(90)),
            ("↻  右转 90°", lambda: self.do_turn(-90)),
            ("⇅  掉头 180°", self.do_uturn),
        ]
        # 四列两行：六个动作 + 急停（横跨两格）。加成四列是为了塞进圆弧之后
        # **仍然只有两行** —— 三列的话第七个按钮会撑出第三行，窗口又高 38px。
        for i, (text, cmd) in enumerate(moves):
            ttk.Button(grid, text=text, style="Move.TButton", command=cmd).grid(
                row=i // 4, column=i % 4, sticky="ew", padx=4, pady=4
            )
        self.stop_btn = ttk.Button(grid, text="■  急停", style="Stop.TButton", command=self.emergency_stop)
        self.stop_btn.grid(row=1, column=2, columnspan=2, sticky="ew", padx=4, pady=4)
        for c in range(4):
            grid.columnconfigure(c, weight=1)

        # ---- 数据区：底盘 / IMU / 里程计三个标签页 ----
        # 用标签页而不是上下堆叠，是因为窗口高度有限：IMU 面板 + 陀螺仪曲线
        # 直接摊开会让窗口高到屏幕放不下。三个页都是**可滚动**的 ——
        # 屏幕逻辑高度只有 864，光靠压控件尺寸凑不出来，理由见 _scrollable_tab。
        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True)
        self.nb = nb   # 留个引用：要跳到某个标签页时用（比如以后自动切到出问题的那个）

        self._build_chassis_tab(self._scrollable_tab(nb, "  底盘状态  "))
        self._build_imu_tab(self._scrollable_tab(nb, "  IMU  "))
        self._build_odom_tab(self._scrollable_tab(nb, "  里程计  "))

        # ---- 日志 ----
        # 以前日志头上还有一行「指令」输入框，用来手敲 ASCII。删掉了：帧里没有
        # 「自由文本」这种数据域，那一行从挂上 FrameProtocol 起就是禁用态，
        # 现在连它对应的固件入口也没了。
        ttk.Label(root, text="日志", style="Title.TLabel").pack(anchor="w", pady=(10, 3))
        log_frame = ttk.Frame(root, style="Panel.TFrame", padding=1)
        # expand=False 是刻意的：Notebook 和日志都 expand=True 的话，多出来的
        # 竖向空间会在两者间**平分**，日志会白白吃掉几十像素（实测请求 62 拿 118）。
        # 日志固定 4 行，多出来的全给 Notebook。
        log_frame.pack(fill="x", expand=False)

        text_wrap = ttk.Frame(log_frame, style="Panel.TFrame")
        text_wrap.pack(fill="x", expand=False)
        self.log = tk.Text(
            text_wrap, bg=C_PANEL, fg=C_TEXT, insertbackground=C_TEXT,
            relief="flat", height=4, wrap="none", font=("Consolas", 9),
        )
        scroll = ttk.Scrollbar(text_wrap, command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")
        self.log_line(
            "就绪。每次点按钮 = 发一条二进制帧，界面上的参数（速度 / 时长 / 角度 / "
            "PID / 刻度）直接进数据域，日志里 >> 开头的十六进制就是线上真实字节。"
            "板子上电有 3 秒启动延时、接着约 5 秒陀螺零偏标定。",
            dim=True,
        )
        self.log_line(
            "点按钮没反应时：先点顶上串口栏的「帧统计」看固件的收帧计数 ——"
            "「好帧」不涨说明帧没到（波特率 / 共地 / 端口选错），"
            "「CRC 错」一直涨说明字节收到了但内容不对（线太长 / 干扰）。"
            "帧彻底不通时这条查询也发不进去，那种情况固件会自己打 [FRM] 收帧异常告警。",
            dim=True,
        )
        self.log_line(
            "三个标签页都可以滚轮上下滚 —— 窗口矮的时候里面的内容不会消失，"
            "只是要滚一下。曲线都在页面下半部分。",
            dim=True,
        )

    def _build_chassis_tab(self, tab: ttk.Frame) -> None:
        state = self._panel(tab, "实时状态")
        head = ttk.Frame(state, style="Panel.TFrame")
        head.pack(fill="x")
        for col, (title, color) in enumerate((("左轮", C_LEFT), ("右轮", C_RIGHT))):
            box = ttk.Frame(head, style="Panel.TFrame")
            box.grid(row=0, column=col, sticky="ew", padx=(0, 24))
            tk.Label(box, text=title, bg=C_PANEL, fg=color, font=("Microsoft YaHei", 10, "bold")).pack(anchor="w")
            lbl = ttk.Label(box, text="—", style="Value.TLabel")
            lbl.pack(anchor="w")
            sub = ttk.Label(box, text="", style="Dim.TLabel")
            sub.pack(anchor="w")
            if col == 0:
                self.speed_l_lbl, self.sub_l_lbl = lbl, sub
            else:
                self.speed_r_lbl, self.sub_r_lbl = lbl, sub
        for c in range(2):
            head.columnconfigure(c, weight=1)

        self.plot = RollingPlot(state, SPEED_SERIES, min_span=5.0)
        self.plot.pack(fill="both", expand=True, pady=(8, 0))
        ttk.Label(
            state, text="实线 = 实测速度（蓝 左轮 / 橙 右轮）    虚线 = 目标速度    单位 cm/s",
            style="Dim.TLabel",
        ).pack(anchor="w", pady=(2, 0))

        # 两行 PID 各装在**自己的行框**里，不直接挂在面板上。
        # （Tk 的 pack 是按顺序从空腔里切：先挂 side="left" 的一串，空腔就被切成
        #   一条窄缝，后面再挂 side="top"（默认）的行就只能塞进那条窄缝里 ——
        #   结果是它的宽度**加到**面板宽度上而不是换行。这就是 IMU 页那个
        #   row1/row2/row3 的写法为什么是必要的。）
        pid = self._panel(tab, "PID 整定")

        row1 = ttk.Frame(pid, style="Panel.TFrame")
        row1.pack(fill="x")
        self.kp_var = tk.StringVar(value="2.5")
        self.ki_var = tk.StringVar(value="0.65")
        self.kd_var = tk.StringVar(value="0.5")
        for text, var in (("Kp", self.kp_var), ("Ki", self.ki_var), ("Kd", self.kd_var)):
            ttk.Label(row1, text=text, style="Panel.TLabel").pack(side="left", padx=(0, 2))
            ttk.Entry(row1, textvariable=var, width=7).pack(side="left", padx=(0, 12))
        ttk.Button(row1, text="应用", command=self.apply_pid).pack(side="left")
        self.csv_btn = ttk.Button(row1, text="开始记录 CSV", command=self.toggle_csv)
        self.csv_btn.pack(side="right")

        # yaw 外环。和上面那组**不是同一个环**，量纲差三个数量级（上面是轮速内环
        # 2.5 / 0.65 / 0.5，这里是角度外环 0.008 / 0.020 / 0.0020），所以分成两行
        # 而不是并排 —— 并排太容易看串行、把内环的数填进外环。
        row2 = ttk.Frame(pid, style="Panel.TFrame")
        row2.pack(fill="x", pady=(8, 0))
        ttk.Label(row2, text="yaw 环", style="Panel.TLabel").pack(side="left", padx=(0, 6))
        self.ykp_var = tk.StringVar(value="0.008")
        self.yki_var = tk.StringVar(value="0.020")
        self.ykd_var = tk.StringVar(value="0.0020")
        for text, var in (("Kp", self.ykp_var), ("Ki", self.yki_var), ("Kd", self.ykd_var)):
            ttk.Label(row2, text=text, style="Panel.TLabel").pack(side="left", padx=(0, 2))
            ttk.Entry(row2, textvariable=var, width=8).pack(side="left", padx=(0, 12))
        ttk.Button(row2, text="应用", command=self.apply_yaw_pid).pack(side="left")
        ttk.Label(row2, text="改「转过头 / 停不准」调这个；单位是油门/度，数量级比上面小得多",
                  style="Dim.TLabel").pack(side="right")

        # 数据流。帧统计挪到顶上的串口栏了 —— 那条栏永远可见，见 _build_ui。
        diag = self._panel(tab, "数据流")
        row = ttk.Frame(diag, style="Panel.TFrame")
        row.pack(fill="x")
        self.data_report_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(row, text="底盘数据上报 [DATA]（5Hz）", variable=self.data_report_var,
                        command=self.toggle_data_report).pack(side="left")
        ttk.Label(row, text="关掉它串口会安静很多；再点顶上的「帧统计」看收帧计数",
                  style="Dim.TLabel").pack(side="right")

    def _build_imu_tab(self, tab: ttk.Frame) -> None:
        self.imu_lbls: dict[str, tk.Label] = {}
        self.att_lbls: dict[str, tk.Label] = {}

        # 操作按钮：每个都对应 main.cpp 里的一条 IMU 指令。
        # 分两行 —— 七条指令挤一行会超出窗口宽度。
        ops = self._panel(tab, "IMU 操作")
        row1 = ttk.Frame(ops, style="Panel.TFrame")
        row1.pack(fill="x")
        ttk.Button(row1, text="读一帧", style="Imu.TButton",
                   command=lambda: self.send_proto("imu_read", "读一帧 IMU")).pack(side="left", padx=(0, 6))
        self.imu_report_btn = ttk.Button(row1, text="开始连续上报", style="Imu.TButton",
                                         command=self.toggle_imu_report)
        self.imu_report_btn.pack(side="left", padx=(0, 6))
        ttk.Button(row1, text="零偏标定", style="Imu.TButton",
                   command=self.do_calibrate).pack(side="left", padx=(0, 6))
        ttk.Button(row1, text="扫描 I2C 总线", style="Imu.TButton",
                   command=lambda: self.send_proto("scan_i2c", "扫描 I2C 总线")).pack(side="left", padx=(0, 6))
        ttk.Label(row1, text="零偏标定需车体静止约 5 秒", style="Dim.TLabel").pack(side="right")

        row2 = ttk.Frame(ops, style="Panel.TFrame")
        row2.pack(fill="x", pady=(6, 0))
        ttk.Button(row2, text="读一帧姿态", style="Imu.TButton",
                   command=lambda: self.send_proto("att_read", "读一帧姿态")).pack(side="left", padx=(0, 6))
        self.att_report_btn = ttk.Button(row2, text="开始姿态上报", style="Imu.TButton",
                                         command=self.toggle_att_report)
        self.att_report_btn.pack(side="left", padx=(0, 6))
        ttk.Button(row2, text="yaw 归零", style="Imu.TButton",
                   command=lambda: self.send_proto("att_zero", "yaw 归零")).pack(side="left", padx=(0, 6))
        ttk.Label(row2, text="姿态由固件 100Hz 解算，与是否上报无关 —— 关掉上报它也在算",
                  style="Dim.TLabel").pack(side="right")

        # 定角度转向（yaw 闭环）。和上面 l/r/u 的「定时转向」不是一回事：
        # 那三个转多久由时长决定，这三个转多少由 yaw 决定、转到了自己停。
        row3 = ttk.Frame(ops, style="Panel.TFrame")
        row3.pack(fill="x", pady=(6, 0))
        ttk.Label(row3, text="定角度转向", style="Panel.TLabel").pack(side="left", padx=(0, 6))
        self.turn_var = tk.StringVar(value="90")
        ttk.Entry(row3, textvariable=self.turn_var, width=6,
                  justify="right").pack(side="left", padx=(0, 4))
        ttk.Label(row3, text="°", style="Panel.TLabel").pack(side="left", padx=(0, 6))
        ttk.Button(row3, text="转向", style="Imu.TButton",
                   command=self.do_turn_from_entry).pack(side="left", padx=(0, 10))
        for deg in (90, -90, 180):
            ttk.Button(row3, text=f"{deg:+d}°", style="Imu.TButton", width=5,
                       command=lambda d=deg: self.do_turn(d)).pack(side="left", padx=(0, 4))
        ttk.Button(row3, text="中止", style="Imu.TButton",
                   command=lambda: self.send_proto("yaw_abort", "中止定角度转向")
                   ).pack(side="left", padx=(6, 0))
        ttk.Label(row3, text="正=左转，负=右转；需先开启姿态上报才能看到收敛过程",
                  style="Dim.TLabel").pack(side="right")

        # 数字面板
        state = self._panel(tab, "实时数据")
        head = ttk.Frame(state, style="Panel.TFrame")
        head.pack(fill="x")

        acc = ttk.Frame(head, style="Panel.TFrame")
        acc.grid(row=0, column=0, sticky="nw", padx=(0, 18))
        tk.Label(acc, text="加速度 (g)", bg=C_PANEL, fg=C_ACCENT,
                 font=("Microsoft YaHei", 10, "bold")).pack(anchor="w", pady=(0, 2))
        self._readout(acc, "X", "ax")
        self._readout(acc, "Y", "ay")
        self._readout(acc, "Z", "az")

        gyro = ttk.Frame(head, style="Panel.TFrame")
        gyro.grid(row=0, column=1, sticky="nw", padx=(0, 18))
        tk.Label(gyro, text="角速度 (°/s)", bg=C_PANEL, fg=C_GZ,
                 font=("Microsoft YaHei", 10, "bold")).pack(anchor="w", pady=(0, 2))
        self._readout(gyro, "X", "gx")
        self._readout(gyro, "Y", "gy")
        self._readout(gyro, "Z", "gz")

        att = ttk.Frame(head, style="Panel.TFrame")
        att.grid(row=0, column=2, sticky="nw", padx=(0, 18))
        tk.Label(att, text="姿态角 (°)", bg=C_PANEL, fg=C_YAW,
                 font=("Microsoft YaHei", 10, "bold")).pack(anchor="w", pady=(0, 2))
        self._readout(att, "roll", "roll", width=10, store=self.att_lbls)
        self._readout(att, "pitch", "pitch", width=10, store=self.att_lbls)
        self._readout(att, "yaw", "yaw", width=10, store=self.att_lbls)
        self._readout(att, "静止", "still", width=10, store=self.att_lbls)
        # 定角度转向：目标角 + 是否正在转。曲线图上的灰虚线就是 ytgt。
        self._readout(att, "目标", "ytgt", width=10, store=self.att_lbls)
        self._readout(att, "转向", "ybusy", width=10, store=self.att_lbls)

        health = ttk.Frame(head, style="Panel.TFrame")
        health.grid(row=0, column=3, sticky="nw")
        tk.Label(health, text="芯片 / 解算", bg=C_PANEL, fg=C_DIM,
                 font=("Microsoft YaHei", 10, "bold")).pack(anchor="w", pady=(0, 2))
        self._readout(health, "|a|", "amag", width=10)
        self._readout(health, "温度", "temp", width=10)
        self._readout(health, "零偏", "bias", width=10)
        self._readout(health, "频率", "rate", width=10, store=self.att_lbls)
        for c in range(4):
            head.columnconfigure(c, weight=1)

        self.gyro_plot = RollingPlot(state, GYRO_SERIES, min_span=5.0, height=130,
                                     empty_text="等 IMU 数据…按「读一帧」或「开始连续上报」")
        self.gyro_plot.pack(fill="both", expand=True, pady=(8, 0))
        ttk.Label(
            state,
            text="角速度三轴（蓝 X / 橙 Y / 绿 Z）    单位 °/s    "
                 "静止时应贴在 0 附近小幅抖动，持续单调漂移说明零偏没标定好",
            style="Dim.TLabel",
        ).pack(anchor="w", pady=(2, 0))

        self.att_plot = RollingPlot(state, ATTITUDE_SERIES, min_span=20.0, height=130,
                                    empty_text="等姿态数据…按「读一帧姿态」或「开始姿态上报」")
        self.att_plot.pack(fill="both", expand=True, pady=(8, 0))
        ttk.Label(
            state,
            text="姿态角（蓝 roll / 橙 pitch / 绿 yaw）    单位 °    "
                 "yaw 是累计值不折回 ±180；它没有绝对参考、必然缓慢漂移",
            style="Dim.TLabel",
        ).pack(anchor="w", pady=(2, 0))

    def _build_odom_tab(self, tab: ttk.Frame) -> None:
        self.odom_lbls: dict[str, tk.Label] = {}

        ops = self._panel(tab, "里程计操作")
        row1 = ttk.Frame(ops, style="Panel.TFrame")
        row1.pack(fill="x")
        ttk.Button(row1, text="读一帧", style="Imu.TButton",
                   command=lambda: self.send_proto("odom_read", "读一帧里程计")
                   ).pack(side="left", padx=(0, 6))
        self.odom_report_btn = ttk.Button(row1, text="开始连续上报", style="Imu.TButton",
                                          command=self.toggle_odom_report)
        self.odom_report_btn.pack(side="left", padx=(0, 6))
        ttk.Button(row1, text="归零", style="Imu.TButton",
                   command=self.do_odom_reset).pack(side="left", padx=(0, 6))
        ttk.Button(row1, text="清空轨迹", style="Imu.TButton",
                   command=self.clear_trail).pack(side="left", padx=(0, 6))
        ttk.Label(row1, text="「归零」= 当前位置记为原点、当前朝向记为 0（固件里也会重设基准）",
                  style="Dim.TLabel").pack(side="right")

        # 距离刻度标定：里程计唯一的必测参数。
        row2 = ttk.Frame(ops, style="Panel.TFrame")
        row2.pack(fill="x", pady=(6, 0))
        ttk.Label(row2, text="距离刻度", style="Panel.TLabel").pack(side="left", padx=(0, 6))
        self.scale_var = tk.StringVar(value="1.000")
        ttk.Entry(row2, textvariable=self.scale_var, width=9,
                  justify="right").pack(side="left", padx=(0, 4))
        ttk.Button(row2, text="写入", style="Imu.TButton",
                   command=self.apply_odom_scale).pack(side="left", padx=(0, 6))
        ttk.Button(row2, text="读取", style="Imu.TButton",
                   command=lambda: self.send_proto("odom_get_scale", "读当前距离刻度")
                   ).pack(side="left", padx=(0, 6))
        ttk.Label(row2, text="← 用「100 / 实测走过的 dist」算出来填这里",
                  style="Dim.TLabel").pack(side="left")
        ttk.Label(row2, text="范围 0.2~5.0，超出固件会夹住", style="Dim.TLabel").pack(side="right")

        # 位姿读数
        state = self._panel(tab, "位姿")
        head = ttk.Frame(state, style="Panel.TFrame")
        head.pack(fill="x")

        pos = ttk.Frame(head, style="Panel.TFrame")
        pos.grid(row=0, column=0, sticky="nw", padx=(0, 18))
        tk.Label(pos, text="位置 (cm)", bg=C_PANEL, fg=C_ACCENT,
                 font=("Microsoft YaHei", 10, "bold")).pack(anchor="w", pady=(0, 2))
        self._readout(pos, "X", "x", width=10, store=self.odom_lbls)
        self._readout(pos, "Y", "y", width=10, store=self.odom_lbls)
        self._readout(pos, "离原点", "range", width=10, store=self.odom_lbls)

        head_lbl = ttk.Frame(head, style="Panel.TFrame")
        head_lbl.grid(row=0, column=1, sticky="nw", padx=(0, 18))
        tk.Label(head_lbl, text="航向 / 速度", bg=C_PANEL, fg=C_YAW,
                 font=("Microsoft YaHei", 10, "bold")).pack(anchor="w", pady=(0, 2))
        self._readout(head_lbl, "θ", "th", width=10, store=self.odom_lbls)
        self._readout(head_lbl, "线速度", "v", width=10, store=self.odom_lbls)
        self._readout(head_lbl, "角速度", "w", width=10, store=self.odom_lbls)

        raw = ttk.Frame(head, style="Panel.TFrame")
        raw.grid(row=0, column=2, sticky="nw")
        tk.Label(raw, text="原始量（标定用）", bg=C_PANEL, fg=C_DIM,
                 font=("Microsoft YaHei", 10, "bold")).pack(anchor="w", pady=(0, 2))
        self._readout(raw, "路程", "dist", width=10, store=self.odom_lbls)
        self._readout(raw, "刻度", "scale", width=10, store=self.odom_lbls)
        for c in range(3):
            head.columnconfigure(c, weight=1)

        self.traj_plot = TrajectoryPlot(state)
        self.traj_plot.pack(fill="both", expand=True, pady=(8, 0))
        ttk.Label(
            state,
            text="XY 轨迹（两轴等比例，蓝色走过的路 / 绿色当前位置与朝向 / 灰色原点）    "
                 "走一个正方形或绕一圈，回来时该压在起点十字上 —— 压不上说明刻度没标定",
            style="Dim.TLabel",
        ).pack(anchor="w", pady=(2, 0))

    # ---------------- 连接 ----------------
    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connect(self) -> None:
        if self.link.connected:
            self.link.close()
            self.connect_btn.configure(text="连接")
            self.status_var.set("未连接")
            self._set_status_color(C_DIM)
            self.log_line("已断开连接", dim=True)
            return

        port = self.port_var.get()
        if not port:
            messagebox.showwarning("没有可用端口", "没检测到串口，检查 USB 线和板子上电后点「刷新」。")
            return
        try:
            self.link.open(port, 115200)
        except Exception as exc:
            messagebox.showerror("打开串口失败", f"{port}\n{exc}")
            return

        self.connected_at = time.time()
        self.connect_btn.configure(text="断开")
        self.status_var.set(f"已连接 {port}，等待启动…")
        self._set_status_color(C_ACCENT)
        self.plot.clear()
        self.gyro_plot.clear()
        self.att_plot.clear()
        # 板子刚复位，固件里的连续上报状态回到「关」，界面认知跟着复位，否则会发反
        self.imu_reporting = False
        self.imu_report_btn.configure(text="开始连续上报")
        self.att_reporting = False
        self.att_report_btn.configure(text="开始姿态上报")
        self.log_line(f"已连接 {port}", dim=True)

    def _set_status_color(self, color: str) -> None:
        ttk.Style(self).configure("Status.TLabel", background=C_PANEL, foreground=color)

    def send_all(self, payloads: list[bytes], echo: bool = True) -> bool:
        """把协议层生成的一串帧依次写出去。"""
        if not self.link.connected:
            self.log_line("串口未连接，指令未发送", dim=True)
            return False
        for payload in payloads:
            try:
                self.link.send(payload)
            except Exception as exc:
                self.log_line(f"发送失败：{exc}", dim=True)
                return False
            if echo:
                self.log_line(f">> {payload.hex(' ').upper()}", dim=True)
        return True

    def send_proto(self, method: str, name: str, *args) -> bool:
        """按协议层的方法名发一条指令。"""
        if not self.link.connected:
            self.log_line("串口未连接", dim=True)
            return False
        payloads = getattr(self.proto, method)(*args)
        if self.send_all(payloads):
            self.log_line(f"→ {name}")
            return True
        return False

    # ---------------- 运动控制 ----------------
    def current_duration(self) -> int:
        try:
            return max(1, min(10000, int(float(self.dur_var.get()))))
        except ValueError:
            return 2000

    def on_speed_change(self, value: float) -> None:
        pct = int(round(value))
        self.speed_var.set(pct)
        # 这里的换算必须和 FrameProtocol.move() 里写进数据域的那一份是同一个算法，
        # 否则界面上写着 15.0 cm/s、帧里发的却是别的数 —— 而且没有任何东西会报错。
        cm_s = pct / 100.0 * CHASSIS_MAX_SPEED_CM_S
        self.speed_lbl.configure(text=f"{pct}%  ({cm_s:.1f} cm/s)")

    def move(self, code: str, name: str) -> None:
        if not self.link.connected:
            self.log_line("串口未连接", dim=True)
            return
        pct = self.speed_var.get()
        ms = self.current_duration()
        payloads = self.proto.move(code, pct, ms)
        if self.send_all(payloads):
            self.log_line(f"→ {name} @ {pct}%  {ms} ms")

    def emergency_stop(self) -> None:
        if self.send_proto("stop", "急停"):
            pass  # send_proto 已经打过日志了

    def do_arc(self) -> None:
        """走一段圆弧。半径从「半径」框来，速度用滑块，时长用「时长」框。"""
        if not self.link.connected:
            self.log_line("串口未连接", dim=True)
            return
        try:
            r = float(self.arc_r_var.get())
        except ValueError:
            messagebox.showwarning("参数不对", "圆弧半径必须是数字（单位 cm）。")
            return
        if r <= 0:
            messagebox.showwarning("参数不对",
                                   "圆弧半径必须大于 0。\n\n要直走请点「前进」—— "
                                   "半径趋近无穷大就是直走。")
            return

        pct = self.speed_var.get()
        ms = self.current_duration()

        # 半径太小时算出来的 w 会超过满油门，固件那边会夹。夹住之后实际走出来的
        # 半径比填的大 —— 这个差额必须说出来：不说的话用户会以为车坏了
        # （「我要 5cm 它转了 15cm」），而真相是这个速度下物理上转不了那么急。
        w_want = arc_omega_mrad_s(pct, r)
        w_cap = CHASSIS_SPIN_FULL_SCALE_MRAD_S
        if abs(w_want) > w_cap:
            r_eff = 1000.0 * (pct / 100.0 * CHASSIS_MAX_SPEED_CM_S * 10.0) / w_cap
            self.log_line(
                f"⚠ {pct}% 档走 {r:g}cm 需要 {w_want:.0f} mrad/s，超过满油门 "
                f"{w_cap:.0f} —— 实际按 {r_eff:.1f}cm 走。想更急就降速。",
                dim=True,
            )

        payloads = self.proto.arc(pct, r, ms)
        if self.send_all(payloads):
            self.log_line(f"→ 圆弧 半径 {r:g} cm @ {pct}%  {ms} ms")

    def apply_pid(self) -> None:
        try:
            kp, ki, kd = float(self.kp_var.get()), float(self.ki_var.get()), float(self.kd_var.get())
        except ValueError:
            messagebox.showwarning("参数不对", "Kp / Ki / Kd 必须是数字。")
            return
        self.send_proto("set_pid", f"PID = {kp} / {ki} / {kd}", kp, ki, kd)

    def apply_yaw_pid(self) -> None:
        try:
            kp = float(self.ykp_var.get())
            ki = float(self.yki_var.get())
            kd = float(self.ykd_var.get())
        except ValueError:
            messagebox.showwarning("参数不对", "yaw 环的 Kp / Ki / Kd 必须是数字。")
            return
        # 固件回的 [CMD] 行是**回读值**，可能被夹过。它和这里填的数不一样的时候
        # 就是被夹了 —— 不用在这里预先夹一遍，那样只会多一个看不见的天花板。
        self.send_proto("set_yaw_pid", f"yaw 环 PID = {kp} / {ki} / {kd}", kp, ki, kd)

    def toggle_data_report(self) -> None:
        """[DATA] 开关。发的是目标状态（置位），不是「切换」—— 勾选框本身
        就是状态，不会和固件跑偏。"""
        want = self.data_report_var.get()
        self.send_proto("data_report", f"底盘数据上报{'开' if want else '关'}", want)

    def do_frame_stats(self) -> None:
        self.send_proto("frame_stats", "帧接收统计")

    # ---------------- IMU ----------------
    def toggle_imu_report(self) -> None:
        want = not self.imu_reporting
        if self.send_proto("imu_report", f"IMU 连续上报{'开' if want else '关'}", want):
            self.imu_reporting = want
            self.imu_report_btn.configure(
                text="停止连续上报" if want else "开始连续上报")

    def do_calibrate(self) -> None:
        if messagebox.askokcancel(
            "零偏标定",
            "标定期间车体必须完全静止（约 5 秒），不能碰车、不能在桌上敲键盘。\n\n"
            "确认车已放稳、轮子不会转动再继续。",
        ):
            self.send_proto("imu_calibrate", "陀螺仪零偏标定")

    def update_imu(self, kv: dict) -> None:
        def g(key: str) -> float:
            return kv.get(key, 0.0)

        lbl = self.imu_lbls
        for key, fmt in (("ax", "+.3f"), ("ay", "+.3f"), ("az", "+.3f"),
                         ("gx", "+.2f"), ("gy", "+.2f"), ("gz", "+.2f")):
            lbl[key].configure(text=format(g(key), fmt))
        lbl["temp"].configure(text=f"{g('temp'):.1f} °C")

        # |a| 是加速度计好坏的唯一硬指标：静止时必须 ≈ 1.000 g。
        # 偏出 ±5% 直接标红 —— 要么量程选错数据削顶，要么芯片有问题。
        amag = g("amag")
        lbl["amag"].configure(text=f"{amag:.3f} g",
                              fg=C_TEXT if 0.95 <= amag <= 1.05 else C_WARN)

        biased = g("bias") >= 0.5
        lbl["bias"].configure(text="已标定" if biased else "未标定",
                              fg=C_TEXT if biased else C_WARN)

    # ---------------- 姿态 ----------------
    def toggle_att_report(self) -> None:
        want = not self.att_reporting
        if self.send_proto("att_report", f"姿态连续上报{'开' if want else '关'}", want):
            self.att_reporting = want
            self.att_report_btn.configure(
                text="停止姿态上报" if want else "开始姿态上报")

    def do_turn(self, deg: float) -> None:
        """定角度转向（yaw 闭环）。deg 正=左转，负=右转。"""
        if self.send_proto("turn_angle", f"定角度转向 {deg:g}°", deg):
            # 转向靠姿态闭环，没有姿态上报就看不到 yaw 在动，会以为没反应
            if not self.att_reporting:
                self.toggle_att_report()

    def do_uturn(self) -> None:
        """掉头 180°。走 yaw 闭环，所以和 do_turn 一样要打开姿态上报。"""
        if self.send_proto("uturn", "掉头 180°"):
            if not self.att_reporting:
                self.toggle_att_report()

    def do_turn_from_entry(self) -> None:
        raw = self.turn_var.get().strip()
        try:
            deg = float(raw)
        except ValueError:
            self.log_line(f"角度填的不是数字：{raw!r}", dim=True)
            return
        if not -720.0 <= deg <= 720.0:
            # 固件会夹到 ±720（YAW_TARGET_MAX_DEG），这里先拦下来并说清楚为什么
            self.log_line(f"角度超出 ±720° 范围：{deg:g}（固件上限，转更多圈没有意义）",
                          dim=True)
            return
        self.do_turn(deg)

    def update_attitude(self, kv: dict) -> None:
        def g(key: str) -> float:
            return kv.get(key, 0.0)

        lbl = self.att_lbls
        for key in ("roll", "pitch", "yaw"):
            lbl[key].configure(text=format(g(key), "+.2f"))

        # 定角度转向：目标角用 yaw 同色标出来（曲线上的灰虚线就是它），
        # 正在转的时候整个读数提亮，一眼能看出闭环还在工作。
        busy = g("ybusy") >= 0.5
        lbl["ytgt"].configure(text=format(g("ytgt"), "+.1f"),
                              fg=C_YAW if busy else C_DIM)
        lbl["ybusy"].configure(text="转向中" if busy else "—",
                               fg=C_ACCENT if busy else C_DIM)

        # 静止修正状态：yaw 冻结中 + 零偏正在被在线跟踪。
        # 车停下不该动的时候显示「跟踪中」，这时 yaw 不再漂。
        still = g("still") >= 0.5
        lbl["still"].configure(text="跟踪中" if still else "—",
                               fg=C_GZ if still else C_DIM)

        # 实测解算频率：正常贴着 100。掉到七八十说明 loop 被拖慢了，
        # 那时 dt 变大、积分误差跟着变大 —— 属于要先解决的底层问题。
        rate = g("rate")
        lbl["rate"].configure(text=f"{rate:.0f} Hz",
                              fg=C_TEXT if rate >= 90 else C_WARN)

    # ---------------- 里程计 ----------------
    def toggle_odom_report(self) -> None:
        want = not self.odom_reporting
        if self.send_proto("odom_report", f"里程计连续上报{'开' if want else '关'}", want):
            self.odom_reporting = want
            self.odom_report_btn.configure(
                text="停止连续上报" if want else "开始连续上报")

    def do_odom_reset(self) -> None:
        self.send_proto("odom_reset", "里程计归零")
        # 归零后旧轨迹是「上一个原点」下的，留着会和新的原点十字对不上，直接清掉
        self.clear_trail()

    def clear_trail(self) -> None:
        self.traj_plot.clear()
        self.log_line("轨迹已清空（固件里的位姿没动，只是不画了）", dim=True)

    def apply_odom_scale(self) -> None:
        raw = self.scale_var.get().strip()
        try:
            scale = float(raw)
        except ValueError:
            self.log_line(f"刻度填的不是数字：{raw!r}", dim=True)
            return
        if scale <= 0:
            self.log_line(f"刻度必须为正：{raw!r}", dim=True)
            return
        if not 0.2 <= scale <= 5.0:
            # 固件会夹到 [0.2, 5.0]，但静默夹住最坑 —— 用户以为写进去了，
            # 实际用的是另一个值。先说清楚，再照发（固件回读的才是真相）。
            self.log_line(f"刻度 {scale:g} 超出固件范围 0.2~5.0，会被夹住；"
                          "这个偏差一般说明标定时的测量有问题", dim=True)
        self.send_proto("odom_set_scale", f"设置距离刻度 {scale:g}", scale)

    def update_odom(self, kv: dict) -> None:
        def g(key: str) -> float:
            return kv.get(key, 0.0)

        lbl = self.odom_lbls
        x, y = g("x"), g("y")
        lbl["x"].configure(text=format(x, "+.2f"))
        lbl["y"].configure(text=format(y, "+.2f"))

        # 离原点距离：轨迹图上看不出精确值，这里给个数。回到起点时它该接近 0。
        rng = math.hypot(x, y)
        lbl["range"].configure(text=f"{rng:.2f}",
                               fg=C_DIM if rng < 2.0 else C_TEXT)
        lbl["th"].configure(text=format(g("th"), "+.2f"))
        lbl["v"].configure(text=format(g("v"), "+.2f"))
        lbl["w"].configure(text=format(g("w"), "+.1f"))
        lbl["dist"].configure(text=f"{g('dist'):.2f}")

        scale = g("scale")
        # 刻度还是 1.000 = 没标定过，标成警告色 —— 这时候 x/y 的绝对值不能信
        lbl["scale"].configure(text=f"{scale:.4f}",
                               fg=C_WARN if abs(scale - 1.0) < 1e-4 else C_TEXT)

    # ---------------- CSV ----------------
    def toggle_csv(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()
            path, rows = self.csv_path, self.csv_rows
            self.csv_file = self.csv_writer = None
            self.csv_btn.configure(text="开始记录 CSV")
            self.log_line(f"CSV 已保存：{path}（{rows} 行）")
            return

        path = filedialog.asksaveasfilename(
            title="保存数据到 CSV",
            defaultextension=".csv",
            initialfile="car_data.csv",
            filetypes=[("CSV 文件", "*.csv"), ("全部文件", "*.*")],
        )
        if not path:
            return
        self.csv_file = open(path, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(
            self.csv_file, fieldnames=CSV_FIELDS, extrasaction="ignore", restval="")
        self.csv_writer.writeheader()
        self.csv_rows = 0
        self.csv_t0 = time.time()
        self.csv_path = path
        self.csv_btn.configure(text="停止记录 CSV")
        self.log_line(f"开始记录到 {path}")

    def write_csv(self, kv: dict) -> None:
        if self.csv_file is None:
            return
        row = {f: kv.get(f, "") for f in CSV_FIELDS if f != "t"}
        row["t"] = f"{time.time() - self.csv_t0:.3f}"
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.csv_rows += 1

    # ---------------- 主循环 ----------------
    def poll_queue(self) -> None:
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "line":
                    self.on_line(payload)
                else:
                    self.log_line(payload)
        except queue.Empty:
            pass
        self.after(POLL_MS, self.poll_queue)

    def on_line(self, line: str) -> None:
        # 四种数据行都带 "DATA" 字样，必须先按前缀区分开 ——
        # 顺序也有讲究：它们都以 '[' 开头且都含 DATA，而且 [ODOM] 和 [DATA] 只差
        # 一个前缀，所以具体前缀必须排在 DATA_PREFIX 前面先判
        if line.startswith(IMU_DATA_PREFIX):
            kv = {k: float(v) for k, v in KV_RE.findall(line[len(IMU_DATA_PREFIX):])}
            if kv:
                self.update_imu(kv)
                self.gyro_plot.add(kv)
                self.write_csv(kv)
            return

        if line.startswith(ATT_DATA_PREFIX):
            kv = {k: float(v) for k, v in KV_RE.findall(line[len(ATT_DATA_PREFIX):])}
            if kv:
                self.update_attitude(kv)
                self.att_plot.add(kv)
                self.write_csv(kv)
            return

        if line.startswith(ODOM_DATA_PREFIX):
            kv = {k: float(v) for k, v in KV_RE.findall(line[len(ODOM_DATA_PREFIX):])}
            if kv:
                self.update_odom(kv)
                self.traj_plot.add(kv)
                self.write_csv(kv)
            return

        if line.startswith(DATA_PREFIX):
            kv = {k: float(v) for k, v in KV_RE.findall(line[len(DATA_PREFIX):])}
            if kv:
                self.update_state(kv)
                self.plot.add(kv)
                self.write_csv(kv)
            return

        # IMU 的其余输出（标定进度、扫描结果、报错）都是给人看的，直接进日志
        self.log_line(line)

        # 板子打完指令表说明启动完成
        if "[HELP]" in line and self.status_var.get().startswith("已连接"):
            self.status_var.set(f"已连接 {self.port_var.get()}")

    def update_state(self, kv: dict) -> None:
        def g(key: str) -> float:
            return kv.get(key, 0.0)

        self.speed_l_lbl.configure(text=f"{g('speed_l'):6.2f} cm/s")
        self.speed_r_lbl.configure(text=f"{g('speed_r'):6.2f} cm/s")
        self.sub_l_lbl.configure(
            text=f"目标 {g('tgt_l'):5.1f}   PWM {g('pwm_l'):4.0f}   计数 {g('enc_l'):.0f}"
        )
        self.sub_r_lbl.configure(
            text=f"目标 {g('tgt_r'):5.1f}   PWM {g('pwm_r'):4.0f}   计数 {g('enc_r'):.0f}"
        )

    def refresh_plot(self) -> None:
        # 隐藏的那个 canvas 宽度是 1，redraw 会自己提前返回，不用判断当前标签页
        self.plot.redraw()
        self.gyro_plot.redraw()
        self.att_plot.redraw()
        self.traj_plot.redraw()
        self.after(PLOT_REFRESH_MS, self.refresh_plot)

    def log_line(self, text: str, dim: bool = False) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "100.0")  # 只留最近 400 行，防止越跑越慢
        self.log.see("end")
        self.log.configure(state="disabled")

    def on_close(self) -> None:
        try:
            if self.link.connected:
                self.send_all(self.proto.stop(), echo=False)  # 退出前确保停车
                time.sleep(0.1)
        except Exception:
            pass
        if self.csv_file is not None:
            self.csv_file.close()
        self.link.close()
        self.destroy()


def default_port() -> str:
    """优先用 platformio.ini 里的 monitor_port。"""
    ini = Path(__file__).resolve().parent.parent / "platformio.ini"
    if ini.exists():
        text = ini.read_text(encoding="utf-8", errors="ignore")
        m = re.search(r"^\s*monitor_port\s*=\s*(\S+)", text, re.M)
        if m:
            return m.group(1)
    return "COM10"


def main() -> None:
    parser = argparse.ArgumentParser(description="ROS-CAR 上位机控制台（图形界面）")
    parser.add_argument("-p", "--port", default=None, help="串口号，默认取 platformio.ini 的 monitor_port")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="波特率，默认 115200")
    args = parser.parse_args()

    app = CarConsole(args.port or default_port(), args.baud)
    app.mainloop()


if __name__ == "__main__":
    main()
