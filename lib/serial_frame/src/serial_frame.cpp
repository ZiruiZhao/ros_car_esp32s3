#include "serial_frame.h"

#include <string.h>

// 解析缓存。**持久**跨调用 —— 半包就是靠它接上的。
// 大小 = 一帧最长长度：够放下任何一帧，多了没用（多出来的字节不等它攒满就
// 已经在 frame_take 里被挪出去了）。
static uint8_t g_frame_buf[FRAME_TOTAL_MAX];
static uint16_t g_frame_len = 0;
static uint32_t g_frame_last_feed_ms = 0;
static FrameStats g_frame_stats = {0, 0, 0, 0, 0};

// CRC16-Modbus：多项式 0xA001（0x8005 反射），初值 0xFFFF，无最终异或。
// 和上位机 test/car_upper.py 的 crc16_modbus() 是同一段算法 —— 那边用标准校验值
// b"123456789" → 0x4B37 锚过。差一个多项式，两边就永远对不上，
// 而现象只是「帧全被丢掉」，不报任何错。
static uint16_t frame_crc16_modbus(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 1U) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001U);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

// 丢掉缓存最前面的 n 个字节，剩下的往前挪。
static void frame_buf_drop(uint16_t n)
{
  if (n >= g_frame_len) {
    g_frame_len = 0;
    return;
  }
  memmove(g_frame_buf, g_frame_buf + n, static_cast<size_t>(g_frame_len - n));
  g_frame_len = static_cast<uint16_t>(g_frame_len - n);
}

void frame_protocol_init()
{
  g_frame_len = 0;
  g_frame_last_feed_ms = millis();
  g_frame_stats.good = 0;
  g_frame_stats.bad_crc = 0;
  g_frame_stats.oversize = 0;
  g_frame_stats.timeout = 0;
  g_frame_stats.dropped = 0;
}

void frame_feed(uint8_t byte)
{
  const uint32_t now = millis();

  // 距上一个字节太久 = 那半帧凉了，丢掉重新找同步。见头文件：不加这一条，
  // 一次半途而废的发送会让解析器永久卡死，后面什么指令都进不来。
  if (g_frame_len > 0 && (now - g_frame_last_feed_ms) > FRAME_INTER_BYTE_TIMEOUT_MS) {
    g_frame_stats.timeout++;
    g_frame_len = 0;
  }
  g_frame_last_feed_ms = now;

  if (g_frame_len >= FRAME_TOTAL_MAX) {
    // 缓存满了还凑不出一帧，说明前面那堆字节根本不是帧（或者长度字段在被干扰）。
    // 丢**最老的一个**字节，而不是清空整个缓存：清空会把「刚好还没收全的那一帧」
    // 的头部一起丢掉，而丢一个字节的话，真正的帧头迟早会滑到最前面被认出来。
    g_frame_stats.dropped++;
    frame_buf_drop(1);
  }
  g_frame_buf[g_frame_len++] = byte;
}

bool frame_take(Frame *out)
{
  // 每轮循环开始时，缓存区里要么有一个对齐好的候选帧，要么已经被挪到只剩
  // 最后几个还可能是帧头的字节。这个不变量是下面每一步都成立的前提。
  while (g_frame_len >= 2) {
    // ---- ① 对齐帧头 ----
    // 对不上就往前挪一个字节重新找。这一步是「丢同步后自己找回边界」的全部实现。
    if (g_frame_buf[0] != FRAME_HEAD_0 || g_frame_buf[1] != FRAME_HEAD_1) {
      frame_buf_drop(1);
      continue;
    }

    // ---- ② 头部还没收齐 ----
    // 帧头 + 地址 + 功能码 + 序号 + 长度 = 6 字节，长度字段在第 5 个字节上。
    // 不够就原样等着，下次喂进来接着拼（半包）。这里**不能**返回失败并清缓存。
    if (g_frame_len < 6) {
      return false;
    }

    const uint8_t plen = g_frame_buf[5];

    // ---- ③ 长度字段不合理 ----
    // 这个「帧头」多半是假的（真帧不会有超限的数据域），当噪声处理：挪一格重找。
    // 不按 plen 去等 —— 等一个被干扰出来的长度值，会把后面真正的帧全堵死。
    if (plen > FRAME_PAYLOAD_MAX) {
      g_frame_stats.oversize++;
      frame_buf_drop(1);
      continue;
    }

    const uint16_t total = static_cast<uint16_t>(FRAME_OVERHEAD + plen);

    // ---- ④ 数据域 + CRC 还没收齐（半包）----
    if (g_frame_len < total) {
      return false;
    }

    // ---- ⑤ 校验 ----
    // CRC 在帧尾，小端：低字节在前。
    const uint16_t crc_wire = frame_rd_u16(g_frame_buf + 6 + plen);
    // 范围从设备地址（偏移 2）到数据域末尾，共 4 + plen 个字节（地址/功能码/序号/长度/数据域）。
    const uint16_t crc_calc =
        frame_crc16_modbus(g_frame_buf + 2, static_cast<uint16_t>(4 + plen));
    if (crc_wire != crc_calc) {
      g_frame_stats.bad_crc++;
      frame_buf_drop(1);  // 挪一格。见头文件：为什么不是整帧丢掉
      continue;
    }

    // ---- ⑥ 成帧 ----
    // 地址**不在这里过滤**。广播帧（0xFF）也得收，而「不是发给我的」属于业务判断，
    // 该由上层决定是丢掉还是回一句 —— 解析层不该替它做这个决定。
    out->addr = g_frame_buf[2];
    out->func = g_frame_buf[3];
    out->seq = g_frame_buf[4];
    out->len = plen;
    memcpy(out->payload, g_frame_buf + 6, plen);

    g_frame_stats.good++;
    frame_buf_drop(total);
    return true;
  }

  // 只剩一个字节，而且它不是帧头首字节 —— 它不可能是任何帧的开头，留着只会
  // 白白占着缓存，让 frame_pending() 一直报「还有没消化完的字节」。
  // （是帧头首字节就留着，等下一个字节来配对。）
  if (g_frame_len == 1 && g_frame_buf[0] != FRAME_HEAD_0) {
    g_frame_len = 0;
  }

  return false;
}

uint16_t frame_feed_bytes(const uint8_t *data, uint16_t len, FrameHandler on_frame)
{
  uint16_t count = 0;
  Frame f;

  for (uint16_t i = 0; i < len; ++i) {
    frame_feed(data[i]);

    // 每喂一个字节就把能取的帧全取出来。**必须取到 false 为止**（粘包），
    // 把循环封在这里，调用方就没有漏写的机会。
    while (frame_take(&f)) {
      if (on_frame != nullptr) {
        on_frame(f);
      }
      count++;
    }
  }

  return count;
}

uint16_t frame_pending()
{
  return g_frame_len;
}

FrameStats frame_get_stats()
{
  return g_frame_stats;
}

const char *frame_func_name(uint8_t func)
{
  // 应答帧（bit7 = 1）下位机不会收到，但真收到了要说清楚，别让人对着
  // 「未知功能码 0x81」猜 —— 那是方向位搞反了，不是码表里少了一项。
  if (func & 0x80U) {
    return "应答码（bit7=1，方向位反了）";
  }

  switch (func) {
    case FC_CHASSIS_DRIVE: return "底盘速度控制";
    case FC_CHASSIS_STOP: return "底盘急停";
    case FC_TURN_ANGLE: return "定角度转向";
    case FC_YAW_ABORT: return "yaw 闭环中止";
    case FC_CHASSIS_PID: return "底盘 PID 参数";
    case FC_IMU_READ: return "IMU 读一帧";
    case FC_IMU_REPORT: return "IMU 连续上报";
    case FC_IMU_CALIBRATE: return "陀螺零偏标定";
    case FC_I2C_SCAN: return "扫描 I2C 总线";
    case FC_ATT_READ: return "姿态读一帧";
    case FC_ATT_REPORT: return "姿态连续上报";
    case FC_ATT_ZERO: return "yaw 归零";
    case FC_ODOM_READ: return "里程计读一帧";
    case FC_ODOM_REPORT: return "里程计连续上报";
    case FC_ODOM_RESET: return "里程计归零";
    case FC_ODOM_SET_SCALE: return "距离刻度设置";
    case FC_YAW_PID: return "yaw 环 PID 参数";
    case FC_FRAME_STATS: return "帧接收统计";
    case FC_DATA_REPORT: return "底盘数据上报开关";
    case FC_ODOM_GET_SCALE: return "读距离刻度";
    default: return "未知功能码";
  }
}
