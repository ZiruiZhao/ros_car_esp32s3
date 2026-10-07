#pragma once

#include <Arduino.h>

// ============================================================================
//  串口数据帧协议 · 接收侧 —— 字节流 → 帧
//
//   串口上来的从来不是「帧」，是一串没有边界的字节。本模块负责把它切成帧并
//   校验，然后交给调用方。协议定义在 串口通信手册.md，这里重复必须写死在
//   代码里的部分：
//
//     偏移  字段        字节
//       0    帧头        2      0xEB 0x90
//       2    设备地址     1      0x01~0xFE 单播，0xFF 广播
//       3    功能码       1      bit7 = 方向位，0 = 上位机→下位机
//       4    帧序号       1      0~255 循环自增
//       5    数据长度     1      数据域字节数
//       6    数据域       N      小端
//     6+N   CRC16       2      CRC16-Modbus，小端
//
//   全帧小端。CRC 范围 = 设备地址 .. 数据域末尾（**不含帧头**，也不含 CRC 自己）。
//
//   ---- 这一层不知道什么是速度、什么是转角 ----
//
//   它只做「字节 → 帧」。解出一帧就回调给上层（main 的 handle_frame），由那边
//   解释成动作。**本模块不打任何日志** —— 和其余 lib 一致，串口输出统一归 main。
//
//   ---- 字节流上真正难的三件事 ----
//
//   要变成帧，必须处理：
//
//     ① **粘包**：一次读出两帧半是常态。所以取帧是「能取几帧取几帧」，
//        不能「读一次算一帧」。这条约束封在 frame_feed_bytes() 里 ——
//        用那个入口，调用方就没有写漏循环的机会。
//     ② **半包**：一帧可能横跨好几次 read。数据不够时**什么都不做**，原样留着，
//        下次喂进来接着拼 —— 绝不能「不够就先丢掉，等下一帧」。
//     ③ **丢同步**：噪声、上电瞬间的乱码、或者上位机发到一半被拔线，都会让缓存区
//        的内容跟帧头对不上。这时必须能自己找回边界，否则后面所有帧全废。
//
//   ---- 为什么 CRC 错了是「挪一个字节」而不是「整帧丢掉」----
//
//   因为在我们判定「这是一帧」之前，那个帧头本身就可能是个假象 —— 它可能是
//   上一帧数据域里恰好出现的 EB 90。整帧丢掉的话，真正的帧头会被一起丢掉，
//   坏一帧变成坏一片。挪一个字节重新找，最多多扫几遍，不会漏掉后面的帧。
//
//   一帧最长 8 + 32 = 40 字节，挪一次的代价可以忽略。
// ============================================================================

// 数据域上限。本工程定义的请求码里最长的数据域是 6 字节（0x01 速度控制、0x05 PID），
// 留到 32 是给后续扩展留的余量 —— 加一个新指令不用回来改这个数。
// ⚠ 它同时决定了帧长上限，别设小；长度字段超过它的帧会被当成坏帧丢掉并计入 oversize。
static constexpr uint8_t FRAME_PAYLOAD_MAX = 32;
static constexpr uint8_t FRAME_OVERHEAD = 8;  // 帧头2 + 地址1 + 功能码1 + 序号1 + 长度1 + CRC2
static constexpr uint8_t FRAME_TOTAL_MAX = FRAME_OVERHEAD + FRAME_PAYLOAD_MAX;

// 帧内字节间隔超时（ms）。超过这么久还没凑齐一帧，就把半截缓存丢掉重新找同步。
//
// 为什么非有不可：半包缓存是**跨调用**保留的。发送端要是发到一半断了（拔线、
// 上位机崩了、复位），那半帧会永远留在缓存里，把后面所有东西堵死 ——
// 而且症状极隐蔽：串口看着一切正常，就是从此再也没有任何指令生效。
//
// 50ms 很宽容：115200 下相邻字节只隔 0.09ms，正常帧绝不会触发。
static constexpr uint32_t FRAME_INTER_BYTE_TIMEOUT_MS = 50;

static constexpr uint8_t FRAME_HEAD_0 = 0xEB;
static constexpr uint8_t FRAME_HEAD_1 = 0x90;

// 本机地址。单机场景就一台车。
// 地址字段现在就占着位，是为了以后总线上再挂传感器节点时有得区分。
static constexpr uint8_t FRAME_ADDR_LOCAL = 0x01;
static constexpr uint8_t FRAME_ADDR_BROADCAST = 0xFF;

// ---- 功能码 ----
// 最高位是方向位：0 = 上位机→下位机（请求），1 = 下位机→上位机（应答）。
// 所以请求码只占 0x00~0x7F，应答码 = 请求码 | 0x80。
//
// ⚠ 这张表和上位机 test/car_upper.py 顶部那张是**同一份契约**，改一个数两边都得改。
static constexpr uint8_t FC_CHASSIS_DRIVE = 0x01;  // v(int16 mm/s), w(int16 mrad/s), 时长(uint16 ms)
static constexpr uint8_t FC_CHASSIS_STOP = 0x02;   // 空
static constexpr uint8_t FC_TURN_ANGLE = 0x03;     // int16 mrad，带符号，正 = 左转
static constexpr uint8_t FC_YAW_ABORT = 0x04;      // 空
static constexpr uint8_t FC_CHASSIS_PID = 0x05;    // Kp, Ki, Kd 各 uint16（增益 ×1000）
static constexpr uint8_t FC_IMU_READ = 0x20;       // 空
static constexpr uint8_t FC_IMU_REPORT = 0x21;     // uint8，0 = 关 / 1 = 开
static constexpr uint8_t FC_IMU_CALIBRATE = 0x22;  // uint16 采样数（0 = 用固件默认）
static constexpr uint8_t FC_I2C_SCAN = 0x23;       // 空
static constexpr uint8_t FC_ATT_READ = 0x24;       // 空
static constexpr uint8_t FC_ATT_REPORT = 0x25;     // uint8，0 = 关 / 1 = 开
static constexpr uint8_t FC_ATT_ZERO = 0x26;       // 空
static constexpr uint8_t FC_ODOM_READ = 0x27;      // 空
static constexpr uint8_t FC_ODOM_REPORT = 0x28;    // uint8，0 = 关 / 1 = 开
static constexpr uint8_t FC_ODOM_RESET = 0x29;     // 空
static constexpr uint8_t FC_ODOM_SET_SCALE = 0x2A; // uint16（刻度 ×10000）

// ---- 0x2B~0x2E：调参与诊断 ----
//
//   0x2B 调 yaw 环 PID —— 没有这条就只能改代码重烧
//   0x2C 帧接收统计
//   0x2D 关掉 [DATA] 上报 —— 没有这条就永久 5Hz 刷屏，关不掉
//   0x2E 只读当前距离刻度（写入是 0x2A）
//
// ⚠ 0x2C 有个循环依赖：**帧本身收不进来的时候，这条帧也发不进来**。所以 main 里另有
//   poll_frame_stats_warning()：错误计数器一涨就自己出声，不用谁来问。
//
// ⚠⚠ 0x2B 的量化是 **×10000，不是 0x05 那套 ×1000**。这不是笔误：
//   yaw 环的三个参数比轮速环小三个数量级（默认 0.008 / 0.020 / 0.0020），
//   ×1000 的话 Kd 只能取到 2 —— 分辨率 0.001 是默认值的一半，等于把 Kd 这个
//   旋钮废掉了（Kp 也只有 8，1 格 12%）。×10000 后默认值是 80 / 200 / 20，
//   分辨率 1e-4，正好和这个环打印用的 %.4f 对上。
//   量程：×10000 下 uint16 最大 6.5535。yaw 环的输出本来就夹在 ±0.8 油门，
//   Kp 到 1 就已经「差 1° 就满舵」了，用不到那么大。
static constexpr uint8_t FC_YAW_PID = 0x2B;        // Kp, Ki, Kd 各 uint16（增益 ×10000）
static constexpr uint8_t FC_FRAME_STATS = 0x2C;    // 空
static constexpr uint8_t FC_DATA_REPORT = 0x2D;    // uint8，0 = 关 / 1 = 开
static constexpr uint8_t FC_ODOM_GET_SCALE = 0x2E; // 空

// 解出的一帧。payload 是定长数组，实际有效长度看 len。
struct Frame {
  uint8_t addr;
  uint8_t func;
  uint8_t seq;
  uint8_t len;
  uint8_t payload[FRAME_PAYLOAD_MAX];
};

// 解析统计。串口出问题时第一个要看的就是它，能直接分出是哪一类故障：
//   good 不动、bad_crc 也不动   → 上位机压根没发（或者线断了，见 main 的告警）
//   bad_crc 一直涨              → 字节收到了但内容不对：波特率不匹配 / 线太长 / 没共地
//   oversize 涨                 → 长度字段被干扰了，或者两边对 FRAME_PAYLOAD_MAX 理解不一致
//   timeout 涨                  → 发送端发到一半断了。正常情况应该一直是 0
//   dropped 涨                  → 缓存区满：帧长超限，或者主循环被别的东西卡住太久
struct FrameStats {
  uint32_t good;      // 通过校验、已交给上层的帧
  uint32_t bad_crc;   // 帧头对上了但 CRC 不过
  uint32_t oversize;  // 长度字段超过 FRAME_PAYLOAD_MAX
  uint32_t timeout;   // 半帧放置超时，被迫丢弃重新找同步
  uint32_t dropped;   // 缓存区满，被迫丢弃最老的字节
};

// 解出一帧时的回调。main 传 handle_frame 进来。
using FrameHandler = void (*)(const Frame &);

// 清空解析缓存和统计。必须在第一次喂字节之前调 —— 缓存是跨调用保留的，
// 上电时里面是随机垃圾的话，头几个字节会被当成半帧吃掉。
void frame_protocol_init();

// 喂一个字节进解析缓存。串口每读到一个字节就往这里送一个。
void frame_feed(uint8_t byte);

// 从缓存里取下一帧。取到返回 true 并填 *out；取不到（没数据 / 半包）返回 false。
//
// ⚠ 直接用它的话，**必须循环调用到返回 false 为止**：一次 read 很可能含好几帧
//   （粘包），只取一帧的话剩下的要等下一轮，收帧速度就被主循环拖住了。
//   不想操心这件事就用 frame_feed_bytes()，它把这个循环封在里面了。
bool frame_take(Frame *out);

// 便利入口：喂一串字节，每解出一帧就调一次 on_frame，返回解出的帧数。
// main 的 poll_serial() 用的就是这个 —— 粘包、半包、丢同步、超时全在内部处理，
// 调用方只需要「把这次读到的字节交出去」。
uint16_t frame_feed_bytes(const uint8_t *data, uint16_t len, FrameHandler on_frame);

// 缓存里还有多少个字节没消化掉。只用于报告（main 的 print_frame_stats）。
uint16_t frame_pending();

FrameStats frame_get_stats();

// 功能码 → 中文名，打日志用。不认识的码返回 "未知功能码"。
// 名字表放在这里而不是 main，是为了和上面那张码表挨着 —— 分开放就会随着改码
// 悄悄过期，那种过期没有任何东西会报错。
const char *frame_func_name(uint8_t func);

// ---- 小端取值 ----
// 自己拼字节而不是把 uint8_t* 强转成 int16_t* 再解引用：那样会同时依赖
// ① 对齐（ESP32 允许非对齐访问，换个平台就崩）② 字节序（ESP32 是小端，
// 别的架构不一定）。协议写死了小端，那就老老实实按小端拼出来。
static inline int16_t frame_rd_i16(const uint8_t *p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                              (static_cast<uint16_t>(p[1]) << 8));
}

static inline uint16_t frame_rd_u16(const uint8_t *p)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}
