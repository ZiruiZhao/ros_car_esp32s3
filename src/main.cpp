#include <Arduino.h>
#include <Ticker.h>

#include <math.h>
#include <string.h>

#include "attitude.h"
#include "chassis.h"
#include "imu_mpu6050.h"
#include "odometry.h"
#include "serial_frame.h"
#include "yaw_control.h"

// ============================================================================
//  ESP32-S3 差速小车 · 下位机固件
//
//    车上的工作拆成八个模块，每个一个 lib。本文件只做初始化、跑节拍、把串口帧
//    翻译成动作 —— 控制律一概不在这里：
//      imu_mpu6050   MPU6050 六轴，I2C 寄存器级驱动（原始值 + 零偏标定）
//      attitude      互补滤波 → roll / pitch / yaw，静止时冻结 yaw 并跟踪零偏
//      wheel_encoder AB 双相正交编码，4 倍频计数 → 轮速
//      motor_driver  PWM + 方向双线，带换向死区
//      chassis       轮速环：目标 cm/s → 增量式 PID → PWM（差速混速在这层）
//      yaw_control   转向环：目标角度 → yaw 闭环 → 归一化 w，串级在 chassis 之上
//      odometry      航位推算：编码器 + yaw → x / y / θ
//      serial_frame  串口帧接收：字节流 → 帧（CRC 校验、粘包 / 半包 / 丢同步）
//
//    ---- 主循环的分工 ----
//
//    control_ticker 每 50ms 置一次 g_control_due，闭环计算全部在 loop() 里做 ——
//    定时器回调里不碰 I2C、不碰串口，那会和主循环抢资源。loop() 每圈依次走：
//      闭环（chassis_update + yaw 环，仅节拍到点时）
//      → 姿态解算（自带 100Hz 门限）
//      → 里程计（自带 50ms 门限，必须在闭环之后：它要读这一拍刚采的编码器）
//      → 四路周期上报（各自按 millis 分频）
//      → 收帧、发异常告警
//
//    ⚠ 距离刻度**必须实测标定**：轮径和每转计数在 wheel_encoder.h 里是标称值，
//      没量过。不标定的话走 1 米会报成 90cm 或 110cm，而且走越远偏越多。
//      标定方法见 odometry.h 顶部，30 秒的事，标完用 0x2A 号帧存进去。
//
//    ---- 下行只有一条路：数据帧 ----
//
//    上位机 test/car_upper.py 发过来的二进制帧，拆帧（字节 → 帧，含 CRC 校验）
//    由 serial_frame 负责，下面「协议帧的处理」一节负责帧 → 动作。
//    帧里一次带全 v、w、时长，没有跨帧状态。
//
//    功能码速查（完整表 + 样例帧字节见 串口通信手册.md，改一个数两边都得改）：
//      0x01 底盘速度控制   v(int16 mm/s), w(int16 mrad/s), 时长(uint16 ms)
//      0x02 急停           空
//      0x03 定角度转向      int16 mrad，带符号，正 = 左转
//      0x04 中止转向        空
//      0x05 轮速环 PID      Kp, Ki, Kd 各 uint16（×1000）
//      0x20 IMU 读一帧      空
//      0x21 IMU 上报开关    uint8，0 = 关 / 1 = 开
//      0x22 陀螺零偏标定    uint16 采样数（0 = 用默认）
//      0x23 扫描 I2C 总线   空
//      0x24 姿态读一帧      空
//      0x25 姿态上报开关    uint8
//      0x26 yaw 清零        空
//      0x27 里程计读一帧    空
//      0x28 里程计上报开关  uint8
//      0x29 里程计归零      空
//      0x2A 距离刻度设置    uint16（×10000）
//      0x2B yaw 环 PID      Kp, Ki, Kd 各 uint16（×10000，注意与 0x05 不同）
//      0x2C 帧接收统计      空
//      0x2D [DATA] 上报开关 uint8
//      0x2E 读距离刻度      空
//
//    测试前务必：车轮架空离地！首次测试建议从慢速开始 —— 0x01 帧的 v 给 200 mm/s
//    （满油门是 300 mm/s，别一上来就发满）。
//    0x03 的首次测试务必也架空 —— 先看方向对不对、能不能自己停，再落地。
//    IMU 标定（0x22）时务必：车体完全静止，不要碰车、不要在桌上敲键盘。
// ============================================================================

// ---- 运行参数 ----
static constexpr uint32_t CONTROL_PERIOD_MS = 50;     // 定时器节拍 = 闭环周期
static constexpr uint32_t STARTUP_DELAY_MS = 3000;    // 上电延时，留时间架车 / 放稳
static constexpr uint32_t DEFAULT_MANEUVER_MS = 1000; // 运动指令默认持续时长
static constexpr uint32_t MAX_MANEUVER_MS = 10000;    // 单条指令时长上限，防误输入长时长

// 这里没有「掉头时长」这个常数：掉头走 yaw 闭环，转角由角度定不由时间定，
// 一个「转 180° 该给多少毫秒」的值根本不存在（原因见 yaw_control.h）。
// 同理也没有「速度档位」—— 帧协议直接给物理量（mm/s、mrad/s），不需要中间那层。

// ---- 协议帧的物理量 ↔ 归一化油门的换算 ----
// 协议里传的是物理量（mm/s、mrad/s），而底盘要的是归一化油门 [-1, +1]，
// 中间这两个常数就是翻译表。**必须和上位机 test/car_upper.py 顶部的
// CHASSIS_MAX_SPEED_CM_S / CHASSIS_SPIN_FULL_SCALE_MRAD_S 一致** ——
// 不一致的话车照样会动，只是速度不对，没有任何东西会报错。
//
// 线速度：满油门 = CHASSIS_MAX_SPEED_CM_S cm/s = 这个数 × 10 mm/s。
//   所以 v 油门 = v_mm_s / (30 × 10) = v_mm_s / 300。
static constexpr float PROTO_MM_S_PER_FULL_THROTTLE = CHASSIS_MAX_SPEED_CM_S * 10.0f;

// 角速度：满油门转多少 mrad/s。**标称值，不是标定值** —— 它是从上位机那边
// 一次实测反推的（定时开环 2000ms @ 50% 转了 284.5°，即 284.5 °/s = 4966 mrad/s），
// 取整成 5000（≈286 °/s）。它只决定「100% 转向」对应多少 mrad/s，两边用同一个数就一致。
// 要按物理量精确控角速度得先量出轮距 L（odometry.h 里也还标着「轮距没标定」）。
static constexpr float PROTO_SPIN_FULL_SCALE_MRAD_S = 5000.0f;

// 毫弧度 → 度：1 mrad = 0.0572957795°（180/π/1000）。
// 除法比乘法慢不了多少，但写成乘法少一次函数调用，而且和上位机的 MRAD_PER_DEG 对称。
static constexpr float MRAD_TO_DEG = 0.05729577951f;

// [DATA] 底盘数据上报周期。200ms = 5Hz，与 IMU / 姿态 / 里程计三路一致。
//
// 不要挂到 50ms 的闭环节拍上直接发 —— 那就是 20Hz，每秒 20 行、每行 90 来个字符，
// 按一下按钮发出的 `[CMD]` 回显、启动日志、报错全被冲得找不到。
// 5Hz 画曲线足够：一秒 5 个点就看得出速度和目标跟得紧不紧。
// 嫌快就调大：500 = 2Hz，1000 = 1Hz。
static constexpr uint32_t DATA_REPORT_PERIOD_MS = 200;

// [IMU] 连续上报周期：200ms = 5Hz，和另外三个上报一致
static constexpr uint32_t IMU_REPORT_PERIOD_MS = 200;

// [ATT] 姿态上报周期。解算本身是 100Hz，但上报 5Hz 就够了 ——
// 姿态角变化比原始数据慢得多，20Hz 刷屏只会让人看不清。
static constexpr uint32_t ATT_REPORT_PERIOD_MS = 200;

// [ODOM] 里程计上报周期。也是 5Hz —— 位置变化比姿态还慢，快了没意义。
static constexpr uint32_t ODOM_REPORT_PERIOD_MS = 200;

// [FRM] 收帧异常告警的限流周期。不是上报周期 —— 它只在错误计数器变大时出声。
//
// 2 秒是「盯着串口看的时候不算刷屏」和「不看串口的时候漏不掉」之间的折中。
// 调小只是更吵（报的是累计值，不是增量，漏不掉任何信息）；调大则一段短促的
// 干扰可能在两次告警之间被合并掉。
static constexpr uint32_t FRAME_WARN_PERIOD_MS = 2000;

// ---- 运行状态 ----
static Ticker control_ticker;
static volatile bool g_control_due = false; // 定时器只置位，计算全在 loop 里做

static bool g_report_enabled = true;
static bool g_imu_report_enabled = false; // IMU 连续上报默认关，调姿态时再开
static bool g_att_report_enabled = false; // 姿态连续上报同理
static bool g_odom_report_enabled = false; // 里程计连续上报同理

static bool g_maneuver_active = false;
static uint32_t g_maneuver_end_ms = 0;
static const char *g_maneuver_name = "";

static void on_control_tick()
{
  g_control_due = true;
}

// 启动一个定时运动：抢占底盘 → 下运动指令 → 挂牌定时。
//
// 参数是归一化油门 (v, w)，和 chassis_drive() 一致 —— 这是**唯一**一条
// 「按下就跑一段时间」的入口，协议帧的 0x01 走到这里，v/w 由上位机直接给
// （mm/s、mrad/s 换算过来的）。
//
// 定时运动和 yaw 闭环都在写同一个底盘，必须互斥 —— 否则一个说「往左转」
// 一个说「到目标了停车」，每 50ms 抢一次，车会抖。定时运动是后发的指令，
// 所以由它抢占：中止正在进行的 yaw 闭环。
//
// ⚠ 上面三步的顺序不能换，而且必须绑在同一个函数里。抢占用的
// yaw_control_abort() 会 chassis_stop()（清目标 + 清 PID + PWM 归零），
// 所以「先下运动指令、再抢占」会把刚设好的速度原样抹掉 —— 车收到指令却
// 一动不动，偏偏日志看起来完全正常（照样打印「原地右转，2000 ms」，
// 两秒后打印「结束，停车」）。三步绑在一起，调用方就没有写反的机会。
static void start_maneuver(const char *name, float v, float w, uint32_t duration_ms)
{
  yaw_control_abort();

  if (duration_ms == 0U) duration_ms = DEFAULT_MANEUVER_MS;
  if (duration_ms > MAX_MANEUVER_MS) {
    // 悄悄夹住会很难查：上位机说 30 秒，车跑 10 秒就停，看着像「运动指令提前
    // 结束」。所以夹的时候必须说一声。
    Serial.printf("[ERR] 时长 %u ms 超过上限，按 %u ms 执行\n", duration_ms, MAX_MANEUVER_MS);
    duration_ms = MAX_MANEUVER_MS;
  }

  chassis_drive(v, w);  // 抢占之后才下运动指令，否则会被上面的停车抹掉

  g_maneuver_active = true;
  g_maneuver_end_ms = millis() + duration_ms;
  g_maneuver_name = name;

  // 打印归一化百分比而不是原始 v/w：协议那头给的是 mm/s、mrad/s，
  // 换算成百分比之后日志和底盘内部的量纲一致，对着调 PID 时不用心算。
  Serial.printf("[CMD] %s，%u ms，v = %+d%%，w = %+d%%\n", name, duration_ms,
                static_cast<int>(lroundf(v * 100.0f)),
                static_cast<int>(lroundf(w * 100.0f)));
}

static void stop_now(const char *reason)
{
  yaw_control_abort();
  g_maneuver_active = false;
  chassis_stop();
  Serial.printf("[CMD] %s，已停车\n", reason);
}

static void report_data()
{
  const ChassisState s = chassis_get_state();
  Serial.printf("[DATA] enc_l=%ld,enc_r=%ld,speed_l=%.2f,speed_r=%.2f,"
                "tgt_l=%.2f,tgt_r=%.2f,pwm_l=%d,pwm_r=%d\n",
                static_cast<long>(s.enc_left), static_cast<long>(s.enc_right),
                s.speed_left_cm_s, s.speed_right_cm_s,
                s.target_left_cm_s, s.target_right_cm_s,
                s.pwm_left, s.pwm_right);
}

// [DATA] 连续上报（非阻塞，靠 millis 分频）。
//
// 分频这件事**不能挂在闭环节拍上**：闭环是 50ms 一次，挂上去就是 20Hz。
// 挪到 loop 里自己按 millis 判，才和 IMU / 姿态 / 里程计一个写法。
static void poll_data_report()
{
  if (!g_report_enabled) return;

  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (now - last_ms < DATA_REPORT_PERIOD_MS) return;
  last_ms = now;

  report_data();
}

// ---- IMU ----

// 打印一帧 IMU 数据。
// 用与 [DATA] 相同的 key=value 格式，上位机复用同一套解析；`[IMU] DATA ` 前缀
// 让它和标定进度、扫描结果这些纯文本行区分开（那些直接进日志）。
//
// amag（加速度合矢量）是判断加速度计好坏的唯一硬指标：静止时必须 ≈ 1.000。
// 偏离太多说明量程选错（数据全削顶）或芯片异常。
// bias=1 表示这帧的角速度已扣过零偏。
static void print_imu()
{
  if (!imu_is_online()) {
    Serial.println(F("[IMU] 芯片不在线 —— 发 0x23 号帧扫描 I2C 总线排查接线"));
    return;
  }

  ImuSample s;
  if (!imu_read(&s)) {
    Serial.println(F("[IMU] 读取失败（I2C 无应答）—— 发 0x23 号帧扫描总线"));
    return;
  }

  const float a_mag = sqrtf(s.accel_x_g * s.accel_x_g +
                            s.accel_y_g * s.accel_y_g +
                            s.accel_z_g * s.accel_z_g);

  Serial.printf("[IMU] DATA ax=%.3f,ay=%.3f,az=%.3f,amag=%.3f,"
                "gx=%.2f,gy=%.2f,gz=%.2f,temp=%.1f,bias=%d\n",
                s.accel_x_g, s.accel_y_g, s.accel_z_g, a_mag,
                s.gyro_x_dps, s.gyro_y_dps, s.gyro_z_dps,
                s.temp_c, s.bias_applied ? 1 : 0);
}

// 扫描 I2C 总线：接线对不对、地址是不是 0x68，一跑就知道
static void scan_i2c_bus()
{
  static constexpr uint8_t MAX_FOUND = 8;
  uint8_t addrs[MAX_FOUND] = {0};

  const uint8_t found = imu_i2c_scan(addrs, MAX_FOUND);

  if (found == 0) {
    Serial.println(F("[IMU] I2C 总线没扫到任何设备，逐项查："));
    Serial.println(F("[IMU]   1) VCC 是否接 3V3、GND 是否与 ESP32 共地"));
    Serial.println(F("[IMU]   2) SDA 是否接 GPIO8、SCL 是否接 GPIO9（别接反）"));
    Serial.println(F("[IMU]   3) 杜邦线是否松动 / 模块电源灯是否亮"));
    return;
  }

  Serial.printf("[IMU] I2C 总线发现 %u 个设备：", found);
  for (uint8_t i = 0; i < found && i < MAX_FOUND; i++) {
    Serial.printf(" 0x%02X%s", addrs[i], (addrs[i] == IMU_I2C_ADDR) ? "(MPU6050)" : "");
  }
  Serial.println();

  // 扫到了设备但不是 0x68 —— 最典型的原因是 AD0 接了 3V3，地址变成 0x69
  bool has_mpu = false;
  for (uint8_t i = 0; i < found && i < MAX_FOUND; i++) {
    if (addrs[i] == IMU_I2C_ADDR) has_mpu = true;
  }
  if (!has_mpu) {
    Serial.printf("[IMU]   没看到 0x%02X：若上面有 0x69，说明 AD0 接了 3V3，"
                  "要么把 AD0 接地，要么改 IMU_I2C_ADDR\n", IMU_I2C_ADDR);
  }
}

static void start_gyro_calibration(uint16_t samples)
{
  if (!imu_is_online()) {
    Serial.println(F("[IMU] 芯片不在线，无法标定 —— 先发 0x23 号帧排查接线"));
    return;
  }
  if (imu_calibrate_busy()) {
    Serial.println(F("[ERR] 上一次标定还没结束，等它跑完"));
    return;
  }

  // 这里的限幅跟驱动层保持一致，只是为了把真实样本数打进日志
  if (samples == 0U) samples = IMU_CAL_DEFAULT_SAMPLES;
  if (samples > IMU_CAL_MAX_SAMPLES) samples = IMU_CAL_MAX_SAMPLES;

  imu_calibrate_start(samples);
  Serial.printf("[CMD] 零偏标定开始（%u 样本 ≈ %.0f 秒）—— 车体必须完全静止！\n",
                samples, static_cast<float>(samples) / static_cast<float>(IMU_SAMPLE_RATE_HZ));
}

// 开机自动标定。此时 Ticker 还没挂载、电机已断电，阻塞是安全的。
static void auto_calibrate_gyro()
{
  if (!imu_is_online()) return;

  Serial.printf("[IMU] 开机零偏标定：%.0f 秒内保持车体静止...\n",
                static_cast<float>(IMU_CAL_DEFAULT_SAMPLES) / static_cast<float>(IMU_SAMPLE_RATE_HZ));

  imu_calibrate_start();

  uint8_t last_step = 0;
  while (imu_calibrate_busy()) {
    imu_calibrate_update();

    const uint8_t step = static_cast<uint8_t>(imu_calibrate_percent() / 20U);
    if (step != last_step) {
      last_step = step;
      Serial.printf("[IMU] 标定 %u%%\n", static_cast<unsigned>(step) * 20U);
    }
    delay(1);
  }

  const ImuGyroBias b = imu_get_bias();
  Serial.printf("[IMU] 标定完成（%u 样本）：零偏 = %+.3f / %+.3f / %+.3f °/s\n",
                b.samples, b.x_dps, b.y_dps, b.z_dps);
}

// IMU 连续上报（非阻塞，靠 millis 分频）
static void poll_imu_report()
{
  if (!g_imu_report_enabled) return;

  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (now - last_ms < IMU_REPORT_PERIOD_MS) return;
  last_ms = now;

  print_imu();
}

// ---- 姿态角 ----

// 车轮速度低于这个值就认为车没在动（单位 cm/s）。
//
// 为什么是 3.0 这么宽：按本工程的参数（1750 counts/rev、轮径 6.5cm），1 个编码器
// 计数 ≈ 0.0117cm，50ms 测速窗口里只要 5 个噪声计数就能把速度顶过 1.0cm/s ——
// 电机 PWM 走线旁边的编码器线很容易收到这个量级的毛刺。阈值取 1.0 太紧，3.0
// 相当于 256 个噪声计数/50ms，才留出余量。
//
// 这个门槛为什么必须够宽： 「车没动」是静止修正的入场券，而 attitude.cpp 里
// **任何一拍不满足就把已攒的静止时间清零重来**。门槛太紧的表现是：still 一直闪、
// 永远攒不满 0.5s、yaw 永不冻结 —— 现象正是「车明明停着，yaw 却一直在跳」。
//
// 放宽不会误判真实运动：车真在转的话陀螺那边有一票否决（见 attitude.h），
// 而在动但没转的时候 yaw 本来也不该变。
static constexpr float ATTITUDE_STILL_SPEED_CM_S = 3.0f;

// 车是不是停着。判据只用编码器 —— 解算层拿到这个标志后会再叠一层
// 「陀螺是否安静」的判断，两个都满足才算真静止（见 attitude.h 的静止修正）。
static bool vehicle_is_stopped()
{
  if (g_maneuver_active) return false; // 正在执行运动指令，肯定不算停

  const ChassisState s = chassis_get_state();
  return (fabsf(s.speed_left_cm_s) < ATTITUDE_STILL_SPEED_CM_S) &&
         (fabsf(s.speed_right_cm_s) < ATTITUDE_STILL_SPEED_CM_S);
}

// 打印一帧姿态角。
// rate 是实测解算频率：正常应该贴着 100，掉到七八十说明 loop 被拖慢了，
// 那时 dt 会变大、积分误差跟着变大，属于要先解决的底层问题。
// still=1 表示正在做静止修正（yaw 冻结 + 零偏在线跟踪）—— 验证这个功能就看它。
static void print_attitude()
{
  if (!imu_is_online()) {
    Serial.println(F("[ATT] IMU 不在线 —— 发 0x23 号帧扫描 I2C 总线排查接线"));
    return;
  }

  const Attitude a = attitude_get();
  if (!a.valid) {
    Serial.println(F("[ATT] 姿态未就绪：IMU 还没读到可用数据（或车在动，等停稳）"));
    return;
  }

  // ytgt / ybusy 是给定角度转向用的：GUI 把 ytgt 画成一条横线，
  // 就能直接看出「车有没有朝目标收敛、超调多少、最后停在哪」。
  // 没在转的时候 ytgt 保留上一轮的目标值（方便回看残差），ybusy=0。
  Serial.printf("[ATT] DATA roll=%.2f,pitch=%.2f,yaw=%.2f,rate=%u,still=%d,"
                "ytgt=%.2f,ybusy=%d\n",
                a.roll_deg, a.pitch_deg, a.yaw_deg,
                static_cast<unsigned>(attitude_get_rate_hz()),
                attitude_is_still() ? 1 : 0,
                yaw_control_get_target_deg(),
                yaw_control_busy() ? 1 : 0);
}

// 姿态连续上报（非阻塞，靠 millis 分频）
static void poll_att_report()
{
  if (!g_att_report_enabled) return;

  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (now - last_ms < ATT_REPORT_PERIOD_MS) return;
  last_ms = now;

  print_attitude();
}

// ---- 里程计 ----

// 打印一帧里程计。
// scale 一起报出来是为了让标定过程自解释：发完 0x2A 立刻能看到当前用的是哪个系数，
// 不用回去翻自己发过什么。scale 还是 1.000 就说明没标定过，读到 dist 也别当真。
static void print_odom()
{
  if (!odom_is_ready()) {
    Serial.println(F("[ODOM] 未就绪：IMU 还没读出可用姿态（里程计靠 yaw 定航向）"));
    return;
  }

  const OdomState o = odom_get();
  Serial.printf("[ODOM] DATA x=%.2f,y=%.2f,th=%.2f,dist=%.2f,"
                "v=%.2f,w=%.2f,scale=%.4f\n",
                o.x_cm, o.y_cm, o.theta_deg, o.dist_cm,
                o.v_cm_s, o.w_dps, odom_get_dist_scale());
}

static void poll_odom_report()
{
  if (!g_odom_report_enabled) return;

  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (now - last_ms < ODOM_REPORT_PERIOD_MS) return;
  last_ms = now;

  print_odom();
}

// ---- 定角度转向（yaw 闭环）----

// 结束原因的说明文字。三种失败要能一眼分清是「调参问题」还是「硬件问题」，
// 否则看到车没转到就只能猜。
static const char *yaw_result_text(YawResult r)
{
  switch (r) {
    case YawResult::ARRIVED: return "到位";
    case YawResult::TIMEOUT: return "超时未到位";
    case YawResult::STALLED: return "卡住（使劲了但车没动）";
    case YawResult::ABORTED: return "已中止";
    default: return "结束";
  }
}

static void report_yaw_result(YawResult r)
{
  const Attitude a = attitude_get();
  Serial.printf("[YAW] %s：目标 %.1f°，实际 %.1f°，残差 %+.1f°，用时 %u ms\n",
                yaw_result_text(r), yaw_control_get_target_deg(), a.yaw_deg,
                yaw_control_get_error_deg(),
                static_cast<unsigned>(yaw_control_get_elapsed_ms()));

  if (r == YawResult::STALLED) {
    Serial.println(F("[YAW] 排查顺序：① 车轮是否被架空了（空转不产生转角）"
                     "② 地面是否太涩转不动 ③ 电池是否没电 ④ Ki 是否太小捅不过 PWM 死区"));
  } else if (r == YawResult::TIMEOUT) {
    Serial.println(F("[YAW] 排查顺序：① Kp 是否太小（转得太肉）"
                     "② 目标角是否太大（8 秒转不完）③ 是否一直超调在目标附近来回摆"));
  }
}

static void start_yaw_turn(float delta_deg)
{
  // 先确认姿态有效再抢底盘 —— 否则姿态没好，还要先把正在跑的运动指令停掉
  if (!attitude_get().valid) {
    Serial.println(F("[ERR] 姿态未就绪，无法定角度转向（IMU 没数据或车还在动）"));
    return;
  }

  // 抢占：定时运动指令和 yaw 闭环都在写同一个底盘，只能有一个主人。
  // 抢之前先停车，让车从静止开始 —— 否则残余速度会让外环一上来就超调。
  g_maneuver_active = false;
  chassis_stop();

  if (!yaw_control_start(delta_deg)) {
    Serial.println(F("[ERR] 启动失败，姿态数据不可用"));
    return;
  }

  Serial.printf("[CMD] 定角度转向 %+.1f°（目标 %.1f°），yaw 闭环接管 —— 转到了自己停\n",
                delta_deg, yaw_control_get_target_deg());
}

// 开机打一张功能码速查表。名字从 frame_func_name() 取，**不另抄一份** ——
// 抄一份就会随着改码而悄悄过期，那种过期没有任何东西会报错。
//
// 数据域的编码（每个字段几字节、什么量纲）刻意不在这里重复：那是两边的契约，
// 权威版本在 串口通信手册.md。这里只负责让人把码和名字对上号。
static void print_code_table()
{
  static const uint8_t codes[] = {
      FC_CHASSIS_DRIVE, FC_CHASSIS_STOP, FC_TURN_ANGLE, FC_YAW_ABORT, FC_CHASSIS_PID,
      FC_IMU_READ, FC_IMU_REPORT, FC_IMU_CALIBRATE, FC_I2C_SCAN,
      FC_ATT_READ, FC_ATT_REPORT, FC_ATT_ZERO,
      FC_ODOM_READ, FC_ODOM_REPORT, FC_ODOM_RESET, FC_ODOM_SET_SCALE,
      FC_YAW_PID, FC_FRAME_STATS, FC_DATA_REPORT, FC_ODOM_GET_SCALE,
  };

  Serial.println(F("[HELP] 功能码速查（数据域字节布局见 串口通信手册.md）："));
  for (uint8_t i = 0; i < sizeof(codes); i++) {
    Serial.printf("  0x%02X  %s\n", codes[i], frame_func_name(codes[i]));
  }
  Serial.println(F("[HELP] 本机地址 0x01（0xFF 广播也收）；回复以文本日志行给出"));
}

// 帧接收统计。串口一有说不清的问题，第一个该看的就是它 —— 几个计数器
// 就能把「没发」「发错」「线有问题」这三类分干净，不用猜。
static void print_frame_stats()
{
  const FrameStats s = frame_get_stats();
  Serial.printf("[FRM] 好帧 %u / CRC 错 %u / 长度超限 %u / 半帧超时 %u / 缓存溢出 %u\n",
                s.good, s.bad_crc, s.oversize, s.timeout, s.dropped);
  Serial.printf("[FRM] 缓存里还有 %u 字节未消化，本机地址 = %u\n",
                frame_pending(), FRAME_ADDR_LOCAL);
  Serial.println(F("[FRM] 全是 0 = 一条帧都没收到过，先确认上位机发的是不是数据帧"));
  Serial.println(F("[FRM] CRC 错一直涨 = 字节收到了但内容不对：查波特率、共地、线长"));
}

// 收帧异常主动告警。
//
// 为什么非有不可：0x2C 就是上面那个 print_frame_stats()，可它有个循环依赖 ——
// **帧本身收不进来的时候，这条帧也发不进来**，没有任何「不依赖收帧」的查询手段。
// 所以错误计数器必须自己出声。
//
// 只报「计数器变大」，不报「好帧为 0」：板子上电后没人发帧是常态（上位机没开
// 就是这个状态），报它是纯噪声。计数器一动才说明确实有字节在进来、只是内容
// 不对 —— 那才是需要人当场知道的。
//
// ⚠ 它报不出「一个字节都没收到」（TX 没接 / 线断 / 上位机压根没发）。那种情况
//   和「安静待机」在固件眼里完全一样，没有任何信息能把两者分开。要分辨只能从
//   上位机那头看：它有没有收到板子的 [ATT] / [DATA] 输出。
static void poll_frame_stats_warning()
{
  static FrameStats prev = {0, 0, 0, 0, 0};
  static uint32_t last_ms = 0;

  const FrameStats s = frame_get_stats();
  const bool changed = (s.bad_crc != prev.bad_crc) ||
                       (s.oversize != prev.oversize) ||
                       (s.timeout != prev.timeout) ||
                       (s.dropped != prev.dropped);
  if (!changed) return;

  const uint32_t now = millis();
  if (now - last_ms < FRAME_WARN_PERIOD_MS) return;  // 攒着，到点一次性报

  // prev 只在真打印时才推进。反过来写（先推进再判限流）会把限流窗口里攒下的
  // 那批错误永远吞掉：窗口过后 s == prev，判定「没变化」，就再也不报了。
  // 报的是累计值不是增量，所以晚报 2 秒不丢任何信息。
  last_ms = now;
  prev = s;

  Serial.printf("[FRM] 收帧异常：CRC 错 %u / 长度超限 %u / 半帧超时 %u / 缓存溢出 %u（好帧 %u）\n",
                static_cast<unsigned>(s.bad_crc), static_cast<unsigned>(s.oversize),
                static_cast<unsigned>(s.timeout), static_cast<unsigned>(s.dropped),
                static_cast<unsigned>(s.good));
  Serial.println(F("[FRM] 字节收到了但内容不对：查波特率(115200)、共地、线长、上位机那边的 CRC"));
}

// ============================================================================
//  协议帧的处理 —— 把解出来的一帧变成动作
//
//  这是下行的唯一入口。动作全部落到四个函数上：
//  start_maneuver() / start_yaw_turn() / stop_now() / print_*()。
//
//  分工：serial_frame 只管「字节 → 帧」，不解释任何字段的含义；
//  到这里才知道什么是速度、什么是角度，才决定去动底盘还是去读传感器。
//
//  ⚠ 每条 case 都**先核对数据域长度**再取字段。长度对不上说明两边对这条指令的
//    理解不一致（版本没对上，或者帧被改过），这时按自己的理解硬读，读到的可能
//    是别的字段的一半，车会做出一个谁都解释不了的动作。宁可整帧丢掉。
// ============================================================================

static float clamp_throttle(float v)
{
  if (v > CHASSIS_THROTTLE_MAX) return CHASSIS_THROTTLE_MAX;
  if (v < -CHASSIS_THROTTLE_MAX) return -CHASSIS_THROTTLE_MAX;
  return v;
}

// 数据域长度不对时的统一报错。
//
// 长度对不上 = 两边对这条指令的理解不一致（版本没对上、或者帧被改过）。
// 这时**绝不能按自己的理解硬读** —— 那样读到的可能是别的字段的一半，
// 车会做出一个谁都解释不了的动作。宁可丢掉这一帧。
static void frame_len_error(const Frame &f, uint8_t want)
{
  Serial.printf("[ERR] %s(0x%02X) 的数据域应为 %u 字节，收到 %u 字节 —— 已丢弃\n",
                frame_func_name(f.func), f.func, want, f.len);
}

static void handle_frame(const Frame &f)
{
  // ---- 地址过滤 ----
  // 不是发给本机的就丢掉，但**要吭声**：地址配错意味着上位机连到了别的节点，
  // 默默丢掉的话现场表现是「按了没反应」，比一句报错难查得多。
  if (f.addr != FRAME_ADDR_LOCAL && f.addr != FRAME_ADDR_BROADCAST) {
    Serial.printf("[ERR] 帧地址 %u 不是本机（%u），已丢弃\n", f.addr, FRAME_ADDR_LOCAL);
    return;
  }

  switch (f.func) {
    // ---- 底盘控制类 ----

    case FC_CHASSIS_DRIVE: {
      if (f.len != 6) { frame_len_error(f, 6); return; }
      const float v =
          static_cast<float>(frame_rd_i16(f.payload + 0)) / PROTO_MM_S_PER_FULL_THROTTLE;
      const float w =
          static_cast<float>(frame_rd_i16(f.payload + 2)) / PROTO_SPIN_FULL_SCALE_MRAD_S;
      const uint32_t ms = frame_rd_u16(f.payload + 4);
      // 协议允许发超出满油门的数（比如 1000 mm/s），底盘吃不下。夹在这里而不是
      // 指望 chassis_drive 内部处理，是为了让日志打出来的是**真实执行值**。
      start_maneuver("底盘速度控制", clamp_throttle(v), clamp_throttle(w), ms);
      return;
    }

    case FC_CHASSIS_STOP:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      stop_now("急停帧");
      return;

    case FC_TURN_ANGLE: {
      if (f.len != 2) { frame_len_error(f, 2); return; }
      // 目标角上限由 yaw_control 的 YAW_TARGET_MAX_DEG 把关，这里不预先夹：
      // 量程是多少只有它自己知道，夹两遍反而会出现「谁夹的」说不清的情况。
      start_yaw_turn(static_cast<float>(frame_rd_i16(f.payload)) * MRAD_TO_DEG);
      return;
    }

    case FC_YAW_ABORT:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      if (!yaw_control_busy()) {
        Serial.println(F("[CMD] 当前没有正在进行的定角度转向"));
        return;
      }
      // abort 只停车、不清目标角和误差，所以先中止再读
      // 拿到的是**这一轮的最终残差**（下一轮开始才会被覆盖）。
      yaw_control_abort();
      Serial.printf("[YAW] 上位机中止：目标 %.1f°，实际 %.1f°，残差 %+.1f°\n",
                    yaw_control_get_target_deg(), attitude_get().yaw_deg,
                    yaw_control_get_error_deg());
      return;

    case FC_CHASSIS_PID: {
      if (f.len != 6) { frame_len_error(f, 6); return; }
      // 增益在协议里是 ×1000 的整数，回来要除回去。字面量写 1000.0f 而不是
      // 1000.0：ESP32-S3 的 double 是软件模拟的，掺一个 double 字面量进去会让
      // 整个表达式落到软件浮点上，比硬件 float 慢一个数量级。
      const float kp = static_cast<float>(frame_rd_u16(f.payload + 0)) / 1000.0f;
      const float ki = static_cast<float>(frame_rd_u16(f.payload + 2)) / 1000.0f;
      const float kd = static_cast<float>(frame_rd_u16(f.payload + 4)) / 1000.0f;
      chassis_set_pid(kp, ki, kd);
      Serial.printf("[CMD] 轮速环 PID = %.3f / %.3f / %.3f\n", kp, ki, kd);
      return;
    }

    // ---- 传感器 / 配置类 ----

    case FC_IMU_READ:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      print_imu();
      return;

    case FC_IMU_REPORT:
      if (f.len != 1) { frame_len_error(f, 1); return; }
      // 这里是**置位**不是切换 —— 界面得自己记着开没开，一旦和固件不同步
      // （板子复位、另一台上位机改过）就一直是反的，现象是「点了没反应」。
      // 帧协议带的是目标状态，发几次结果都一样。
      g_imu_report_enabled = (f.payload[0] != 0);
      Serial.printf("[CMD] IMU 连续上报 %s（5Hz）\n", g_imu_report_enabled ? "已开启" : "已关闭");
      return;

    case FC_IMU_CALIBRATE:
      if (f.len != 2) { frame_len_error(f, 2); return; }
      // 0 = 用固件默认采样数。限幅交给 start_gyro_calibration，它才知道上下限。
      start_gyro_calibration(frame_rd_u16(f.payload));
      return;

    case FC_I2C_SCAN:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      scan_i2c_bus();
      return;

    case FC_ATT_READ:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      print_attitude();
      return;

    case FC_ATT_REPORT:
      if (f.len != 1) { frame_len_error(f, 1); return; }
      g_att_report_enabled = (f.payload[0] != 0);
      Serial.printf("[CMD] 姿态连续上报 %s（5Hz）\n", g_att_report_enabled ? "已开启" : "已关闭");
      return;

    case FC_ATT_ZERO:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      // 先确认姿态有效，否则清了个无效值、用户以为生效了
      if (!attitude_get().valid) {
        Serial.println(F("[ERR] 姿态未就绪，无法清零 yaw"));
        return;
      }
      attitude_zero_yaw();
      Serial.println(F("[CMD] yaw 已清零，当前朝向 = 0°"));
      return;

    case FC_ODOM_READ:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      print_odom();
      return;

    case FC_ODOM_REPORT:
      if (f.len != 1) { frame_len_error(f, 1); return; }
      g_odom_report_enabled = (f.payload[0] != 0);
      Serial.printf("[CMD] 里程计连续上报 %s（5Hz）\n", g_odom_report_enabled ? "已开启" : "已关闭");
      return;

    case FC_ODOM_RESET:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      if (!odom_is_ready()) {
        Serial.println(F("[ERR] 里程计未就绪，无法归零"));
        return;
      }
      odom_reset();
      Serial.println(F("[CMD] 里程计已归零：当前位置 = 原点，当前朝向 = 0°"));
      Serial.println(F("[CMD] （只动了里程计的坐标系；IMU 的 yaw 没变，仍是累计值）"));
      return;

    case FC_ODOM_SET_SCALE:
      if (f.len != 2) { frame_len_error(f, 2); return; }
      // 刻度在协议里是 ×10000 的整数。不在这里判范围 —— odom_set_dist_scale()
      // 自己会夹到 [0.2, 5.0]，而且回读值才是真相，所以打印的是回读值。
      odom_set_dist_scale(static_cast<float>(frame_rd_u16(f.payload)) / 10000.0f);
      Serial.printf("[CMD] 距离刻度 = %.4f（超出 0.2~5.0 会被夹住）\n",
                    static_cast<double>(odom_get_dist_scale()));
      return;

    // ---- 补齐的四条（0x2B~0x2E）----

    case FC_YAW_PID: {
      // 目标不是 0x05 那个环 —— 0x05 是内环轮速，这条是外环 yaw。改哪个环要看
      // 现象：轮速跟不上目标用 0x05，转过头 / 停不准用这条。
      // ⚠ 量化也不同：这条是 ÷10000，0x05 是 ÷1000（理由见上面常量定义处）。
      if (f.len != 6) { frame_len_error(f, 6); return; }
      // 字面量同样写 10000.0f 而不是 10000.0，理由见 0x05 那条。
      const float kp = static_cast<float>(frame_rd_u16(f.payload + 0)) / 10000.0f;
      const float ki = static_cast<float>(frame_rd_u16(f.payload + 2)) / 10000.0f;
      const float kd = static_cast<float>(frame_rd_u16(f.payload + 4)) / 10000.0f;
      yaw_control_set_pid(kp, ki, kd);
      // 打印**回读值**而不是入参：入参只是从这一帧里解出来的数，回读值才是环里
      // 真正生效的那个。当前两者必然相等（yaw_control_set_pid 按原值存入，不做
      // 夹取）—— 但改这里去打实际生效值，以后加了范围限制也不会撒谎。
      float rkp = 0.0f, rki = 0.0f, rkd = 0.0f;
      yaw_control_get_pid(&rkp, &rki, &rkd);
      Serial.printf("[CMD] yaw 环 PID = %.4f / %.4f / %.4f\n",
                    static_cast<double>(rkp), static_cast<double>(rki),
                    static_cast<double>(rkd));
      return;
    }

    case FC_FRAME_STATS:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      // 回的是文本日志，不是应答帧 —— 帧协议只统一了上位机→下位机这个方向。
      // 另外：帧本身收不到的时候这条也发不进来，那种情况靠错误计数器的
      // 主动告警（poll_frame_stats_warning）兜底。
      print_frame_stats();
      return;

    case FC_DATA_REPORT:
      if (f.len != 1) { frame_len_error(f, 1); return; }
      // 置位不是切换，理由同 0x21 / 0x25 / 0x28。
      g_report_enabled = (f.payload[0] != 0);
      Serial.printf("[CMD] 底盘数据上报 %s（5Hz）\n", g_report_enabled ? "已开启" : "已关闭");
      return;

    case FC_ODOM_GET_SCALE:
      if (f.len != 0) { frame_len_error(f, 0); return; }
      // 只读。写入是 0x2A。不需要 odom_is_ready() —— 刻度是个纯配置量，
      // 和里程计有没有解算出来无关。
      Serial.printf("[CMD] 当前距离刻度 = %.4f（1.0 = 未标定）\n",
                    static_cast<double>(odom_get_dist_scale()));
      return;

    default:
      // 未知功能码要连序号一起报：序号能对上上位机的日志，立刻知道是哪一条帧。
      Serial.printf("[ERR] 未知功能码 0x%02X（帧序号 %u，%u 字节数据域）—— 已丢弃\n",
                    f.func, f.seq, f.len);
      return;
  }
}

// 串口收信：把线上读到的字节交给 serial_frame，解出一帧就回调 handle_frame。
//
// 不需要上层做任何分流 —— 不认识的字节（噪声、丢同步后的残渣、上电瞬间的乱码）
// 本来就该由 serial_frame 内部「挪一个字节重新对齐」消化掉，那套逻辑就是为此写的。
// 粘包、半包、半帧超时也一样封在它里面：这里只管把字节交出去。
static void poll_serial()
{
  // 一次最多读这么多。**必须按 available() 的量读，不能固定读满** ——
  // Stream::readBytes() 凑不满时会阻塞等超时，一次卡 1 秒，闭环和串口全停摆。
  static constexpr int SERIAL_RX_CHUNK = 64;
  uint8_t buf[SERIAL_RX_CHUNK];

  while (Serial.available() > 0) {
    const int avail = Serial.available();
    const int want = (avail < SERIAL_RX_CHUNK) ? avail : SERIAL_RX_CHUNK;
    const int n = Serial.readBytes(buf, want);
    if (n <= 0) break;  // 读不到就收手，别空转

    frame_feed_bytes(buf, static_cast<uint16_t>(n), handle_frame);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("[SYS] 底盘闭环（编码器 + PID）+ MPU6050 六轴 IMU + 航位推算"));

  // 帧解析器。必须在第一次 poll_serial 之前初始化 —— 它带着一个跨调用保留的
  // 半包缓存，上电时里面是随机垃圾的话，头几个字节会被当成半帧吃掉。
  frame_protocol_init();

  chassis_init();

  // IMU 初始化：失败不 abort —— 底盘是独立功能，不该被一个没插好的模块拖死
  const bool imu_ok = imu_init();
  if (imu_ok) {
    Serial.printf("[IMU] MPU6050 在线（WHO_AM_I=0x%02X），I2C %ukHz，DLPF 档 %u，"
                  "采样 %uHz，陀螺 %.0f LSB/(°/s)，加速度 %.0f LSB/g\n",
                  static_cast<unsigned>(IMU_WHO_AM_I_VALUE),
                  static_cast<unsigned>(IMU_I2C_FREQ_HZ / 1000U),
                  static_cast<unsigned>(IMU_DLPF_CFG),
                  static_cast<unsigned>(IMU_SAMPLE_RATE_HZ),
                  static_cast<double>(IMU_GYRO_LSB_PER_DPS),
                  static_cast<double>(IMU_ACCEL_LSB_PER_G));
  } else {
    Serial.println(F("[IMU] 未检测到 MPU6050 —— IMU 功能关闭，底盘不受影响"));
    Serial.println(F("[IMU] 排查：发 0x23 号帧扫描 I2C 总线"));
  }

  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  chassis_get_pid(&kp, &ki, &kd);

  Serial.printf("[SYS] %u ms 后启动，请把车架空或放稳\n", STARTUP_DELAY_MS);
  delay(STARTUP_DELAY_MS);

  // 车已放稳，此时标定陀螺零偏最准。Ticker 还没挂载，阻塞安全。
  auto_calibrate_gyro();

  // 姿态解算初始化。放在标定之后 —— 标定改变了零偏，解算必须用标定后的数据。
  attitude_init();
  if (imu_ok) {
    const float alpha = attitude_get_alpha();
    const float tau_s = alpha * (ATTITUDE_PERIOD_US * 1e-6f) / (1.0f - alpha);
    Serial.printf("[ATT] 姿态解算就绪：互补滤波 α=%.3f（τ≈%.2f 秒），解算 %uHz\n",
                  static_cast<double>(alpha), static_cast<double>(tau_s),
                  static_cast<unsigned>(1000000U / ATTITUDE_PERIOD_US));
  }

  // yaw 转向环初始化。必须在 attitude_init 之后 —— 它的输入就是姿态角。
  yaw_control_init();
  if (imu_ok) {
    float ykp = 0.0f, yki = 0.0f, ykd = 0.0f;
    yaw_control_get_pid(&ykp, &yki, &ykd);
    Serial.printf("[YAW] 定角度转向就绪：PID = %.4f / %.4f / %.4f，容差 %.1f°，"
                  "超时 %u ms\n",
                  static_cast<double>(ykp), static_cast<double>(yki),
                  static_cast<double>(ykd), static_cast<double>(YAW_TOLERANCE_DEG),
                  static_cast<unsigned>(YAW_TIMEOUT_MS));
  }

  // 里程计初始化。同样必须在 attitude_init 之后 —— 它靠 yaw 定航向。
  odom_init();
  const float dist_scale = odom_get_dist_scale();
  Serial.printf("[ODOM] 里程计就绪：距离刻度 = %.4f\n",
                static_cast<double>(dist_scale));
  if (fabsf(dist_scale - 1.0f) < 1e-4f) {
    // 没标定就说清楚后果，别让用户以为里程计装好就能用
    Serial.println(F("[ODOM] ⚠ 刻度还是 1.0（未标定）—— wheel_encoder.h 里轮径和"
                     "每转计数是标称值，走 1 米可能报成 90cm 或 110cm。"));
    Serial.println(F("[ODOM]   标定：发 0x29 归零 → 沿卷尺走 100cm → 发 0x27 读 dist → "
                     "发 0x2A 存 <100/dist>（见 odometry.h 顶部）"));
  }

  control_ticker.attach_ms(CONTROL_PERIOD_MS, on_control_tick);

  Serial.printf("[SYS] 就绪，轮速环 PID = %.3f / %.3f / %.3f\n", kp, ki, kd);
  print_code_table();
}

void loop()
{
  // IMU 零偏标定（非阻塞）：自带 100Hz 节拍，标定期间串口和底盘闭环照常跑
  if (imu_calibrate_update()) {
    const ImuGyroBias b = imu_get_bias();
    Serial.printf("[IMU] 标定完成（%u 样本）：零偏 = %+.3f / %+.3f / %+.3f °/s\n",
                  b.samples, b.x_dps, b.y_dps, b.z_dps);
  }

  // 定时器只置位，闭环计算在这里做：串口与计算互不阻塞
  if (g_control_due) {
    g_control_due = false;

    // 定角度转向（yaw 闭环）。挂在底盘闭环节拍上 —— 串级控制里外环必须
    // 与内环同步跑，否则两个环的相位关系就乱了。它和定时运动指令互斥
    // （start_yaw_turn 会抢占），所以在下面用 else if 分流。
    if (yaw_control_busy()) {
      const YawResult r = yaw_control_update();
      if (r != YawResult::RUNNING) {
        // 结束的那一拍 yaw_control_update 内部已经停车了，这里只负责上报
        report_yaw_result(r);
      }
    } else if (g_maneuver_active &&
               static_cast<int32_t>(millis() - g_maneuver_end_ms) >= 0) {
      // 运动指令到时自动停车（靠时长自终止，手动调试阶段不需要看门狗）
      g_maneuver_active = false;
      chassis_stop();
      Serial.printf("[CMD] %s 结束，停车\n", g_maneuver_name);
    }

    chassis_update();
  }

  // 姿态解算（非阻塞）：内部自带 100Hz 节拍，没到点连 I2C 都不读。
  // 放在闭环之后 —— 闭环是 50ms 的硬节拍，优先级最高，不能被 0.4ms 的 I2C 读挤到。
  // 传入「车是否停着」：解算层靠它在静止时冻结 yaw 并跟踪零偏（见 attitude.h）。
  attitude_update(vehicle_is_stopped());

  // 里程计（非阻塞，内部自带 50ms 节拍）。
  // 位置放在 attitude_update 之后、上报之前，是为了让这一拍「新采的编码器」和
  // 「刚解算出的 yaw」是配套的 —— 里程计要把两者乘在一起，差一拍就成了两帧拼接。
  // （编码器那边没问题：chassis_get_state() 是当场重新读计数的，拿到的一定是最新的。）
  odom_update();

  poll_data_report();
  poll_imu_report();
  poll_att_report();
  poll_odom_report();
  poll_serial();

  // 收帧异常告警放在最后：poll_serial() 刚把这一圈的字节喂进解析器，计数器此刻
  // 才是最新的，在这里比就是零延迟。放前面也能报出来，只是慢一圈。
  poll_frame_stats_warning();
}
