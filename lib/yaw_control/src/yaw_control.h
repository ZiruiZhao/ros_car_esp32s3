#pragma once

#include <Arduino.h>

// ============================================================================
// 定角度转向 —— yaw 闭环
//
//   发「转 90°」，车自己转过去、自己停。这是本工程第一个**闭环姿态控制**：
//   前一个模块 attitude 回答「车现在朝哪」，这个模块回答「怎么转到我想让它朝的地方」。
//
//   ---- 串级结构 ----
//
//     yaw 环（本模块，50ms）
//       误差 = 目标角 - 当前 yaw
//       输出 = 归一化转向油门 w ∈ [-1,+1]
//              ↓
//     chassis_drive(0, w)  ← 差速混速
//              ↓
//     轮速环（chassis，50ms）：目标 cm/s → 增量式 PID → PWM
//              ↓
//           电机
//
//   为什么要串级、不直接把 PWM 接到 yaw 上：
//     ① 内环已经把手洗过一遍了。chassis 只管「左右轮各转多快」，电机特性、
//        电池电压下降、左右轮摩擦不一致、死区，全被内环吸收掉了。
//        外环看到的是一个近似线性的「给 w → 得到转向角速度」的对象。
//     ② 外环因此可以做得非常笨 —— 这就是下面「只要 P」能成立的原因。
//
//   代价是外环必须比内环慢，否则两个环互相打架（外环还没看到内环的响应就开始
//   修正，修正量叠加 → 震荡）。内环目标斜坡是 8cm/s 每周期，0→30 大约 200ms；
//   外环时间常数按 0.5~1s 设计，留了 3 倍以上余量。
//
//   ---- 用归一化 w，不用物理 °/s ----
//
//   本来更自然的是让外环输出「我要转 150°/s」。但那需要轮距（左右轮中心距）：
//     ω[rad/s] = (v右 - v左) / 轮距
//   本工程没测过轮距 —— wheel_encoder.h 里只有轮径，没有左右轮中心距。
//   凭空写一个轮距进去，等于把假数据混进控制回路，调出来的参数全是错的。
//
//   所以直接用 chassis 已有的归一化单位：
//     w = 1.0 → left=-30cm/s, right=+30cm/s（满转）
//   代价是「w 对应多少 °/s」未知，Kp 只能实测整定（见下面「整定顺序」）。
//   好处是整定出来的参数跟轮距无关，以后量了轮距也不用改。
//
//   ---- 被控对象是积分器，所以 P 就够了 ----
//
//   我们控制的是**角度**，而 w 决定的是**角速度**。角速度积一次才是角度：
//     dyaw/dt = K · w        （K = w=1 时的转向角速度，约 200°/s 量级，待实测）
//   纯 P：w = Kp·误差 → dyaw/dt = K·Kp·误差
//   这是标准一阶系统，平衡点是 误差=0。**稳态误差为零** —— 只要误差还在，
//   车就还在转，一直转到误差消失为止。不需要积分项去消静差。
//
//   那为什么还留了 I：因为 PWM 死区。误差很小时 w 也很小，小到 PWM 顶不过
//   motor_driver 的死区（PWM_DEADBAND=5）和 chassis.cpp:62 的硬截断，车根本
//   不动，于是永远停不到容差以内。I 负责在这种时候把输出「推」过死区。
//   它是捅死区用的，不是消静差用的，所以给得小、还夹了上限。
//
//   ---- D 项为什么用陀螺仪读数，而不是对角度求差分 ----
//
//   数学上 d(误差)/dt = -dyaw/dt，用角度差分成 (yaw - yaw上一拍)/dt 完全等价。
//   但角度本身是积分出来的，差分等于把两拍的噪声放大 1/dt 倍（dt 只有 10ms，
//   放大 100 倍）。陀螺仪读数是**直接测量**的角速度，干净得多。
//   反正 attitude 每一拍都把扣完零偏的 gyro_z 带出来了，白用不用。
//
//   注意：D 项这里本质是阻尼 —— 「转得越快，越往回拉」。它在接近目标时把车速
//   压下来，抑制内环滞后带来的过冲。
//
//   ⚠ 这个阻尼项是**承重的，不是锦上添花**。仿真实测：把 Kd 设成 0，轮距 10cm
//     的车（满转 344°/s）转 90° 会冲过头 61°；Kd=0.002 时只差 -1.2°。
//     原因是内环有约 150ms 滞后，车会「照着上一拍的指令」继续转 —— 转得越快，
//     这段时间冲过的角度越多。P 项只知道「还差多少度」，管不了「现在多快」，
//     只有 D 项能看见速度。所以别为了「少调一个参数」把 Kd 去掉。
//
//   ---- 三重保护（都会停车，不会失控）----
//     容差   误差进入 YAW_TOLERANCE_DEG → 正常到位
//     超时   超过 YAW_TIMEOUT_MS 还没到 → 放弃（比如轮子被卡、地面太滑）
//     卡死   输出不小但 yaw 连续一段时间几乎不动 → 判定卡住（典型是死区）
//   三个都有独立的上报文字，看串口就知道是「调参没调好」还是「硬件问题」。
//
//   ---- 整定顺序 ----
//     1. 先只调 Kp：Ki=Kd=0，发 0x03 帧转 90°。太慢/太肉 → 加大；来回过冲 → 减小。
//     2. 加 Kd 抑制过冲：快到目标时「刹不住」就加大 Kd。
//     3. 最后加 Ki 补死区：总是差几度停下不动 → 加大 Ki（一次加一点，Ki 最容易震）。
//     4. 每次改完用 0x2B 帧在线写进去试，试好了再改这里的默认值重新烧录。
// ============================================================================

// 调用周期，必须与 chassis 闭环周期一致（外环挂在内环节拍上跑）
static constexpr uint32_t YAW_CONTROL_PERIOD_US = 50000U;

// ---- PID 默认参数（未实测整定，先给保守值）----
// 单位说明：Kp 的量纲是「归一化油门 / 度」，不是常见的 PWM/度。
static constexpr float YAW_KP_DEFAULT = 0.008f;
static constexpr float YAW_KI_DEFAULT = 0.020f;
static constexpr float YAW_KD_DEFAULT = 0.0020f;

// 输出上限。留 20% 余量：一上来就满转既没必要，也让 D 项没有发挥空间
// （w 已经顶在 ±1 时，阻尼想拉也拉不回来）。
static constexpr float YAW_W_MAX = 0.8f;

// 到位容差。定得比「实际能停到的精度」略大一点：
// 陀螺零偏残余约 0.02°/s、转 90° 耗时约 1s，角度误差量级 0.1°，不是瓶颈；
// 真正的瓶颈是死区导致的「最后几度挪不动」。容差太小 → 永远到不了，只能等超时。
static constexpr float YAW_TOLERANCE_DEG = 2.5f;

// 积分限幅（抗积分饱和的第一道，第二道是 .cpp 里的条件积分）。
// 按「积分项最多贡献多少油门」反推：Ki·MAX = 0.02×5 = 0.1，即满量程 ±0.8 的 12%。
// 捅 PWM 死区只需要 0.02~0.04，所以 0.1 已经很宽裕了。
// （仿真对比过 I_MAX=30 和 5：过冲没有实质差别。取小的纯粹是「够用就好」，
//   不是因为它修了什么 bug —— 别把它当成超调的调节旋钮，调超调请动 Kp/Kd。）
static constexpr float YAW_INTEGRAL_MAX = 5.0f;

// 单次转向的目标角上限（相对角）。±720° = 两圈，够用了。
// 夹一下是为了防止串口手滑输入 90000 让车转 250 圈。
static constexpr float YAW_TARGET_MAX_DEG = 720.0f;

// ---- 保护阈值 ----
static constexpr uint32_t YAW_TIMEOUT_MS = 8000U;

// 卡死检测：每 YAW_STALL_WINDOW_MS 检查一次，这段时间内 yaw 变化小于
// YAW_STALL_MIN_DEG 而输出却大于 YAW_STALL_MIN_W → 判定卡住。
// 窗口要够长，否则「起步加速」的前半程会被误判成卡死。
static constexpr uint32_t YAW_STALL_WINDOW_MS = 600U;
static constexpr float YAW_STALL_MIN_DEG = 1.0f;
static constexpr float YAW_STALL_MIN_W = 0.05f;

// 转向环状态 / 结束原因
enum class YawResult : uint8_t {
  IDLE,     // 没在转（还没启动，或上一轮已结束）
  RUNNING,  // 转着呢
  ARRIVED,  // 到位（误差已进入容差）
  TIMEOUT,  // 超时没到
  STALLED,  // 卡住了（输出不小但角度不动）
  ABORTED,  // 被中止（姿态数据失效、或收到停车指令）
};

void yaw_control_init();

// 启动一次相对转向：target = 当前 yaw + delta_deg。
// 左转为正、右转为负 —— 「转 90°」就发 +90，和 attitude 的 yaw 正方向一致。
// 返回 false = 姿态还没就绪（IMU 还没攒够数据），这次没启动。
bool yaw_control_start(float delta_deg);

// 每 YAW_CONTROL_PERIOD_US 调一次（挂在底盘闭环节拍上）。
// 返回 RUNNING 表示还在转；返回 ARRIVED/TIMEOUT/STALLED/ABORTED 的那一拍
// 表示这一轮**刚刚结束**（车已经停了），之后就一直返回 IDLE。
YawResult yaw_control_update();

// 主动中止（停车指令、其他运动指令抢占时调用）
void yaw_control_abort();

// 是否有一轮转向正在进行
bool yaw_control_busy();

// ---- 调参 / 观察 ----
void yaw_control_set_pid(float kp, float ki, float kd);
void yaw_control_get_pid(float *kp, float *ki, float *kd);

float yaw_control_get_target_deg(); // 本轮的绝对目标角
float yaw_control_get_error_deg();  // 最近一次的误差（到位后就是最终残差）
float yaw_control_get_w();          // 最近一次的输出
uint32_t yaw_control_get_elapsed_ms();
