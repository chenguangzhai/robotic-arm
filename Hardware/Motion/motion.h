/**
  * 末端运动控制：按【时间】推进，而不是按主循环圈数
  *
  * 为什么单独一个模块：
  *   时间推进需要跨轮次保存「上次 Tick 的毫秒数」「累计待走距离」「是否在动」，
  *   这些状态既不属于协议层（那是解析命令的），也不适合堆在 main.c 的循环体里。
  *   抽出来之后，按键和串口两条输入通道都只调 Motion_SetTarget()，
  *   运动逻辑只有一份。
  *
 * 与旧做法的区别（旧：主循环每转一圈走固定 1mm）：
 *   主循环周期并不固定——OLED_Update() 一次十几毫秒，区间扫描几十毫秒。
 *   按圈数走 → 速度随刷新节奏抖动，整体还被拖慢。
 *   按时间走 → 速度恒定，与主循环快慢无关；主循环偶尔卡住时
 *   用累加器补齐，并用 MOTION_DT_MAX_MS 限制单次补的量，避免一下跳很远。
 *
 * 3.0 又往前走了一步：速度不再是常数，而是梯形剖面（见 Motion_Tick 的注释）。
 *   - 起步受 MOTION_ACCEL 限制，不会从 0 直接跳满速
 *   - 快到位时按 sqrt(2*a*剩余距离) 主动减速，不会撞到目标再弹回
 *   - 巡航速度可运行时用 V 命令改，不必改代码重烧
 *
 * 调用顺序（每个主循环轮次）：
 *   Protocol_Poll();            // 串口命令 → 改目标
 *   ...按键 → Motion_SetTarget()
 *   Motion_Tick();              // 按时间推进一次
 */
#ifndef __MOTION_H
#define __MOTION_H

#include "main.h"
#include <stdint.h>

/* ===== 速度与精度 ===== */

/* 上电默认末端线速度（米/秒）。0.05 = 50mm/s。
 * 舵机本身跟不上太快，实测超过 80mm/s 关节会明显滞后。
 *
 * 注意：这只是【默认值】，运行中已被 Motion_SetSpeed() 改成可调状态量。
 * 烧录后想改速度不必重新编译，串口发 V <mm/s> 即可（见 protocol.h）。 */
#define MOTION_SPEED_DEFAULT    0.05f

/* 可调速度的允许区间（毫米/秒）。
 * 下限 20：再慢主循环一圈的位移小于 STEP_MIN，动作会一顿一顿。
 * 上限 150：留出试探余量，但超过 80 关节就会明显滞后，别当成正常值用。 */
#define MOTION_SPEED_MM_MIN     20u
#define MOTION_SPEED_MM_MAX     150u

/* 末端加速度（米/秒²）。
 * 这是梯形速度剖面的斜率，决定了「起步多猛、刹车多急」。
 * 0.5 m/s² = 0.1 秒从静止加到 50mm/s，加速段走 2.5mm。
 *
 * 为什么需要它：原来 s_accum 是纯匀速，速度从 0 直接跳到 50mm/s，
 * 启动瞬间冲击电流最大，到点又是直接停 —— 结构回弹，
 * 夹着东西转大角度会把工件甩出去（文档 10.1）。
 * 有了斜率，冲击电流是「逐渐加上去」的，停止也是「逐渐减下来」的。
 *
 * 调小的代价是每段动作变慢；调大则回到硬起硬停。 */
#define MOTION_ACCEL            0.5f

/* 单次推进的最小距离（米）。累加器攒够这么多才真正走一步并重算逆解，
 * 避免主循环很快时（<5ms 一轮）为 0.05mm 反复解逆解白烧 CPU。 */
#define MOTION_STEP_MIN         0.0005f     /* 0.5 mm */

/* 单次 Tick 最多补多少毫秒。主循环被区间扫描卡住 50ms 以上时，
 * 用它把「欠账」截断，否则会一步跳出好几毫米。 */
#define MOTION_DT_MAX_MS        50u

/* 到位判定容差（米）。小于它就认为到位，不再微调。 */
#define MOTION_ARRIVE_EPS       0.0005f     /* 0.5 mm */

/* 坐标绝对值上限（米）。挡住单位笔误：把 280(mm) 当米发会被拒，
 * 而不是解出一个 OUT 或者跑出个诡异姿态。 */
#define MOTION_COORD_LIMIT      1.0f

/* ===== 路径校验 ===== */

/* 1 = 受理目标前沿直线采样，中途穿出可达区就整条拒绝（回 ERR:PATH）
 * 0 = 只校验终点，中途不可达时末端会卡一下再继续
 *
 * 采样步长越小越严，代价是受理一次命令要多解 dist/步长 遍逆解。
 * 实测单次逆解约 30us，5mm 步长下 30cm 的移动约 60 次 ≈ 2ms，可接受。 */
#define MOTION_PATH_CHECK       1
#define MOTION_PATH_STEP        0.005f      /* 5 mm */

/* 路径采样点数上限。防止一次跨度很大时算太久（超过就按上限等分）。 */
#define MOTION_PATH_MAX_SAMPLES 128

/* ===== 返回值 ===== */
#define MOTION_OK               0
#define MOTION_ERR_OUT          1   /* 目标点本身不可达 */
#define MOTION_ERR_PATH         2   /* 目标可达，但直线路径中间穿出可达区 */
#define MOTION_ERR_PARAM        3   /* 坐标超范围（多半是单位笔误） */

/* 初始化：把目标与当前位置都设到初始点 (0.28, 0.10, 0)，并记录起始时刻。
 * 必须在 Servo_motor_Init() 之后、进入主循环之前调用。 */
void Motion_Init(void);

/* 设定目标坐标（米）。成功则末端按当前巡航速度沿直线推进（起步有加速段）。
 * 返回 MOTION_OK 表示已受理，不代表已到位——到位由 Motion_IsBusy() 变 0 表示。 */
uint8_t Motion_SetTarget(float x, float y, float z);

/* 只做路径可行性校验，不改目标、不驱动舵机。 */
uint8_t Motion_CheckPath(float x, float y, float z);

/* 按时间推进一次。每个主循环轮次调用一次，没有移动时开销极小。 */
void Motion_Tick(void);

/* 是否还在运动中。0 = 已到位。 */
uint8_t Motion_IsBusy(void);

/* 查询目标坐标 / 当前实际坐标（米）。 */
void Motion_GetTarget(float *x, float *y, float *z);
void Motion_GetCurrent(float *x, float *y, float *z);

/* 上一次 Motion_SetTarget 的结果（MOTION_OK / ERR_OUT / ERR_PATH / ERR_PARAM）。
 * 供 OLED 显示用：按键被拒时能看出是被终点拒了还是被路径拒了。 */
uint8_t Motion_GetLastError(void);

/* ===== 速度在线可调 =====
 *
 * 为什么做成状态量而不是宏：MOTION_SPEED 原来的注释是「照 1mm/圈 × 20Hz 估的，
 * 未实测」。未实测的参数写成宏，等于把「调它」变成「改代码 + 重烧」，
 * 结果就是谁都不去调。改成运行时可设，才能一边跑一边试。
 *
 * 改的是【上限】，不是当前速度：正在走的那一段会按新上限重新规划，
 * 已经走到的位置不受影响，所以随时改都安全。
 *
 * 返回 0 成功，1 超出允许区间（不改变原值）。
 * 单位统一用整数 mm/s，避开 %f。 */
uint8_t  Motion_SetSpeed(float mm_s);
uint16_t Motion_GetSpeedMmS(void);

/* 当前实际速度（mm/s）。回放/减速段里会小于设定值，
 * 可用来观察梯形剖面是不是真的生效了。 */
uint16_t Motion_GetCurrentSpeedMmS(void);

/* ===== 纯偏航 / 柱坐标模式（3.1 新增）=====
 *
 * 为什么需要它：M 改 z 时末端是沿【直线】横移的，到基座的距离 r 随之变大
 * （r = hypot(x, z)），肩/肘/腕必须跟着补偿 —— 用户看到的就是
 * 「只想转个方向，结果四个关节一起动」。那不是 bug，是笛卡尔坐标的几何必然。
 *
 * 但如果目的是「把手臂转到另一个朝向」，这件事在柱坐标下就是纯偏航：
 * r、h 恒定，只有底座在动。此时俯仰三关节的逆解输入根本没变，
 * 所以这里【刻意不走 model_now()】—— 它会重解一遍三关节，
 * 虽然结果相同，但 (uint16_t) 截断在浮点尾数压线时仍可能翻 1 位，
 * 而纯偏航的定义就是「一个舵机都不许动」。直接写底座 PWM 才干净。
 *
 * 与 M 的关系：Y 只改朝向，M 只改位置，两者互不替代。
 * 任何一条 M 都会把机械臂切回笛卡尔模式，起点取当前所在的弧上点；
 * 反过来说，Y 也可以在中途接手一条还没走完的 M，从当前位置开始转。
 */
#define MOTION_YAW_SPEED       60.0f   /* 巡航角速度（度/秒） */
#define MOTION_YAW_ACCEL      120.0f   /* 角加速度（度/秒²） */
#define MOTION_YAW_STEP_MIN     0.25f  /* 角累加器阈值：攒够才发一次 PWM */
#define MOTION_YAW_EPS          0.25f  /* 到位容差（度），配合 1° 舵机分辨率 */
#define MOTION_YAW_LIMIT       88.0f   /* 偏航角上限，留 2° 免得机械限位顶死 */
#define MOTION_YAW_R_MIN        0.05f  /* 半径小于它朝向无意义，直接拒 */

/* 设定偏航角（度，正 = 朝 z 正方向 / 面向机械臂的左侧）。
 * 首次调用会锁定当前半径与高度；之后连续调用只改角度，r/h 保持不变。
 * 返回 MOTION_OK 已受理，其余见上面的错误码。
 * 角度超 ±MOTION_YAW_LIMIT 回 ERR_PARAM；该朝向下工作空间够不到回 ERR_OUT。 */
uint8_t Motion_SetYaw(float deg);

/* 当前偏航角（度）。偏航模式下取推进中的角，否则由当前坐标反算。 */
float   Motion_GetYaw(void);

/* 是否处于纯偏航模式。OLED/上位机想区分「在转」和「在走」时用。 */
uint8_t Motion_IsPolar(void);

/* 用【当前实际舵机角】正解出末端坐标，并把目标与当前位置都重设到该点。
 *
 * 什么时候必须调：只要有人绕过了 Motion 直接动舵机，Motion 的 s_cx/s_cy/s_cz
 * 就和实物脱节了 —— 目前有两个入口：示教回放（play.c 直接下发 6 路角度）
 * 和标定模式。不同步的后果是：下一条 M 会从过期的起点算直线和路径校验，
 * 第一步把机械臂猛地拽到那条错误的直线上。
 *
 * 由 play.c 在回放结束 / 中途停止时调用，不需要业务层操心。 */
void Motion_SyncFromCurrent(void);

#endif
