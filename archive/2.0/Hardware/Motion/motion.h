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

/* 末端线速度（米/秒）。0.05 = 50mm/s。
 * 舵机本身跟不上太快，实测超过 80mm/s 关节会明显滞后。 */
#define MOTION_SPEED            0.05f

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

/* 设定目标坐标（米）。成功则末端开始按 MOTION_SPEED 直线推进。
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

#endif
