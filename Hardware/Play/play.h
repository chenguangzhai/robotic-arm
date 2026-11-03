#ifndef __PLAY_H
#define __PLAY_H

#include "main.h"
#include <stdint.h>

/* ===== 示教 / 回放 =====
 *
 * 设计要点（和 2.0 的笛卡尔运动最大的不同）：
 *   记录的是【6 个舵机的安装角】，不是末端坐标。
 *   回放时把这串角度原样下发，中间不做逆解 —— 所以：
 *     - 不受 66.3% 可达率限制
 *     - 不会被 MOTION_PATH_CHECK 判 ERR:PATH（关节空间插值恒连续、恒可达）
 *     - 精度只取决于舵机重复性，而不是「舵机角→物理角」的标定误差
 *
 *   为什么不在协议层直接调 Servo_Motor_SetAngle 交给 TIM4 慢慢插值？
 *   因为那里是 1°/50ms（20°/s），转 90° 要 4.5 秒，回放会慢到不可用。
 *   这里自己按 PLAY_MS_PER_DEG 算每段的时长，主循环每圈 SetAngleNow 一次，
 *   速度可控、也顺便把 OldAngle 和 NewAngle 对齐，避免两套插值打架。
 */

#define PLAY_MAX_POINTS         64u     /* 点数上限：64 × 14B = 896B RAM */
#define PLAY_DEFAULT_DWELL_MS   500u    /* T 不带参数时的停留时间 */
#define PLAY_MIN_SEG_MS         150u    /* 单段最短时长，太小会像抽搐 */
#define PLAY_MS_PER_DEG         8u      /* 每度用多少毫秒走完：8ms/° ≈ 125°/s */
#define PLAY_MAX_SEG_MS         2000u   /* 单段最长时长，防止慢到让人以为卡死 */

/* 梯形速度剖面的加速段占比：前 25% 时间加速、后 25% 减速、中间匀速。
 *
 * 为什么需要：原来 Play_Tick 是线性插值，每段都是「瞬间到速、瞬间停」。
 * 只放一个点看不出问题，但回放是连着几十个点跑，每段都硬起硬停，
 * 机械臂会像在抽搐，而且每次起停的冲击会叠加到结构上。
 *
 * 代价：峰值角速度是平均值的 1/(1-r) 倍 —— 0.25 时是 1.33 倍。
 * 所以【不要】为了「看起来变快了」再把 PLAY_MS_PER_DEG 减小，
 * 否则峰值反而超过原来那版匀速直冲的速度。 */
#define PLAY_ACCEL_RATIO        0.25f

/* 事件：协议层每轮取一次，用来往外推 PLAY i/n 和 PLAY END */
#define PLAY_EV_NONE    0u
#define PLAY_EV_STEP    1u      /* 开始走第 index 个点 */
#define PLAY_EV_END     2u      /* 一轮结束（单次跑完或次数用尽） */

/* 初始化：从 Flash 载入点表。只填数组，不动舵机 ——
 * 上电绝不能因为「上次存了 20 个点」就把机械臂弹到某个姿态上去。 */
void    Play_Init(void);

/* 记录当前姿态。取 OldAngle（实际位置）而不是 NewAngle（目标），
 * 这样在运动中记录也不会记到一个还没走到的位置。返回记录后的点数，
 * 满了则返回当前点数（调用方用 Play_Count 对比判断是否成功）。 */
uint8_t Play_Record(uint16_t dwell_ms);

uint8_t Play_Count(void);

/* 追加一个【指定角度】的点，不读当前姿态。返回追加后的点数，满了返回原点数。
 *
 * 和 Play_Record 的区别只有一点：角度是外面给的。
 * 为什么要它：回放是单片机执行的，点必须在 RAM 里；而 T 只能"记当前姿态"，
 * 意味着上位机缓存的点表没法推回来 —— 只能把手臂摆到每个姿态再按一次记录。
 * 有了 Play_Append，网页可以把 64 个点一次性写回，断联/断电后不必重新示教。
 *
 * 角度合法性由调用方（协议层）负责，这里只做点数上限判断。 */
uint8_t Play_Append(const uint16_t angles[6], uint16_t dwell_ms);

/* 读第 index 个点（0 基）。返回 0 成功，1 越界。 */
uint8_t Play_GetPoint(uint8_t index, uint16_t angles[6], uint16_t *dwell_ms);

/* 删除第 index 个点（0 基），后面的点前移。返回 0 成功，1 越界。 */
uint8_t Play_Delete(uint8_t index);

void    Play_Clear(void);

/* 开始回放。rounds = 0 表示无限循环。返回 0 受理，1 没有点可放。 */
uint8_t Play_Start(uint8_t rounds);

void    Play_Stop(void);

uint8_t Play_IsPlaying(void);

/* 正在执行第几个点（1 基，给人看）。未回放时返回 0。 */
uint8_t Play_Index(void);

/* 主循环每圈调一次。非回放状态下开销只有一次判断。 */
void    Play_Tick(void);

/* 取事件。返回 PLAY_EV_*，并填 index（1 基）与 total。 */
uint8_t Play_PollEvent(uint8_t *index, uint8_t *total);

/* 存 Flash。返回 0 成功，1 失败（擦写错误，或代码区已经吃掉了那一页）。 */
uint8_t Play_Save(void);

#endif
