#ifndef __MODEL_H
#define __MODEL_H

#include "main.h"

/* ===== 底座偏航的物理标定（逻辑角 ↔ 舵机安装角）=====
 *
 * 逻辑偏航角 yaw（度）：0 = 朝正前方 +x，正 = 朝 +z 一侧；这是 IK/FK、
 * 纯偏航模式和上位机 3D 图共用的那一个角度。
 * 舵机安装角 waist（度）：真正写进 PWM 的那个 0~180。
 *
 *     waist = SERVO_ZERO_W + SERVO_DIR_W · yaw          （正算，下发用）
 *     yaw   = (waist − SERVO_ZERO_W) / SERVO_DIR_W      （反算，显示用）
 *
 * 为什么单独抽出来：底座舵机一圈 180° 只覆盖正前方半平面，它的「角度增大」
 * 对应实物往哪边转，纯粹由舵机安装方向 + 传动决定，和模型无关。
 * 若实测「发 Y +10，实物转到 3D 图的反方向」，就是这里差一个符号 ——
 * 把 SERVO_DIR_W 改成 -1 即可，不要去改 IK 的 z 语义。
 *
 * 改动影响面（四处必须同时用这一对常量，否则彼此打架）：
 *   model.c  solve()           笛卡尔 IK：x,z → waist
 *   model.c  model_fk()        固件正解：waist → x,z
 *   motion.c Motion_PolarApply()  纯偏航直驱：yaw → waist
 *   tools/serial_console.html  forwardKinematics() / parseStatus()
 *
 * 本项目实物实测：舵机角增大 → 实物往 −z 方向转，与模型约定相反，
 * 故 DIR_W = -1。注意这会让「同一组 (x,y,z) 里的 z 符号」在实物上也跟着
 * 翻过来 —— 这是修正，不是副作用：此前笛卡尔模式的 z 也是反的。 */
#define SERVO_ZERO_W    (90.0f)   /* 逻辑 0° 对应的安装角（中位） */
#define SERVO_DIR_W     (-1.0f)   /* 安装角增大时逻辑 yaw 增大(+1)还是减小(-1) */

/* 三维逆运动学解算并下发舵机（三连杆，放弃姿态约束、腕角自动取最优）
 * 参数：x 正前方水平距离 / y 离地高度 / z 横向偏移（单位：米）
 * 返回：0 解算成功；1 目标不可达（保持上次姿态）
 * 说明：夹爪姿态不是输入，而是在够到目标的前提下尽量接近水平。
 *       位置只有 2 个约束、关节有 3 个，多出的 1 个冗余用来挑姿态。 */
uint8_t model(float x, float y, float z);

/* 解算并【立即】下发（跳过插值）。供笛卡尔空间平滑运动使用：
 * 调用方按毫米级步长推进末端坐标，每步调一次，末端即走直线。 */
uint8_t model_now(float x, float y, float z);

/* 只判断可达性，不驱动舵机。按键改坐标前先试算，避免目标跑到空间外。 */
uint8_t model_reachable(float x, float y, float z);

/* 查询有效区间（2mm 分辨率扫描，只判断可达性、不驱动舵机），返回 0 = 有解 */
uint8_t model_XRange(float y, float z, float *x_min, float *x_max);
uint8_t model_YRange(float x, float z, float *y_min, float *y_max);

/* 正运动学：由 4 个安装角算出末端坐标（米），不驱动舵机。
 * 参数：a1 肩 / a2 肘 / a3 腕 / waist 底座偏航，均为舵机安装角（度）
 *
 * 存在的意义：示教回放是直接下发关节角的，绕过了逆解，
 * 于是 Motion 那边记着的坐标就和实物脱节了。回放结束后用它把坐标算回来，
 * 否则下一条 M 会从过期的起点插值，第一步会跳。
 *
 * 标定参数（SERVO_ZERO_A1 等与 SERVO_DIR_A1 等）与逆解共用同一份，
 * 所以「逆解 → 正解」闭环误差为 0 这条性质在这里依然成立。 */
void model_fk(uint16_t a1, uint16_t a2, uint16_t a3, uint16_t waist,
              float *x, float *y, float *z);

#endif
