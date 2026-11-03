#ifndef __MODEL_H
#define __MODEL_H

#include "main.h"

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

#endif
