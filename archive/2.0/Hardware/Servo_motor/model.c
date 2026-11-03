#include "model.h"
#include "math.h"
#include <stdint.h>
#include "Servo_motor.h"

#define PI 3.1415926
#define D2R (PI / 180.0)
#define R2D (180.0 / PI)

/* 连杆长度唯一来源：Servo_motor.c 的 mech_length（单位：米，且 y 为离地高度）
 *   l0 底座立柱 / l1 肩→肘 / l2 肘→腕 / l3 腕→末端
 * 俯仰平面内是标准三连杆（3R）结构：肩、肘、腕三个转动关节。 */
extern length mech_length;

/* ===== 舵机标定参数（必须实测，否则逆解全错）=====
 * 几何零位的定义：
 *   肩 th1=0 → 大臂水平前伸
 *   肘 th2=0 → 小臂与大臂成一条直线（完全伸直）
 *   腕 th3=0 → 夹爪与小臂成一条直线
 *
 * SERVO_ZERO_x ：几何零位对应的舵机安装角（度）
 *                 实测：进 CALIB_MODE，把该关节调到上述姿态，读 OLED 角度填入
 * SERVO_DIR_x  ：舵机角增大时，几何角增大(+1)还是减小(-1)
 *
 * 当前值是「按实物照片反推」的初值，务必实测修正：
 *   依据：初始化 (90,90,135,135) 时实测大臂上斜约 55°、小臂相对大臂下弯约 48°、
 *         夹爪相对小臂下弯约 96°。已确认 a1 增大 → 大臂下压，故 DIR_A1 = -1。
 */
#define SERVO_ZERO_A1   (180.0)   /* 实测：a1=180 大臂向前水平，a1=0 向后水平 */
/* 肘：实测几何零位对应舵机角 180，但实物到位后比目标【高】约 2°
 * （小臂需要往下调 2° 才对得准）。所以把零位减到 178：
 *   a2 = 178 + th2_deg  →  比原来少给 2°，正好抵消偏高。
 *
 * 注意方向：这里减 2 而不是加 2。加 2 会让小臂更往上抬，与期望相反。
 * 若实测偏差不是 2°，改这个数字即可（偏高 1.5° → 178.5，偏高 2.5° → 177.5）。
 * 若某天发现补偿过头（变成偏低），往 180 方向回调。 */
#define SERVO_ZERO_A2   (178.0)   /* 实测 180 − 2° 偏高补偿 */
#define SERVO_ZERO_A3   (90.0)    /* 实测：a3=90 夹爪与小臂同轴 */

#define SERVO_DIR_A1    (-1)      /* 实测：a1 变大 → 大臂向后/下压 */
#define SERVO_DIR_A2    (+1)      /* 实测：a2 变小 → 小臂向后弯 */
#define SERVO_DIR_A3    (-1)      /* 实测：a3 变大 → 夹爪往里扣 */

/* 姿态角 α 的候选搜索步长（度）
 *   FINE   ：model() 精解，5° 一档
 *   COARSE ：区间扫描用，15° 一档（scan_axis 要跑 200 个点，用细步长会卡） */
#define ALPHA_STEP_FINE    5.0
#define ALPHA_STEP_COARSE  15.0

/* 上一次成功解算的安装角，目标不可达时用于回退不动作。
 * 初值与 Servo_motor_Init() 的上电中位姿态保持一致。 */
static uint16_t last_angle[3] = {90, 90, 90};

/**
 * 将舵机安装角夹到 [0, 180]（舵机角度范围 0~180°）
 */
static double clamp_servo(double a)
{
    if (a > 180.0) return 180.0;
    if (a <   0.0) return   0.0;
    return a;
}

/**
 * 函数：给定末端姿态角 α，解三连杆（肩 / 肘 / 腕）
 * 参数：r, h  末端在俯仰平面内的水平距离与相对肩部高度（米）
 * 参数：alpha 末端姿态角（弧度，0 = 夹爪水平，向上为正）
 * 输出：out_a1 / out_a2 / out_a3 = 肩 / 肘 / 腕 的舵机安装角（度）
 * 返回：0 该姿态可行；1 不可行（腕点几何不可达，或某个关节超舵机行程）
 *
 * 说明：三连杆有 3 个关节角，末端位姿是 (r, h, α) 三个量。
 *       一旦指定 α，就是 3 方程 3 未知，解唯一 —— 所以本函数不含搜索。
 */
static uint8_t solve_at_alpha(double r, double h, double alpha,
                              double *out_a1, double *out_a2, double *out_a3)
{
    const double A = mech_length.l1;   /* 肩 → 肘 */
    const double B = mech_length.l2;   /* 肘 → 腕 */
    const double C = mech_length.l3;   /* 腕 → 末端 */

    /* 1) 末端 = 腕点 + C·(cosα, sinα)，反推腕点 */
    double Wx = r - C * cos(alpha);
    double Wy = h - C * sin(alpha);
    double D  = hypot(Wx, Wy);

    /* 2) 腕点必须落在肩部 2R 环带 [|A-B|, A+B] 内 */
    if (D > (A + B) + 1e-6 || D < fabs(A - B) - 1e-6)
    {
        return 1;
    }

    /* 3) 前两段做 2R 逆解。余弦定理只能给出 |th2|，而肘上 / 肘下两种构型
     *    都能到达同一个点，物理上哪个可行取决于装配方向和舵机行程，
     *    所以两个都试一遍。（早期只取肘上，导致大量本可达的点被判 OUT。） */
    double cos_t2 = (D * D - A * A - B * B) / (2.0 * A * B);
    if (cos_t2 < -1.0) cos_t2 = -1.0;   /* 数值保护，避免 acos 越界 */
    if (cos_t2 >  1.0) cos_t2 =  1.0;

    double th2_mag = acos(cos_t2);

    for (uint8_t s = 0; s < 2; s++)
    {
        double th2 = (s == 0) ? th2_mag : -th2_mag;   /* 肘上 / 肘下 */
        double th1 = atan2(Wy, Wx) - atan2(B * sin(th2), A + B * cos_t2);
        double th3 = alpha - th1 - th2;               /* 姿态闭合：α = θ1+θ2+θ3 */

        /* 4) 弧度转角度，再用标定参数换算成舵机安装角
         *    a = ZERO + DIR × th_deg
         *    ZERO 是「几何零位」时的舵机角，DIR 是方向（+1 同向 / -1 反向）。
         *    这些值必须实测，见文件顶部 SERVO_ZERO_* / SERVO_DIR_* 的说明。
         *
         *    行程检查容差取 0.01°：既能吸收浮点误差，又不会让后面的 clamp_servo
         *    产生可观的静默截断。（早期用 0.5° 容差，a2=180.49° 会被判合格再钳到
         *    180°，0.49° 的截断在末端放大成约 2.2mm 的位置误差。） */
        double a1 = SERVO_ZERO_A1 + SERVO_DIR_A1 * (th1 * R2D);
        double a2 = SERVO_ZERO_A2 + SERVO_DIR_A2 * (th2 * R2D);
        double a3 = SERVO_ZERO_A3 + SERVO_DIR_A3 * (th3 * R2D);

        /* 肘的上限保持 180：ZERO_A2 已改为 178（往下补偿），
         * a2 最大出现在 th2 = 0（完全伸直），即 178，不会顶到 180。
         * 这里若放宽到 182，th2 ∈ [0, 4°] 的「肘上」构型会被误接受，
         * 和实物只走肘下的事实冲突，姿态会突然跳到反关节。 */
        if (a1 < -0.01 || a1 > 180.01 ||
            a2 < 44.99 || a2 > 180.01 ||
            a3 < -0.01 || a3 > 180.01)
        {
            continue;   /* 这个构型超行程，试另一个 */
        }

        *out_a1 = clamp_servo(a1);
        *out_a2 = clamp_servo(a2);
        *out_a3 = clamp_servo(a3);
        return 0;
    }

    return 1;
}

/**
 * 函数：逆解内核（只解算角度，不驱动舵机）
 * 参数：x     末端在水平面内「正前方」方向的水平距离（米）
 * 参数：y     末端离地高度（米，含底座立柱；内部自动扣除立柱高）
 * 参数：z     末端在水平面内的「横向」偏移（米，与 x 垂直）
 * 参数：step_deg 姿态角搜索步长（度），FINE 精解 / COARSE 快扫
 * 输出：out_waist / out_a1 / out_a2 / out_a3 = 底座/肩/肘/腕 安装角（度）
 * 返回：0 解算成功；1 目标超出可达范围
 *
 * 说明：这是「放弃姿态控制」的三连杆解法 —— 放弃的是对 α 的**约束**，
 *       不是放弃腕关节。末端位置只有 (r, h) 两个约束，而关节有 3 个，
 *       于是多出 1 个冗余自由度。这里用这个冗余去挑「|α| 最小」的解：
 *       从 α = 0（夹爪水平）出发向两侧逐步放宽，第一个可行姿态即为答案。
 *       结果：姿态能水平就水平，够不到才倾斜，工作空间比锁死腕大得多。
 */
static uint8_t solve(float x, float y, float z, double step_deg,
                     double *out_waist, double *out_a1,
                     double *out_a2, double *out_a3)
{
    /* 1) 底座偏航角：横向 z 与前向 x 共同决定，再映射到 0~180 的安装角 */
    double yaw_rad   = atan2((double)z, (double)x);
    double waist_raw = yaw_rad * R2D + 90.0;

    /* 底座舵机行程 0~180° 只覆盖 x >= 0 的半平面（正前方左右各 90°）。
     * x < 0（目标在背后）时 atan2 给出的角度会超出 ±90°，waist_raw 落到
     * [0,180] 之外。早期这里只 clamp 不检查：clamp 把角度静默压到 0 或 180，
     * 而俯仰解算用的是 r = hypot(x,z)（恒为正），照样成功 → 被判「可达」，
     * 实际末端甩到正侧面，串口还回 OK。必须在这里直接拒掉。
     * 容差 0.01° 与 a1/a2/a3 的行程检查保持一致。 */
    if ((waist_raw < -0.01) || (waist_raw > 180.01))
    {
        *out_waist = clamp_servo(waist_raw);
        return 1;
    }

    double waist = waist_raw;

    /* 2) 投影到俯仰平面
     *    r = hypot(x, z) —— 水平面内的真实半径，必须由两个水平分量合成。
     *    h = y - l0      —— 相对肩关节的高度。
     *    注意：早期版本此处写成 r = x，z≠0 时末端会整体偏移（最大 92.5mm）。 */
    double r = hypot((double)x, (double)z);
    double h = (double)y - mech_length.l0;

    /* 3) 从 α = 0 向两侧找第一个可行姿态（冗余自由度的用法） */
    for (double d = 0.0; d <= 90.0 + 1e-6; d += step_deg)
    {
        if (solve_at_alpha(r, h, d * D2R, out_a1, out_a2, out_a3) == 0)
        {
            *out_waist = waist;
            return 0;
        }
        if (d > 0.0 &&
            solve_at_alpha(r, h, -d * D2R, out_a1, out_a2, out_a3) == 0)
        {
            *out_waist = waist;
            return 0;
        }
    }

    *out_waist = waist;
    return 1;
}

/**
 * 函数：只判断目标是否可达，不驱动舵机
 * 返回：0 可达；1 不可达
 * 用途：按键改变目标坐标前先试算，避免目标跑到工作空间外之后
 *       要按很多次才能拉回来。
 */
uint8_t model_reachable(float x, float y, float z)
{
    double waist, a1, a2, a3;
    return solve(x, y, z, ALPHA_STEP_FINE, &waist, &a1, &a2, &a3);
}

/**
 * 函数：解算并【立即】下发舵机（跳过 Servo_Motor_Updata 的插值）
 * 返回：0 解算成功；1 目标不可达（舵机保持不动）
 *
 * 说明：这是给「笛卡尔空间平滑运动」用的。
 *       主循环把末端坐标按毫米级步长推进，每一步都调用本函数重算 IK，
 *       于是末端走的是直线，而不是三个关节各自插值拼出来的弧线。
 *       因为调用方已经保证了步长足够小，这里不需要再做关节插值。
 */
uint8_t model_now(float x, float y, float z)
{
    double waist, a1, a2, a3;

    if (solve(x, y, z, ALPHA_STEP_FINE, &waist, &a1, &a2, &a3) != 0)
    {
        return 1;
    }

    Servo_Motor_SetAngleNow(1, (uint16_t)waist);   /* 底座偏航 */
    Servo_Motor_SetAngleNow(2, (uint16_t)a1);      /* 肩 */
    Servo_Motor_SetAngleNow(3, (uint16_t)a2);      /* 肘 */
    Servo_Motor_SetAngleNow(4, (uint16_t)a3);      /* 腕 */

    last_angle[0] = (uint16_t)a1;
    last_angle[1] = (uint16_t)a2;
    last_angle[2] = (uint16_t)a3;

    return 0;
}

/**
 * 函数：机械臂三维逆运动学解算并下发舵机（三连杆，腕角自动取最优）
 * 返回：0 解算成功；1 目标不可达（肩/肘/腕保持上次姿态不动）
 * 说明：夹爪姿态不再是输入，而是在保证够到目标的前提下尽量接近水平。
 */
uint8_t model(float x, float y, float z)
{
    double waist, a1, a2, a3;

    if (solve(x, y, z, ALPHA_STEP_FINE, &waist, &a1, &a2, &a3) != 0)
    {
        Servo_Motor_SetAngle(2, last_angle[0]);
        Servo_Motor_SetAngle(3, last_angle[1]);
        Servo_Motor_SetAngle(4, last_angle[2]);
        return 1;
    }

    Servo_Motor_SetAngle(1, (uint16_t)waist);     /* 底座偏航 */
    Servo_Motor_SetAngle(2, (uint16_t)a1);        /* 肩 */
    Servo_Motor_SetAngle(3, (uint16_t)a2);        /* 肘 */
    Servo_Motor_SetAngle(4, (uint16_t)a3);        /* 腕 */

    last_angle[0] = (uint16_t)a1;
    last_angle[1] = (uint16_t)a2;
    last_angle[2] = (uint16_t)a3;

    return 0;
}

/**
 * 函数：固定一个轴，扫描另一个轴上的有效区间（单位：米，供调试显示用）
 * 参数：axis = 0 时 fixed 是 y，扫描 x 区间；axis = 1 时 fixed 是 x，扫描 y 区间
 * 说明：y 从地面 0 开始扫，负高度不计入有效范围
 * 返回：0 有解；1 该条件下不存在任何有效值
 */
static uint8_t scan_axis(uint8_t axis, float fixed, float z,
                         float *out_lo, float *out_hi)
{
    /* 5mm 分辨率：引入姿态搜索后每个点最坏要算 13 次逆解，
     * 用 2mm 会让整轮扫描过慢（阻塞主循环），5mm 对显示足够。 */
    const float STEP = 0.005f;
    double waist, a1, a2, a3;
    float t, first = 0.0f, last = 0.0f;
    uint8_t found = 0;

    for (t = 0.0f; t <= 0.40f; t += STEP)
    {
        float px = (axis == 0) ? t : fixed;
        float py = (axis == 0) ? fixed : t;

        if (solve(px, py, z, ALPHA_STEP_COARSE,
                  &waist, &a1, &a2, &a3) == 0)
        {
            if (found == 0) { first = t; found = 1; }
            last = t;
        }
    }

    *out_lo = first;
    *out_hi = last;
    return (found == 0) ? 1 : 0;
}

/* 查询当前 y 下 x 的有效区间（返回 0 = 有解） */
uint8_t model_XRange(float y, float z, float *x_min, float *x_max)
{
    return scan_axis(0, y, z, x_min, x_max);
}

/* 查询当前 x 下 y 的有效区间（返回 0 = 有解） */
uint8_t model_YRange(float x, float z, float *y_min, float *y_max)
{
    return scan_axis(1, x, z, y_min, y_max);
}
