#include "protocol.h"
#include "uart_app.h"
#include "motion.h"
#include "Servo_motor.h"
#include "play.h"
#include <stdarg.h>
#include <stdio.h>

/* ===== 行缓冲状态 ===== */
static uint8_t  s_line[PROTO_LINE_MAX];   /* 当前正在拼的这一行（不含结尾 \0） */
static uint16_t s_len;                    /* 已拼了多少字节 */
static uint8_t  s_discard;                /* 1 = 本行已超长，丢弃到下一个 '\n' 为止 */

/* ===== 到位回执状态 =====
 * 只在「串口受理过一条 M」之后才置 1，所以按键遥控时不会冒出 DONE。
 * 判据用 Motion_IsBusy() 由 1 变 0，而不是定时器猜——到位是运动模块
 * 自己算出来的事实，协议层不该重复实现一遍。 */
static uint8_t  s_wait_done;

/* ===== 统计（调试用，可在调试器里看）===== */
static uint32_t s_rx_lines;               /* 收到的完整行数 */
static uint32_t s_err_count;              /* 回过的错误数 */

/* ===== 夹爪角度 =====
 * 实物接线（2026-11-02 用户确认）：
 *   PA6 / 舵机5 = 旋转爪子（绕末端轴转，改变夹持朝向）
 *   PA7 / 舵机6 = 开合收紧
 *
 * 本文件只控制 PA7 的开合。PA6 固定 0°（夹爪平行于地面），不在这里动。
 *
 * 开合方向已实测确认（2026-11-02）：G 1 → 60° = 收紧，G 0 → 0° = 张开。
 *
 * 闭合角别给 180 —— 手指顶住工件后继续加力会堵转发热，烧舵机。 */
#define GRIP_ANGLE_OPEN     0u
#define GRIP_ANGLE_CLOSE    60u

/**
  * 函    数：跳过参数之间的分隔符
  * 参    数：pp 指向指针的指针，会就地前移
  * 返 回 值：无
  * 注    意：空格、Tab、逗号都当分隔符，所以 "M 0.3,0.1,0" 和
  *           "M 0.3 0.1 0" 等价。上位机用哪个都行。
  */
static void Proto_SkipSep(const char **pp)
{
    const char *p = *pp;

    while ((*p == ' ') || (*p == '\t') || (*p == ','))
    {
        p++;
    }
    *pp = p;
}

/**
  * 函    数：从字符串里解析一个十进制浮点数
  * 参    数：pp  指向指针的指针，成功后就地移到数字之后
  * 参    数：out 解析结果
  * 返 回 值：1 成功，0 没解析到数字
  * 注    意：不用 strtof / atof。newlib-nano 里那套浮点解析会额外拉进
  *           2~3KB 代码，而这里只需要 "[-]ddd.ddd" 这一种形式，
  *           手写反而更小、也不会受 locale 影响。
  *           最多吃 6 位小数——1e-6 米 = 1 微米，再细没有物理意义。
  */
static uint8_t Proto_ParseFloat(const char **pp, float *out)
{
    const char *p = *pp;
    uint32_t    ip = 0;         /* 整数部分 */
    uint32_t    fp = 0;         /* 小数部分 */
    float       div = 1.0f;     /* 小数部分的除数 */
    uint8_t     neg = 0;
    uint8_t     digits = 0;

    Proto_SkipSep(&p);

    if (*p == '-')
    {
        neg = 1;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    while ((*p >= '0') && (*p <= '9'))
    {
        if (ip < 1000000u)                  /* 防 uint32 溢出，超出部分直接丢 */
        {
            ip = ip * 10u + (uint32_t)(*p - '0');
        }
        p++;
        digits++;
    }

    if (*p == '.')
    {
        p++;
        while ((*p >= '0') && (*p <= '9'))
        {
            if (div < 1000000.0f)
            {
                fp = fp * 10u + (uint32_t)(*p - '0');
                div *= 10.0f;
            }
            p++;
            digits++;
        }
    }

    if (digits == 0)                        /* "-"、"."、"-." 这种只有符号没有数字 */
    {
        return 0;
    }

    *out = (float)ip + (float)fp / div;
    if (neg)
    {
        *out = -*out;
    }
    *pp = p;
    return 1;
}

/**
  * 函    数：米 → 整数毫米（四舍五入）
  * 返 回 值：毫米值，负数也正确
  * 注    意：'?' 的应答里坐标一律用整数毫米，避开 printf 的 %f。
  */
static int Proto_Mm(float v)
{
    float mm = v * 1000.0f;

    return (int)((mm >= 0.0f) ? (mm + 0.5f) : (mm - 0.5f));
}

/**
  * 函    数：'M x y z' —— 设定末端目标坐标
  * 参    数：p 指向 'M' 之后的第一个字符
  * 返 回 值：无
  * 注    意：这里只交给 Motion 受理，不直接驱动舵机。
  *           OK 的含义是「已受理」，运动在后台按当前巡航速度直线推进，
  *           走完由 Protocol_Poll() 推一行 DONE。
  */
static void Proto_CmdMove(const char *p)
{
    float nx;
    float ny;
    float nz;

    /* 回放期间不接受笛卡尔目标。
     * 两个模块都在动舵机，同时生效的话末端会被两条控制律反复拉扯，
     * 最终停在哪儿取决于谁后写，属于"看着像能跑、其实不可复现"的那类 bug。 */
    if (Play_IsPlaying())
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    if ((Proto_ParseFloat(&p, &nx) == 0) ||
        (Proto_ParseFloat(&p, &ny) == 0) ||
        (Proto_ParseFloat(&p, &nz) == 0))
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    /* 三个参数后面还有东西，说明多给了参数（比如误写成 M x y z 姿态角） */
    Proto_SkipSep(&p);
    if (*p != '\0')
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    /* 校验全在 Motion_SetTarget 里：范围 → 终点可达 → 路径可达。
     * 这里只负责把结果翻成文本错误码。 */
    switch (Motion_SetTarget(nx, ny, nz))
    {
        case MOTION_OK:
            s_wait_done = 1;                /* 等运动结束推 DONE */
            Protocol_Reply("OK");
            break;

        case MOTION_ERR_PARAM:
            s_err_count++;
            Protocol_Reply("ERR:RANGE");
            break;

        case MOTION_ERR_PATH:
            s_err_count++;
            Protocol_Reply("ERR:PATH");
            break;

        case MOTION_ERR_OUT:
        default:
            s_err_count++;
            Protocol_Reply("ERR:OUT");
            break;
    }
}

/**
  * 函    数：'Y <角度>' —— 纯偏航：只把底座转到指定朝向
  * 参    数：p 指向 'Y' 之后的第一个字符
  * 返 回 值：无
  * 注    意：角度单位是【度】，不是米 —— 这是全工程唯一一处非米制的输入，
  *           因为它描述的是「朝哪个方向」，不是「离多远」。
  *
  *           和 'M x y z' 的分工：
  *             M 是「末端到空间里的哪个点」，末端沿直线走，z 改变时
  *               半径 hypot(x,z) 也变，肩/肘/腕必须跟着补偿 —— 几何必然。
  *             Y 是「手臂转到哪个朝向」，r、h 锁死，只有底座动。
  *
  *           所以「调一个轴结果四个关节一起动」的正解是用 Y 改朝向，
  *           而不是把 M 的 z 改成角度：那会让 z 与 x/y 量纲不同，
  *           Motion 里所有距离/速度的式子全失效。
  */
static void Proto_CmdYaw(const char *p)
{
    float deg;

    if (Play_IsPlaying())
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    if (Proto_ParseFloat(&p, &deg) == 0)
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    Proto_SkipSep(&p);
    if (*p != '\0')
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    switch (Motion_SetYaw(deg))
    {
        case MOTION_OK:
            s_wait_done = 1;                /* 转完推一行 DONE */
            Protocol_Reply("OK");
            break;

        case MOTION_ERR_PARAM:
            s_err_count++;
            Protocol_Reply("ERR:RANGE");
            break;

        case MOTION_ERR_OUT:
        default:
            s_err_count++;
            Protocol_Reply("ERR:OUT");
            break;
    }
}

/**
  * 函    数：'G 0|1' —— 夹爪开合
  * 参    数：p 指向 'G' 之后的第一个字符
  * 返 回 值：无
  * 注    意：0 = 张开（0°），1 = 收紧（GRIP_ANGLE_CLOSE，默认 60°）。
  *           两个手指（舵机 5/6）同时动作。
  *
  *           这里用 SetAngleNow 立即到位，而不是 SetAngle 走插值：
  *           插值由 TIM4 的 1°/50ms 节奏驱动，转 60° 要 3 秒，夹爪等不起。
  *           夹爪行程短、负载小，直接到位没有明显冲击。
  *
  *           回 OK 表示已经下发，不是「夹到位」——夹爪没有位置反馈，
  *           夹没夹住工件只能靠外部确认。
  */
static void Proto_CmdGrip(const char *p)
{
    float v;

    if (Proto_ParseFloat(&p, &v) == 0)
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    /* 参数后面还有东西，说明多给了（比如 G 1 90） */
    Proto_SkipSep(&p);
    if (*p != '\0')
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    /* 只接受 0 和 1。用 != 比较是安全的：0.0f / 1.0f 都能被浮点精确表示，
     * 且 Proto_ParseFloat 对 "1" 和 "1.0" 解析结果完全相同。 */
    if ((v != 0.0f) && (v != 1.0f))
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    {
        uint16_t a = (v == 1.0f) ? (uint16_t)GRIP_ANGLE_CLOSE
                                 : (uint16_t)GRIP_ANGLE_OPEN;

        /* 只驱动 PA7（舵机 6 = 开合收紧）。
         * PA6（舵机 5）是旋转爪子，固定在中位，这里不碰它——
         * 两个功能完全不同，早期版本把两者设成同一角度是错的。 */
        Servo_Motor_SetAngleNow(6, a);
    }

    Protocol_Reply("OK");
}

/**
  * 函    数：'R <角度>' —— 夹爪旋转（PA6 / 舵机 5）
  * 参    数：p 指向 'R' 之后的第一个字符
  * 返 回 值：无
  * 注    意：角度 0~180，单位度。
  *           0°   = 夹爪平行于地面（上电初始姿态，Servo_motor_Init 里设的）
  *           90°  = 夹爪垂直于地面
  *           超过 180 或负数回 ERR:RANGE。
  *
  *           用 SetAngleNow 立即到位：走插值是 1°/50ms，转 90° 要 4.5 秒，
  *           太慢。旋转爪子行程内负载小，直接到位可以接受。
  *           但如果夹着东西还转大角度，惯性可能把工件甩出去 —— 使用上注意。
  *
  *           角度做四舍五入而不是截断：45.9 应该给 46 而不是 45。
  */
static void Proto_CmdRotate(const char *p)
{
    float a;

    if (Proto_ParseFloat(&p, &a) == 0)
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    Proto_SkipSep(&p);
    if (*p != '\0')
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    if ((a < 0.0f) || (a > 180.0f))
    {
        s_err_count++;
        Protocol_Reply("ERR:RANGE");
        return;
    }

    Servo_Motor_SetAngleNow(5, (uint16_t)(a + 0.5f));
    Protocol_Reply("OK");
}

/**
  * 函    数：'V <mm/s>' —— 设置末端巡航速度（'S' 是别名）
  * 参    数：p 指向命令字之后的第一个字符
  * 返 回 值：无（回 "OK <mm/s>"）
  * 注    意：单位是【毫米/秒】而不是米/秒，因为人调速度时说的是
  *           「50 还是 70」，不是「0.05 还是 0.07」。协议层换算，
  *           motion 内部仍用米/秒。
  *
  *           允许区间由 motion.h 的 MOTION_SPEED_MM_MIN/MAX 定义。
  *           越界回 ERR:RANGE 而不是悄悄夹到边界 —— 把 1000 夹成 150
  *           会让人以为速度已经设成 1000 了，之后所有现象都难解释。
  */
static void Proto_CmdSpeed(const char *p)
{
    float v;

    if (Proto_ParseFloat(&p, &v) == 0)
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    Proto_SkipSep(&p);
    if (*p != '\0')
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    if (Motion_SetSpeed(v) != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:RANGE");
        return;
    }

    Protocol_Reply("OK %u", (unsigned int)Motion_GetSpeedMmS());
}

/**
  * 函    数：'T [停留毫秒]' —— 把当前姿态记成一个示教点
  * 参    数：p 指向 'T' 之后的第一个字符
  * 返 回 值：无（回 "OK <总点数>"）
  * 注    意：记的是【舵机安装角】，不是末端坐标 —— 这是整个示教回放能成立的前提：
  *           回放时不走逆解，所以既不受 66.3% 可达率限制，也不会被
  *           MOTION_PATH_CHECK 判 ERR:PATH。
  *
  *           运动中和回放中都不许记：
  *             运动中去记，记到的是插值中间姿态（OldAngle 还没走到 NewAngle），
  *             回放出来的每个点都会比示教时"差一点"，误差还会累积。
  */
static void Proto_CmdTeach(const char *p)
{
    float    v;
    uint16_t dwell = PLAY_DEFAULT_DWELL_MS;
    uint8_t  before;
    uint8_t  after;

    if (Play_IsPlaying() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }
    if (Motion_IsBusy() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    Proto_SkipSep(&p);
    if (*p != '\0')                         /* 带了停留时间 */
    {
        if (Proto_ParseFloat(&p, &v) == 0)
        {
            s_err_count++;
            Protocol_Reply("ERR:ARG");
            return;
        }
        Proto_SkipSep(&p);
        if (*p != '\0')
        {
            s_err_count++;
            Protocol_Reply("ERR:ARG");
            return;
        }
        if ((v < 0.0f) || (v > 60000.0f))
        {
            s_err_count++;
            Protocol_Reply("ERR:RANGE");
            return;
        }
        dwell = (uint16_t)(v + 0.5f);
    }

    before = Play_Count();
    after  = Play_Record(dwell);

    /* 表满时 Play_Record 不改点数，用前后对比来判断，比再查一次内部状态可靠 */
    if (after == before)
    {
        s_err_count++;
        Protocol_Reply("ERR:FULL");
        return;
    }

    Protocol_Reply("OK %u", (unsigned int)after);
}

/**
  * 函    数：'A <a1> <a2> <a3> <a4> <a5> <a6> [停留 ms]' —— 追加一个指定角度的点
  * 参    数：p 指向 'A' 之后的第一个字符
  * 返 回 值：无（回 "OK <追加后的点数>"，满表回 "ERR:FULL"）
  * 注    意：和 'T' 的唯一区别是角度由上位机给，不是取当前姿态。
  *
  *           存在的理由：回放是单片机执行的，点必须在 RAM 里。断联多半意味着
  *           USB 掉电，RAM 里的点表跟着没了，而上位机缓存的点表没有入口写回来。
  *           有了 'A'，上位机可以把 64 个点一次性推回去，不必把手臂摆到每个
  *           姿态再按一次记录。
  *
  *           角度一律 0~180（和 R 命令同一套判据），超了回 ERR:RANGE 而不是
  *           悄悄夹到边界 —— 夹了会让人以为这个点已经存进去了。
  *
  *           回放中拒绝：追加会改 s_count，正在跑的回放可能踩到还没写过的点。
  */
static void Proto_CmdAppend(const char *p)
{
    uint16_t a[6];
    uint16_t dwell = PLAY_DEFAULT_DWELL_MS;
    float    v;
    uint8_t  i;
    uint8_t  before;
    uint8_t  after;

    if (Play_IsPlaying() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    for (i = 0; i < 6u; i++)
    {
        if (Proto_ParseFloat(&p, &v) == 0)
        {
            s_err_count++;
            Protocol_Reply("ERR:ARG");
            return;
        }
        Proto_SkipSep(&p);
        if ((v < 0.0f) || (v > 180.0f))
        {
            s_err_count++;
            Protocol_Reply("ERR:RANGE");
            return;
        }
        a[i] = (uint16_t)(v + 0.5f);
    }

    if (*p != '\0')                         /* 带了停留时间 */
    {
        if (Proto_ParseFloat(&p, &v) == 0)
        {
            s_err_count++;
            Protocol_Reply("ERR:ARG");
            return;
        }
        Proto_SkipSep(&p);
        if (*p != '\0')
        {
            s_err_count++;
            Protocol_Reply("ERR:ARG");
            return;
        }
        if ((v < 0.0f) || (v > 60000.0f))
        {
            s_err_count++;
            Protocol_Reply("ERR:RANGE");
            return;
        }
        dwell = (uint16_t)(v + 0.5f);
    }

    before = Play_Count();
    after  = Play_Append(a, dwell);
    if (after == before)
    {
        s_err_count++;
        Protocol_Reply("ERR:FULL");
        return;
    }

    Protocol_Reply("OK %u", (unsigned int)after);
}

/**
  * 函    数：'L' —— 把点表逐条吐出来
  * 返 回 值：无
  * 注    意：每个点一行 "L <序号> <6 个角度> <停留> ..."，最后一行 "L END <总数>"。
  *           带 L END 而不是让上位机数行数，是因为点数可能为 0 —— 那样
  *           一条 L 都不会发，上位机就永远等不到"结束"。
  *
  *           回放中拒绝：一次 64 行是阻塞发送（约 2KB / 115200 ≈ 180ms），
  *           会把回放的节奏顶掉一拍。
  */
static void Proto_CmdList(void)
{
    uint8_t  i;
    uint8_t  n;
    uint16_t a[6];
    uint16_t dwell;

    if (Play_IsPlaying() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    n = Play_Count();
    for (i = 0; i < n; i++)
    {
        (void)Play_GetPoint(i, a, &dwell);
        Protocol_Reply("L %u %u %u %u %u %u %u %u",
                       (unsigned int)(i + 1u),
                       (unsigned int)a[0], (unsigned int)a[1],
                       (unsigned int)a[2], (unsigned int)a[3],
                       (unsigned int)a[4], (unsigned int)a[5],
                       (unsigned int)dwell);
    }
    Protocol_Reply("L END %u", (unsigned int)n);
}

/**
  * 函    数：'P [遍数]' —— 开始回放
  * 参    数：p 指向 'P' 之后的第一个字符；不带参数 = 无限循环
  * 返 回 值：无
  * 注    意：每进入一个点会主动推 "PLAY <序号>/<总数>"，整轮跑完推 "PLAY END"。
  *           不做单次/循环两个命令字，用一个可选参数表达，少记一个字母。
  */
static void Proto_CmdPlay(const char *p)
{
    float   v;
    uint8_t rounds = 0u;                    /* 0 = 无限循环 */

    if (Play_IsPlaying() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }
    if (Motion_IsBusy() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    Proto_SkipSep(&p);
    if (*p != '\0')
    {
        if (Proto_ParseFloat(&p, &v) == 0)
        {
            s_err_count++;
            Protocol_Reply("ERR:ARG");
            return;
        }
        Proto_SkipSep(&p);
        if (*p != '\0')
        {
            s_err_count++;
            Protocol_Reply("ERR:ARG");
            return;
        }
        if ((v < 0.0f) || (v > 250.0f))
        {
            s_err_count++;
            Protocol_Reply("ERR:RANGE");
            return;
        }
        rounds = (uint8_t)(v + 0.5f);
    }

    if (Play_Start(rounds) != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:EMPTY");
        return;
    }

    Protocol_Reply("OK %u", (unsigned int)Play_Count());
}

/**
  * 函    数：'D <序号>' —— 删除一个示教点
  * 注    意：序号 1 基，与 'L' 打印出来的序号一致，避免"看着第 3 个删掉第 2 个"。
  */
static void Proto_CmdDelete(const char *p)
{
    float   v;
    uint8_t n;

    if (Play_IsPlaying() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    if (Proto_ParseFloat(&p, &v) == 0)
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }
    Proto_SkipSep(&p);
    if (*p != '\0')
    {
        s_err_count++;
        Protocol_Reply("ERR:ARG");
        return;
    }

    n = Play_Count();
    if ((v < 1.0f) || (v > (float)n))
    {
        s_err_count++;
        Protocol_Reply("ERR:INDEX");
        return;
    }

    (void)Play_Delete((uint8_t)((uint8_t)(v + 0.5f) - 1u));
    Protocol_Reply("OK %u", (unsigned int)Play_Count());
}

/**
  * 函    数：'W' —— 把点表存进 Flash
  * 注    意：只存不读；上电时 Play_Init() 会自动载入。
  *           不自动存是因为每记录一个点就擦写一次 Flash 既慢又费寿命，
  *           示教过程可能要反复重记，等到满意了再显式存一次更合理。
  */
static void Proto_CmdSave(void)
{
    if (Play_IsPlaying() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:BUSY");
        return;
    }

    if (Play_Save() != 0u)
    {
        s_err_count++;
        Protocol_Reply("ERR:FLASH");
        return;
    }
    Protocol_Reply("OK %u", (unsigned int)Play_Count());
}

/**
  * 函    数：'?' —— 回一行当前状态
  * 返 回 值：无
  * 注    意：格式固定 11 个字段，上位机可以按空格 split 后按位置取值：
  *           S <x_mm> <y_mm> <z_mm> <底座> <肩> <肘> <腕> <夹爪> <旋转> <busy>
  *           坐标取的是【目标】而不是当前推进点：上位机关心的是「要去哪」。
  *           角度读 NewAngle（目标值），不是 OldAngle（插值中间值）。
  *           busy = 1 时不要发下一条 M，等 DONE。
  *           夹爪 = 舵机 6（PA7，开合收紧），旋转 = 舵机 5（PA6）。
  */
static void Proto_CmdStatus(void)
{
    float tx;
    float ty;
    float tz;

    Motion_GetTarget(&tx, &ty, &tz);

    Protocol_Reply("S %d %d %d %u %u %u %u %u %u %u",
                   Proto_Mm(tx), Proto_Mm(ty), Proto_Mm(tz),
                   (unsigned int)servo_motors[0].NewAngle,
                   (unsigned int)servo_motors[1].NewAngle,
                   (unsigned int)servo_motors[2].NewAngle,
                   (unsigned int)servo_motors[3].NewAngle,
                   (unsigned int)servo_motors[5].NewAngle,   /* PA7 开合 */
                   (unsigned int)servo_motors[4].NewAngle,   /* PA6 旋转 */
                   (unsigned int)Motion_IsBusy());
}

/**
  * 函    数：处理一条已经拼好的完整命令
  * 参    数：s 以 '\0' 结尾的命令字符串（不含 \r \n）
  * 返 回 值：无
  * 注    意：空行不回任何东西。上位机习惯发 "\r\n\r\n" 结尾，
  *           如果空行也回 ERR，日志会被刷满。
  */
static void Proto_HandleLine(char *s)
{
    char *p = s;

    while ((*p == ' ') || (*p == '\t'))
    {
        p++;
    }
    if (*p == '\0')
    {
        return;                             /* 空行静默忽略 */
    }

    switch (*p)
    {
        case 'M':
        case 'm':
            Proto_CmdMove(p + 1);
            break;

        /* 纯偏航：只转底座，肩/肘/腕保持不动。
         * 字母选 Y 是因为 P 已经是回放、R 已经是夹爪旋转，
         * 而 Y 恰好对应世界坐标的偏航轴，不会和别的命令字混。 */
        case 'Y':
        case 'y':
            Proto_CmdYaw(p + 1);
            break;

        case 'G':
        case 'g':
            Proto_CmdGrip(p + 1);
            break;

        case 'R':
        case 'r':
            Proto_CmdRotate(p + 1);
            break;

        /* ===== 速度在线可调（3.0） =====
         * V 是正式命令字；S 留作别名，兼容上位机可能已经按 S 写好的脚本。
         * 两个字母指向同一个 handler，不存在两份实现要同步的问题。
         * 命令方向不会收到 "S x y z ..." 那种状态应答（那是本机发出的），
         * 所以这里的 S 不会和 '?' 的应答格式打架。 */
        case 'V':
        case 'v':
        case 'S':
        case 's':
            Proto_CmdSpeed(p + 1);
            break;

        /* ===== 示教 / 回放（3.0） ===== */
        case 'A':
        case 'a':
            Proto_CmdAppend(p + 1);
            break;

        case 'T':
        case 't':
            Proto_CmdTeach(p + 1);
            break;

        case 'L':
        case 'l':
            Proto_CmdList();
            break;

        case 'P':
        case 'p':
            Proto_CmdPlay(p + 1);
            break;

        case 'X':
        case 'x':
            Play_Stop();
            Protocol_Reply("OK");
            break;

        case 'C':
        case 'c':
            /* 只清 RAM，不动 Flash：擦 Flash 是不可逆的，
             * 误按 C 之后还能靠复位重新载入。要真删就再发一次 W。 */
            Play_Clear();
            Protocol_Reply("OK");
            break;

        case 'D':
        case 'd':
            Proto_CmdDelete(p + 1);
            break;

        case 'W':
        case 'w':
            Proto_CmdSave();
            break;

        case '?':
            Proto_CmdStatus();
            break;

        default:
            s_err_count++;
            Protocol_Reply("ERR:CMD");
            break;
    }
}

/**
  * 函    数：喂一个字节给行解析器
  * 参    数：c 收到的字节
  * 返 回 值：无
  * 注    意：'\r' 直接丢掉，所以 \n 和 \r\n 两种结尾都能用。
  *           控制字符（0x00~0x1F、0x7F）也丢掉：串口线噪或波特率不对时
  *           会收到这类字节，留着会把命令字符串截断成半条。
  */
static void Proto_FeedByte(uint8_t c)
{
    if (c == '\r')
    {
        return;
    }

    if (c == '\n')
    {
        if (s_discard)
        {
            s_discard = 0;                  /* 超长行到此结束，整行已经报过 ERR:LONG */
            s_len = 0;
            return;
        }
        if (s_len == 0)
        {
            return;                         /* 空行 */
        }

        s_line[s_len] = '\0';
        s_rx_lines++;
        Proto_HandleLine((char *)s_line);
        s_len = 0;
        return;
    }

    if (s_discard)
    {
        return;                             /* 正在丢弃超长行的剩余部分 */
    }

    if ((c < 0x20) || (c == 0x7F))
    {
        return;                             /* 控制字符，丢掉 */
    }

    if (s_len >= (PROTO_LINE_MAX - 1))
    {
        s_discard = 1;
        s_len = 0;
        s_err_count++;
        Protocol_Reply("ERR:LONG");
        return;
    }

    s_line[s_len] = c;
    s_len++;
}

/**
  * 函    数：初始化协议层
  * 参    数：无
  * 返 回 值：无
  * 注    意：必须在 Uart_Init() 之后调用。这里不碰舵机也不碰运动，
  *           位姿由 Motion_Init() 负责。
  */
void Protocol_Init(void)
{
    s_len = 0;
    s_discard = 0;
    s_wait_done = 0;
    s_rx_lines = 0;
    s_err_count = 0;
}

/**
  * 函    数：轮询串口并解析
  * 参    数：无
  * 返 回 值：无
  * 注    意：一次把缓冲里的字节全取干净，所以不依赖 Uart_FrameReady()。
  *           按 IDLE 切帧在这里是错的：连发两条命令会被合并成一帧。
  *           行缓冲天然能处理「一条命令跨两次 Uart_Read」，因此只管
  *           字节流即可。没有数据时只有一次 Uart_Read 的开销。
  *
  *           处理完字节后再查到位：如果这条 M 的目标就是当前位置，
  *           Motion 会直接回 busy=0，于是 OK 和 DONE 在同一轮里连着发出去，
  *           上位机不用为「原地不动」这种情况写特例。
  */
void Protocol_Poll(void)
{
    uint8_t  chunk[32];
    uint16_t n;
    uint16_t i;
    uint8_t  ev;
    uint8_t  idx;
    uint8_t  total;

    while ((n = Uart_Read(PROTO_UART, chunk, sizeof(chunk))) > 0)
    {
        for (i = 0; i < n; i++)
        {
            Proto_FeedByte(chunk[i]);
        }
    }

    if ((s_wait_done != 0) && (Motion_IsBusy() == 0))
    {
        s_wait_done = 0;
        Protocol_Reply("DONE");
    }

    /* 回放事件：由 play.c 在「进入下一个点」「整轮结束」时挂起来，
     * 这里取走并翻成文本。放在 Poll 里而不是 Play_Tick 里直接发，
     * 是为了让「谁能往串口写」这件事只有一个出口。 */
    ev = Play_PollEvent(&idx, &total);
    if (ev == PLAY_EV_STEP)
    {
        Protocol_Reply("PLAY %u/%u", (unsigned int)idx, (unsigned int)total);
    }
    else if (ev == PLAY_EV_END)
    {
        Protocol_Reply("PLAY END");
    }
}

/**
  * 函    数：向 PROTO_UART 回一行
  * 参    数：format 格式化字符串，后面跟可变参数
  * 返 回 值：无
  * 注    意：自动补 "\r\n"，调用方不用自己写。
  *           缓冲区 96 字节，超长会截断——应答都是定长短句，不会到这。
  */
void Protocol_Reply(const char *fmt, ...)
{
    char    buf[96];
    va_list arg;
    int     n;

    if (fmt == NULL)
    {
        return;
    }

    va_start(arg, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, arg);
    va_end(arg);

    if (n <= 0)
    {
        return;
    }
    if (n > (int)sizeof(buf) - 3)           /* 给 \r\n\0 留位，超长就截断 */
    {
        n = (int)sizeof(buf) - 3;
    }

    buf[n] = '\r';
    buf[n + 1] = '\n';
    Uart_Write(PROTO_UART, (uint8_t *)buf, (uint16_t)(n + 2));
}
