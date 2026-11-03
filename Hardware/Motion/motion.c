#include "motion.h"
#include "model.h"
#include "Servo_motor.h"
#include <math.h>

/* ===== 状态 ===== */
static float    s_tx, s_ty, s_tz;        /* 目标坐标 */
static float    s_cx, s_cy, s_cz;        /* 当前实际推进到的坐标 */
static float    s_accum;                 /* 时间累加器：攒够 MOTION_STEP_MIN 才走一步 */
static uint32_t s_last_ms;               /* 上次 Tick 的时刻 */
static uint8_t  s_busy;                  /* 1 = 还在朝目标走 */
static uint8_t  s_last_err;              /* 上次 SetTarget 的结果 */

static float    s_vmax;                  /* 速度上限（米/秒），运行时可调 */
static float    s_v;                     /* 当前速度（米/秒），梯形剖面的中间量 */

/* ===== 纯偏航模式的状态 =====
 * 和笛卡尔那套并列存在，靠 s_pol 二选一。
 * 为什么不复用 s_tx/s_ty/s_tz 表达角度：那三个量的含义是「末端在空间里的
 * 位置」，一旦允许其中一个是角度，所有 hypot/距离/速度的式子都变成
 * 量纲混合（米 + 度），梯形剖面算出来的就不是速度了。宁可多存三个量。 */
static uint8_t  s_pol;                   /* 1 = 纯偏航模式 */
static float    s_pr;                    /* 锁定的半径（米） */
static float    s_ph;                    /* 锁定的高度（米） */
static float    s_yaw_c;                 /* 当前偏航角（度） */
static float    s_yaw_t;                 /* 目标偏航角（度） */
static float    s_w;                     /* 当前角速度（度/秒） */
static float    s_accum_d;               /* 角累加器（度） */

#define MOTION_D2R  (3.1415926f / 180.0f)
#define MOTION_R2D  (180.0f / 3.1415926f)

/* 初始位姿，和 main.c 原来写死的初值保持一致 */
#define MOTION_START_X      0.28f
#define MOTION_START_Y      0.10f
#define MOTION_START_Z      0.0f

/**
  * 函    数：求两点直线距离
  */
static float Motion_Dist(float dx, float dy, float dz)
{
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/**
  * 函    数：初始化
  * 注    意：目标与当前都设到初始点，所以上电第一次 Tick 不会动。
  *           s_last_ms 必须在这里取一次，否则第一次 Tick 的 dt 会是
  *           「上电到现在」的一大坨，被 MOTION_DT_MAX_MS 截断后照样跳一下。
  */
void Motion_Init(void)
{
    s_tx = s_cx = MOTION_START_X;
    s_ty = s_cy = MOTION_START_Y;
    s_tz = s_cz = MOTION_START_Z;

    s_accum    = 0.0f;
    s_last_ms  = HAL_GetTick();
    s_busy     = 0;
    s_last_err = MOTION_OK;

    /* 速度相关：上限取默认值，当前速度从 0 起步。
     * 从 0 起步而不是直接等于上限，否则第一段动作就没有加速过程了。 */
    s_vmax = MOTION_SPEED_DEFAULT;
    s_v    = 0.0f;

    /* 偏航模式：上电默认不在这个模式，先把量清干净 */
    s_pol     = 0u;
    s_pr      = 0.0f;
    s_ph      = 0.0f;
    s_yaw_c   = 0.0f;
    s_yaw_t   = 0.0f;
    s_w       = 0.0f;
    s_accum_d = 0.0f;

    /* 先把舵机摆到初始位姿，免得等第一个 Tick 才动 */
    (void)model_now(s_cx, s_cy, s_cz);
}

/**
  * 函    数：沿当前点→目标点直线采样，校验整条路径是否都在可达区内
  * 参    数：tx/ty/tz 目标坐标（米）
  * 返 回 值：MOTION_OK 路径可行；MOTION_ERR_PATH 中途有不可达点
  * 注    意：只校验终点是不够的。起点和终点都可能可达，中间却穿出一个
  *           不可达的空腔（三连杆的工作空间本来就是个带洞的壳）。
  *           那种情况下末端会走到洞边卡住、等目标继续推进才恢复，
  *           表现就是「走到一半顿一下」——这就是要提前拒掉的原因。
  *
  *           采样从 i=1 开始，起点不用校验（它就在那儿）。
  */
uint8_t Motion_CheckPath(float tx, float ty, float tz)
{
    float    dx = tx - s_cx;
    float    dy = ty - s_cy;
    float    dz = tz - s_cz;
    float    dist;
    uint32_t n;
    uint32_t i;

    dist = Motion_Dist(dx, dy, dz);
    if (dist <= MOTION_ARRIVE_EPS)
    {
        return MOTION_OK;                   /* 原地不动，无所谓路径 */
    }

    n = (uint32_t)(dist / MOTION_PATH_STEP) + 1u;
    if (n > MOTION_PATH_MAX_SAMPLES)
    {
        n = MOTION_PATH_MAX_SAMPLES;        /* 跨度太大时等分成上限份，别让受理变卡顿 */
    }

    for (i = 1; i <= n; i++)
    {
        float t = (float)i / (float)n;

        if (model_reachable(s_cx + dx * t, s_cy + dy * t, s_cz + dz * t) != 0)
        {
            return MOTION_ERR_PATH;
        }
    }
    return MOTION_OK;
}

/**
  * 函    数：设定目标坐标
  * 参    数：x/y/z 目标坐标（米）
  * 返 回 值：MOTION_OK 已受理；其余见头文件
  * 注    意：三重校验，顺序有讲究——
  *           1) 数值范围：先挡住 280 这种单位笔误，不必浪费逆解
  *           2) 终点可达：目标本身就不在空间里，直接拒
  *           3) 路径可达：起点终点都行，但中间穿洞，也拒
  *           校验通过才改目标，失败时目标和运动状态原封不动，
  *           所以「越界的目标直接被拒」不会把机械臂带偏。
  */
uint8_t Motion_SetTarget(float x, float y, float z)
{
    uint8_t ret;

    if ((x > MOTION_COORD_LIMIT) || (x < -MOTION_COORD_LIMIT) ||
        (y > MOTION_COORD_LIMIT) || (y < -MOTION_COORD_LIMIT) ||
        (z > MOTION_COORD_LIMIT) || (z < -MOTION_COORD_LIMIT))
    {
        s_last_err = MOTION_ERR_PARAM;
        return MOTION_ERR_PARAM;
    }

    if (model_reachable(x, y, z) != 0)
    {
        s_last_err = MOTION_ERR_OUT;
        return MOTION_ERR_OUT;
    }

#if MOTION_PATH_CHECK
    ret = Motion_CheckPath(x, y, z);
    if (ret != MOTION_OK)
    {
        s_last_err = ret;
        return ret;
    }
#endif

    /* 来这里就意味着回到笛卡尔：离开纯偏航模式，起点取当前所在的弧上点
     * （s_cx/s_cy/s_cz 在偏航期间一直跟着更新，所以这里是无缝的）。
     *
     * s_accum 只在「刚从偏航模式切回来」时清：偏航那段时间它没被用过，
     * 留着的是进入之前的陈旧零头，叠到新路径上就是白走 0.4mm。
     * 而笛卡尔模式内部连发 M 时【不能】清，理由见本节末尾的注释。 */
    if (s_pol != 0u)
    {
        s_accum = 0.0f;
    }
    s_pol = 0u;
    s_w   = 0.0f;

    s_tx = x;
    s_ty = y;
    s_tz = z;

    /* 这里【不】清 s_accum，也【不】清 s_v —— 这是梯形加速之后的一个关键改动，
     * 和 2.0 的做法相反，原因值得记下来：
     *
     *   2.0 清累加器，是为了防止「上一次剩的半截位移叠到新路径上」。
     *   但那是在匀速模型下：累加器直接等于将要走的距离，清了就清干净了。
     *
     *   现在累加器只是「还不够一个 MOTION_STEP_MIN 的零头」（< 0.5mm），
     *   和走哪条路径无关。而按键遥控时目标每轮主循环都在变（一次 5mm），
     *   如果每次都清，累加器永远攒不到 0.5mm —— 表现就是按住键手臂不动。
     *   速度同理：每次清零就等于每次从 0 重新加速，永远到不了设定速度。
     *
     *   中途改目标要不要担心「冲过头」？不需要：Tick 里算出的速度上限
     *   是 sqrt(2*a*剩余距离)，目标一近，上限立刻掉下来，速度跟着掉。
     */
    s_busy  = (Motion_Dist(s_tx - s_cx, s_ty - s_cy, s_tz - s_cz) > MOTION_ARRIVE_EPS) ? 1 : 0;

    s_last_err = MOTION_OK;
    return MOTION_OK;
}

/**
  * 函    数：把锁定半径上的当前偏航角同步成笛卡尔坐标
  * 注    意：偏航模式下 s_cx/s_cy/s_cz 必须始终跟着走。否则中途发一条 M，
  *           Motion 会以为还停在转之前的位置，第一步就把末端拽回旧点再出发。
  */
static void Motion_PolarSync(void)
{
    float rad = s_yaw_c * MOTION_D2R;

    s_cx = s_pr * cosf(rad);
    s_cy = s_ph;
    s_cz = s_pr * sinf(rad);
}

/**
  * 函    数：只下发底座那一路 PWM
  * 注    意：不走 model_now()，理由见 motion.h——这里只需要转一个关节。
  *           waist = SERVO_ZERO_W + SERVO_DIR_W · yaw，和 model.c 的逆解
  *           共用同一对标定常量（定义在 model.h）。
  *
  *           早期这里硬编码成 w = s_yaw_c + 90，等于假设「舵机角增大 = 朝 +z 转」；
  *           实物底座方向相反，于是偏航命令下去手臂和 3D 图各转一边。
  *           凡是要把「逻辑角」变成「安装角」的地方都得走这一对常量。
  */
static void Motion_PolarApply(void)
{
    float w = (float)SERVO_ZERO_W + (float)SERVO_DIR_W * s_yaw_c;

    if (w < 0.0f)   { w = 0.0f; }
    if (w > 180.0f) { w = 180.0f; }

    /* 四舍五入而不是 (uint16_t) 截断：截断会让 89.9° 变成 89°，
     * 每次都偏半度且不可恢复，四舍五入把误差压在一半以内。 */
    Servo_Motor_SetAngleNow(1, (uint16_t)(w + 0.5f));
}

/**
  * 函    数：纯偏航推进一步（梯形角速度剖面）
  * 注    意：形状和笛卡尔那条完全一样，只是把「米」换成「度」。
  *           sqrt(2·a·剩余角) 保证减速段刹得住，不会转过头再被拉回来。
  */
static void Motion_PolarTick(float dts)
{
    float dyaw  = s_yaw_t - s_yaw_c;
    float adist = fabsf(dyaw);
    float v_lim;
    float step;

    if (adist <= MOTION_YAW_EPS)
    {
        s_yaw_c   = s_yaw_t;
        s_busy    = 0u;
        s_w       = 0.0f;
        s_accum_d = 0.0f;
        Motion_PolarSync();
        Motion_PolarApply();
        return;
    }

    v_lim = sqrtf(2.0f * MOTION_YAW_ACCEL * adist);
    if (v_lim > MOTION_YAW_SPEED)
    {
        v_lim = MOTION_YAW_SPEED;
    }

    if (s_w < v_lim)
    {
        s_w += MOTION_YAW_ACCEL * dts;
        if (s_w > v_lim) { s_w = v_lim; }
    }
    else
    {
        s_w = v_lim;                        /* 贴着「还刹得住」的边界走 */
    }

    s_accum_d += s_w * dts;
    if (s_accum_d < MOTION_YAW_STEP_MIN)
    {
        return;                             /* 还不够 0.25°，先攒着 */
    }

    step      = s_accum_d;
    s_accum_d = 0.0f;

    if (step >= adist)
    {
        s_yaw_c = s_yaw_t;
        s_busy  = 0u;
        s_w     = 0.0f;
    }
    else
    {
        s_yaw_c += (dyaw > 0.0f) ? step : -step;
    }

    Motion_PolarSync();
    Motion_PolarApply();
}

/**
  * 函    数：按时间推进一次
  * 注    意：核心是「dt 从哪来」。用 HAL_GetTick() 的差值，而不是数圈数，
  *           这样主循环变快变慢都不影响末端速度。
  *
  *           三个保护：
  *           - dt == 0：同一毫秒内被调了两次，直接返回，避免除以 0 类的怪事
  *           - dt > MOTION_DT_MAX_MS：主循环被卡过，把欠账截断，别一步跳很远
  *           - 累加器没攒够 MOTION_STEP_MIN：先攒着，不白解逆解
  *
  *           ===== 梯形加减速（3.0 新增）=====
  *           除以「走多远」，这里多了「以多快的速度走」。速度不是常数，而是
  *           被两条曲线夹住的一个状态量：
  *
  *             上限 A：s_vmax          —— 人设的巡航速度（V 命令可改）
  *             上限 B：sqrt(2*a*剩余)   —— 以 MOTION_ACCEL 还能刹住的速度
  *
  *           实际速度取两者较小值，并且每轮最多变化 a*dt。于是：
  *             刚起步：剩余距离大，上限是 A，速度被 a*dt 慢慢抬上去 → 加速段
  *             走中段：速度顶到 A，不再变化                                  → 匀速段
  *             快到了：剩余距离变小，上限 B 掉到 A 以下，速度跟着掉          → 减速段
  *
  *           距离短的时候加速段和减速段会撞在一起，梯形退化成三角形，
  *           速度还没到 A 就开始减 —— 这正是我们要的，短距离本就该更慢。
  *
  *           为什么上限用 sqrt(2*a*d) 而不是「按比例分配时间」：
  *           这个式子就是匀减速的运动学解（v² = 2as），它保证在剩余距离内
  *           恰好能把速度降到 0。按比例分配只能得到「看起来像梯形」的形状，
  *           一旦巡航速度改了，减速段就可能刹不住，末端会撞一下目标点。
  */
void Motion_Tick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_last_ms;         /* 无符号减法，溢出绕回也正确 */
    float    dts;
    float    dx;
    float    dy;
    float    dz;
    float    dist;
    float    v_lim;
    float    step;

    if (dt == 0)
    {
        return;
    }
    s_last_ms = now;

    if (!s_busy)
    {
        s_v = 0.0f;                         /* 停着就明确归零，下次起步从 0 加速 */
        s_w = 0.0f;
        return;                             /* 已到位，不做任何计算 */
    }

    if (dt > MOTION_DT_MAX_MS)
    {
        dt = MOTION_DT_MAX_MS;
    }
    dts = (float)dt / 1000.0f;

    /* 纯偏航走另一条推进路径：不碰 s_v / s_accum，也不做直线插值，
     * 终点是「角度」而不是「坐标」。两条路只共用 dt 和 s_busy。 */
    if (s_pol != 0u)
    {
        Motion_PolarTick(dts);
        return;
    }

    dx   = s_tx - s_cx;
    dy   = s_ty - s_cy;
    dz   = s_tz - s_cz;
    dist = Motion_Dist(dx, dy, dz);

    if (dist <= MOTION_ARRIVE_EPS)
    {
        s_cx = s_tx;
        s_cy = s_ty;
        s_cz = s_tz;
        s_busy  = 0;
        s_accum = 0.0f;
        s_v     = 0.0f;
        (void)model_now(s_cx, s_cy, s_cz);  /* 收尾对齐到精确目标 */
        return;
    }

    /* ---- 算这一轮的速度上限 ----
     * 两个上限取小：巡航速度、以及「剩这么点距离还刹得住的最大速度」。
     * 后者随剩余距离单调下降，所以越接近目标，上限越低，速度自然衰减。 */
    v_lim = sqrtf(2.0f * MOTION_ACCEL * dist);
    if (v_lim > s_vmax)
    {
        v_lim = s_vmax;
    }

    if (s_v < v_lim)
    {
        /* 加速段：受加速度限制，不能一步到位 */
        s_v += MOTION_ACCEL * dts;
        if (s_v > v_lim)
        {
            s_v = v_lim;                    /* 正好在这一轮撞到上限，别越过 */
        }
    }
    else
    {
        /* 减速段：直接跟随上限，不做「按加速度慢慢减」。
         *
         * 为什么这里可以突变：v_lim 是【刹得住】的边界，只要速度不超过它，
         * 后续任何时刻都还能停下来。反过来，如果按 -a 慢慢减，万一目标
         * 在中途被改近（按键遥控就会），就会来不及，冲过目标再被拉回来。
         * 贴着边界走永远安全。 */
        s_v = v_lim;
    }

    /* 走多远 = 速度 × 时间。仍然过一遍累加器，理由和原来一样：
     * 主循环很快时单轮位移不足 0.5mm，不必为它解一次逆解。 */
    s_accum += s_v * dts;
    if (s_accum < MOTION_STEP_MIN)
    {
        return;                             /* 攒着，下一轮再说 */
    }

    step = s_accum;
    s_accum = 0.0f;

    if (step >= dist)
    {
        /* 这一步就能到，直接贴到目标，避免在附近来回微调 */
        s_cx = s_tx;
        s_cy = s_ty;
        s_cz = s_tz;
        s_busy = 0;
        s_v    = 0.0f;
    }
    else
    {
        /* 按方向走，走的是直线——每步都重算逆解，末端轨迹就是这条直线 */
        s_cx += dx / dist * step;
        s_cy += dy / dist * step;
        s_cz += dz / dist * step;
    }

    /* 注意：model_now 返回 1 时舵机保持不动。正常情况下 SetTarget 已经
     * 保证了路径可达，不会走到这里；但按键通道是逐步逼近的，
     * 理论上仍可能踩到边界，所以这里不因为失败就停下推进，
     * 否则末端会永远卡在不可达点前。 */
    (void)model_now(s_cx, s_cy, s_cz);
}

/**
  * 函    数：设定目标偏航角（纯偏航模式）
  * 参    数：deg 偏航角（度），正 = 朝 z 正方向
  * 返 回 值：MOTION_OK 已受理；MOTION_ERR_PARAM 超 ±MOTION_YAW_LIMIT；
  *           MOTION_ERR_OUT 该朝向下够不到，或半径太小、朝向没有定义
  * 注    意：首次调用锁定当前 r、h，之后连续调用只改角度。
  *           之所以锁定而不是每步重算，是为了「转过去再转回来」能精确回到原点：
  *           若每次都拿 s_cx/s_cz 反算半径，浮点误差会一步步累积，转十次半径就漂了。
  */
uint8_t Motion_SetYaw(float deg)
{
    float r;
    float h;
    float rad;

    if ((deg < -MOTION_YAW_LIMIT) || (deg > MOTION_YAW_LIMIT))
    {
        s_last_err = MOTION_ERR_PARAM;
        return MOTION_ERR_PARAM;
    }

    if (s_pol != 0u)
    {
        r = s_pr;                           /* 已锁定，直接沿用 */
        h = s_ph;
    }
    else
    {
        r = sqrtf(s_cx * s_cx + s_cz * s_cz);
        h = s_cy;

        if (r < MOTION_YAW_R_MIN)
        {
            s_last_err = MOTION_ERR_OUT;    /* 末端几乎在基座正上方，朝向无定义 */
            return MOTION_ERR_OUT;
        }
    }

    /* 按极坐标反算回笛卡尔点校验一次：可达性判据仍然只来自 model.c 那一份，
     * 不在这里另立一套「转到多少度算合法」的规则，否则两边迟早对不上。 */
    rad = deg * MOTION_D2R;
    if (model_reachable(r * cosf(rad), h, r * sinf(rad)) != 0)
    {
        s_last_err = MOTION_ERR_OUT;
        return MOTION_ERR_OUT;
    }

    if (s_pol == 0u)
    {
        s_pol     = 1u;
        s_pr      = r;
        s_ph      = h;
        s_yaw_c   = atan2f(s_cz, s_cx) * MOTION_R2D;   /* 从当前朝向接着转 */
        s_v       = 0.0f;
        s_w       = 0.0f;
        s_accum_d = 0.0f;
    }

    /* 这里【不】清 s_accum_d（除非刚进入模式，那个分支里清过了）。
     * 原因和笛卡尔那边 SetTarget 不清 s_accum 完全一样：串口连发
     * 「Y 10 / Y 11 / Y 12」时每轮都清的话，累加器永远攒不到 0.25°，
     * 表现就是发了命令手臂不动。残留的零头不到 0.25°，代价可以接受。 */
    s_yaw_t = deg;

    /* 目标坐标同步到弧上的终点：OLED 和 '?' 显示的是「要去哪」。
     * 不更新的话它会一直停在上一条 M 的目标点，看起来像命令没生效。 */
    s_tx = s_pr * cosf(s_yaw_t * MOTION_D2R);
    s_ty = s_ph;
    s_tz = s_pr * sinf(s_yaw_t * MOTION_D2R);

    s_busy     = (fabsf(s_yaw_t - s_yaw_c) > MOTION_YAW_EPS) ? 1u : 0u;
    s_last_err = MOTION_OK;
    return MOTION_OK;
}

float Motion_GetYaw(void)
{
    if (s_pol != 0u)
    {
        return s_yaw_c;
    }
    return atan2f(s_cz, s_cx) * MOTION_R2D;
}

uint8_t Motion_IsPolar(void)
{
    return s_pol;
}

uint8_t Motion_IsBusy(void)
{
    return s_busy;
}

void Motion_GetTarget(float *x, float *y, float *z)
{
    if (x != NULL) { *x = s_tx; }
    if (y != NULL) { *y = s_ty; }
    if (z != NULL) { *z = s_tz; }
}

void Motion_GetCurrent(float *x, float *y, float *z)
{
    if (x != NULL) { *x = s_cx; }
    if (y != NULL) { *y = s_cy; }
    if (z != NULL) { *z = s_cz; }
}

uint8_t Motion_GetLastError(void)
{
    return s_last_err;
}

/**
  * 函    数：设置速度上限
  * 参    数：mm_s 毫米/秒
  * 返 回 值：0 成功；1 超出 [MOTION_SPEED_MM_MIN, MOTION_SPEED_MM_MAX]
  * 注    意：只改上限，不改当前速度。正在走的那一段会在下一轮 Tick
  *           按新上限重新规划——已经走到的位置不受影响，所以随时改都安全。
  *
  *           越界时【不修改原值】：与其把 1000 悄悄夹成 150 让人以为
  *           设成了 1000，不如直接报错，让人回去改参数。
  */
uint8_t Motion_SetSpeed(float mm_s)
{
    if ((mm_s < (float)MOTION_SPEED_MM_MIN) || (mm_s > (float)MOTION_SPEED_MM_MAX))
    {
        return 1u;
    }

    s_vmax = mm_s / 1000.0f;
    return 0u;
}

uint16_t Motion_GetSpeedMmS(void)
{
    return (uint16_t)(s_vmax * 1000.0f + 0.5f);
}

uint16_t Motion_GetCurrentSpeedMmS(void)
{
    return (uint16_t)(s_v * 1000.0f + 0.5f);
}

/**
  * 函    数：用当前实际舵机角正解出末端坐标，并把运动状态对齐到该点
  * 注    意：只读 OldAngle（实际插值位置），不读 NewAngle（目标）——
  *           停在中途时这两者不同，而实物在 OldAngle 那一侧。
  *
  *           正解放在 model.c（和逆解同一份标定参数），这里不重复实现，
  *           否则标定参数一改就得改两处。
  */
void Motion_SyncFromCurrent(void)
{
    float x;
    float y;
    float z;

    model_fk(servo_motors[1].OldAngle,     /* 肩 */
             servo_motors[2].OldAngle,     /* 肘 */
             servo_motors[3].OldAngle,     /* 腕 */
             servo_motors[0].OldAngle,     /* 底座 */
             &x, &y, &z);

    s_tx = s_cx = x;
    s_ty = s_cy = y;
    s_tz = s_cz = z;

    s_busy     = 0;
    s_accum    = 0.0f;
    s_v        = 0.0f;                      /* 速度也要归零：这里代表「已经换了个起点」 */
    s_pol      = 0u;                        /* 刚被正解重定过位，不再是锁半径的弧上点 */
    s_w        = 0.0f;
    s_accum_d  = 0.0f;
    s_last_err = MOTION_OK;

    /* 正解失败（理论上不会：这组角就是实物当前姿态）时上面填的是 0，
     * 位置会显示成原点。与其编一个数，不如让它显示成 0 好让人看出来不对。 */
}
