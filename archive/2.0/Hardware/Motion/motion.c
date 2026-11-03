#include "motion.h"
#include "model.h"
#include <math.h>

/* ===== 状态 ===== */
static float    s_tx, s_ty, s_tz;        /* 目标坐标 */
static float    s_cx, s_cy, s_cz;        /* 当前实际推进到的坐标 */
static float    s_accum;                 /* 时间累加器：攒够 MOTION_STEP_MIN 才走一步 */
static uint32_t s_last_ms;               /* 上次 Tick 的时刻 */
static uint8_t  s_busy;                  /* 1 = 还在朝目标走 */
static uint8_t  s_last_err;              /* 上次 SetTarget 的结果 */

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

    s_tx = x;
    s_ty = y;
    s_tz = z;

    /* 目标变了，累加器要清零：否则上一次剩的半截位移会叠到新路径上，
     * 第一步的步长就偏大，看起来像「起步窜一下」 */
    s_accum = 0.0f;
    s_busy  = (Motion_Dist(s_tx - s_cx, s_ty - s_cy, s_tz - s_cz) > MOTION_ARRIVE_EPS) ? 1 : 0;

    s_last_err = MOTION_OK;
    return MOTION_OK;
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
  */
void Motion_Tick(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_last_ms;         /* 无符号减法，溢出绕回也正确 */
    float    dx;
    float    dy;
    float    dz;
    float    dist;
    float    step;

    if (dt == 0)
    {
        return;
    }
    s_last_ms = now;

    if (!s_busy)
    {
        return;                             /* 已到位，不做任何计算 */
    }

    if (dt > MOTION_DT_MAX_MS)
    {
        dt = MOTION_DT_MAX_MS;
    }

    /* 这一段该走多远 = 速度 × 时间 */
    s_accum += MOTION_SPEED * ((float)dt / 1000.0f);
    if (s_accum < MOTION_STEP_MIN)
    {
        return;                             /* 攒着，下一轮再说 */
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
        (void)model_now(s_cx, s_cy, s_cz);  /* 收尾对齐到精确目标 */
        return;
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
