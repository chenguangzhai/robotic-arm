#include "play.h"
#include "Servo_motor.h"
#include "motion.h"
#include <string.h>

/* ===== 点结构 =====
 * 6 个舵机角 + 到位后停留时间，共 14 字节，且全部是 uint16，没有填充字节，
 * 所以可以整块当 uint16 数组写进 Flash（见 Play_Save 的注释）。 */
typedef struct
{
    uint16_t a[6];          /* 舵机 1~6 的安装角（度） */
    uint16_t dwell;         /* 到位后停留多少毫秒 */
} Play_Point;

/* ===== Flash 存储布局 =====
 *
 * 存放在【最后一页】：F103C8 是 64KB Flash、1KB 一页，最后一页是
 * 0x0800FC00 ~ 0x0800FFFF。代码用了 63KB 以上就会撞车，所以 Play_Save()
 * 开头用链接脚本的 _etext 做一次运行期检查，越界直接拒绝，不冒险擦。
 *
 * 页内布局：
 *   [0]      Play_Header（12 字节：magic / version / count / crc / rsv）
 *   [12]     Play_Point[PLAY_MAX_POINTS]（64 × 14 = 896 字节）
 *   合计 908 字节 < 1024，一页放得下，不用分页也不处理跨页。
 */
#define PLAY_FLASH_PAGE     (0x0800FC00u)
#define PLAY_MAGIC          (0x33594C50u)   /* 小端读出来是 "PLY3" */
#define PLAY_VERSION        (1u)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t count;
    uint16_t crc;           /* 对 points 区算的 CRC16-CCITT */
    uint16_t rsv;           /* 补齐到 12 字节，方便按半字写 */
} Play_Header;

/* 链接脚本里 `_etext = .;` 标记代码段末尾，用来判断最后一页是否被动到 */
extern uint32_t _etext;

/* ===== 状态 ===== */
static Play_Point s_pt[PLAY_MAX_POINTS];
static uint8_t    s_count;

static uint8_t    s_playing;
static uint8_t    s_idx;            /* 当前正在执行的点的下标（0 基） */
static uint8_t    s_rounds_left;    /* 还剩几轮，0 且 s_loop = 1 时无限循环 */
static uint8_t    s_loop;

static uint8_t    s_phase;          /* 0 = 运动中，1 = 停留中 */
static uint32_t   s_phase_ms;       /* 本阶段开始的时刻 */
static uint32_t   s_seg_ms;         /* 本段运动总时长 */

static uint16_t   s_from[6];        /* 本段起点角 */
static uint16_t   s_to[6];          /* 本段终点角 */

/* 事件（协议层每轮取一次） */
static uint8_t    s_ev;
static uint8_t    s_ev_idx;

/**
  * 函    数：CRC16-CCITT（多项式 0x1021，初值 0xFFFF）
  * 注    意：只用来判断 Flash 里的点表是不是被写坏/被别的固件覆盖了。
  *           不做纠错，错了就当没存过，退回空表。
  */
static uint16_t Play_Crc16(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    uint8_t  b;

    for (i = 0; i < n; i++)
    {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (b = 0; b < 8u; b++)
        {
            if (crc & 0x8000u)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            }
            else
            {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/**
  * 函    数：梯形速度剖面 —— 把「这一段的进度 τ」映射成「该走完的比例 s」
  * 参    数：t 归一化时间 0~1
  * 返 回 值：归一化位移 0~1
  * 注    意：核心性质是 s'(0) = s'(1) = 0，也就是起止速度都为 0。
  *           线性插值 s = t 的导数恒为 1，意味着每段开头瞬间就到速、
  *           结尾瞬间停住——那才是「抽搐」的来源。
  *
  *           推导（记加速段占比 r）：
  *             速度三角形面积必须等于总位移 1，
  *             加速段面积 r/2 + 匀速段 (1-2r) + 减速段 r/2 = 1-r，
  *             所以匀速段的归一化速度 v_peak = 1/(1-r)。
  *           分段积分就得到下面三段式子。校验：s(r)=r/(2(1-r))、
  *           s(1-r)=1-r/(2(1-r))、s(1)=1，三处都接得上。
  *
  *           为什么在【关节空间】做而不是先转成末端坐标再规划：
  *           这里本来就是逐个关节各自插值，六个关节同时按同一个 s(t) 走，
  *           它们会在同一时刻加速、同一时刻减速，姿态过渡的节奏是整齐的。
  *           换成末端空间反而要再过一遍逆解，白白把示教回放的优势丢掉。
  */
static float Play_Profile(float t)
{
    const float r  = PLAY_ACCEL_RATIO;
    const float d  = 2.0f * r * (1.0f - r);   /* 加速段的分母，重复用 */
    float       k;

    if (t <= 0.0f)
    {
        return 0.0f;
    }
    if (t >= 1.0f)
    {
        return 1.0f;
    }

    if (t < r)                                 /* 加速段 */
    {
        return (t * t) / d;
    }
    if (t <= (1.0f - r))                       /* 匀速段 */
    {
        /* 常数项是加速段在 t=r 处的终点值 r²/d。
         * 写成 r/d 会大 1/r 倍，接缝处位置会跳 —— 这个坑值得留个记号。 */
        return ((r * r) / d) + ((t - r) / (1.0f - r));
    }
    k = 1.0f - t;                              /* 减速段：与加速段对称 */
    return 1.0f - (k * k) / d;
}

/**
  * 函    数：把一段角度原样下发给 6 个舵机
  * 注    意：用 SetAngleNow 而不是 SetAngle。理由见 play.h 顶部：
  *           TIM4 那套插值是 20°/s，回放用它会把每个点拖成好几秒；
  *           而且我们在主循环里已经自己插过值了，再让它插一遍会打架。
  */
static void Play_Apply(const uint16_t *a)
{
    uint8_t i;

    for (i = 0; i < 6u; i++)
    {
        Servo_Motor_SetAngleNow((uint8_t)(i + 1u), a[i]);
    }
}

/**
  * 函    数：从当前实际位置出发，朝第 idx 个点开始一段运动
  * 注    意：起点取 servo_motors[].OldAngle —— 停在半路再启动时，
  *           起点必须是「现在在哪」，而不是上一个点的目标值。
  */
static void Play_BeginSegment(uint8_t idx)
{
    uint8_t  i;
    uint8_t  max_delta = 0u;
    uint16_t d;

    for (i = 0; i < 6u; i++)
    {
        s_from[i] = servo_motors[i].OldAngle;
        s_to[i]   = s_pt[idx].a[i];

        d = (s_to[i] > s_from[i]) ? (uint16_t)(s_to[i] - s_from[i])
                                  : (uint16_t)(s_from[i] - s_to[i]);
        if (d > max_delta)
        {
            max_delta = d;
        }
    }

    /* 段时长按【最大关节位移】算，而不是按某个固定时间：
     * 这样 5° 的小动作不会也拖 1 秒，90° 的大动作也不会快到甩飞工件。 */
    s_seg_ms = (uint32_t)max_delta * PLAY_MS_PER_DEG;
    if (s_seg_ms < PLAY_MIN_SEG_MS)
    {
        s_seg_ms = PLAY_MIN_SEG_MS;
    }
    if (s_seg_ms > PLAY_MAX_SEG_MS)
    {
        s_seg_ms = PLAY_MAX_SEG_MS;
    }

    s_phase    = 0u;
    s_phase_ms = HAL_GetTick();

    s_ev     = PLAY_EV_STEP;
    s_ev_idx = (uint8_t)(idx + 1u);
}

/**
  * 函    数：结束一轮
  * 注    意：s_loop 为真就一直循环；否则扣次数，扣完才真正结束。
  *           结束时把 Motion 的「当前坐标」同步过来 —— 否则下一次发 M，
  *           运动模块还以回放前的老坐标做路径校验和直线插值，第一步会跳。
  */
static void Play_EndRound(void)
{
    if ((s_loop == 0u) && (s_rounds_left > 0u))
    {
        s_rounds_left--;
    }

    if ((s_loop != 0u) || (s_rounds_left > 0u))
    {
        s_idx = 0u;
        Play_BeginSegment(0u);
        return;
    }

    s_playing = 0u;
    s_ev      = PLAY_EV_END;
    Motion_SyncFromCurrent();
}

/**
  * 函    数：初始化
  * 注    意：只从 Flash 读点表，绝不驱动舵机。
  *           上电姿态由 Servo_motor_Init() + Motion_Init() 决定；
  *           如果这里顺手「恢复上次姿态」，上电就会先弹一下再回工作位。
  */
void Play_Init(void)
{
    const Play_Header *h = (const Play_Header *)PLAY_FLASH_PAGE;

    s_count       = 0u;
    s_playing     = 0u;
    s_idx         = 0u;
    s_rounds_left = 0u;
    s_loop        = 0u;
    s_phase       = 0u;
    s_ev          = PLAY_EV_NONE;
    s_ev_idx      = 0u;

    if ((h->magic != PLAY_MAGIC) || (h->version != PLAY_VERSION))
    {
        return;                             /* 从没存过，或存的是别的格式 */
    }
    if ((h->count == 0u) || (h->count > PLAY_MAX_POINTS))
    {
        return;
    }

    {
        const uint8_t *base = (const uint8_t *)(PLAY_FLASH_PAGE + sizeof(Play_Header));
        uint32_t       n    = (uint32_t)h->count * sizeof(Play_Point);

        if (Play_Crc16(base, n) != h->crc)
        {
            return;                         /* 校验不过，当成没存过 */
        }
        memcpy(s_pt, base, n);
        s_count = (uint8_t)h->count;
    }
}

uint8_t Play_Count(void)
{
    return s_count;
}

uint8_t Play_Record(uint16_t dwell_ms)
{
    uint8_t i;

    if (s_count >= PLAY_MAX_POINTS)
    {
        return s_count;
    }

    for (i = 0; i < 6u; i++)
    {
        s_pt[s_count].a[i] = servo_motors[i].OldAngle;
    }
    s_pt[s_count].dwell = dwell_ms;

    s_count++;
    return s_count;
}

uint8_t Play_Append(const uint16_t angles[6], uint16_t dwell_ms)
{
    uint8_t i;

    if (s_count >= PLAY_MAX_POINTS)
    {
        return s_count;
    }
    if (angles == NULL)
    {
        return s_count;
    }

    for (i = 0; i < 6u; i++)
    {
        s_pt[s_count].a[i] = angles[i];
    }
    s_pt[s_count].dwell = dwell_ms;

    s_count++;
    return s_count;
}

uint8_t Play_GetPoint(uint8_t index, uint16_t angles[6], uint16_t *dwell_ms)
{
    uint8_t i;

    if (index >= s_count)
    {
        return 1u;
    }
    for (i = 0; i < 6u; i++)
    {
        angles[i] = s_pt[index].a[i];
    }
    if (dwell_ms != NULL)
    {
        *dwell_ms = s_pt[index].dwell;
    }
    return 0u;
}

uint8_t Play_Delete(uint8_t index)
{
    uint8_t i;

    if (index >= s_count)
    {
        return 1u;
    }

    for (i = index; (uint8_t)(i + 1u) < s_count; i++)
    {
        s_pt[i] = s_pt[i + 1u];
    }
    s_count--;

    /* 删到正在执行的点之前时，下标要跟着挪，否则回放会跳过一个点 */
    if (s_playing != 0u)
    {
        if (s_count == 0u)
        {
            Play_Stop();
        }
        else if (index < s_idx)
        {
            s_idx--;
        }
        else if (s_idx >= s_count)
        {
            s_idx = (uint8_t)(s_count - 1u);
        }
    }
    return 0u;
}

void Play_Clear(void)
{
    s_playing = 0u;
    s_count   = 0u;
    s_idx     = 0u;
}

uint8_t Play_Start(uint8_t rounds)
{
    if (s_count == 0u)
    {
        return 1u;
    }

    s_idx    = 0u;
    s_playing = 1u;
    s_phase   = 0u;

    if (rounds == 0u)
    {
        s_loop        = 1u;
        s_rounds_left = 0u;
    }
    else
    {
        s_loop        = 0u;
        s_rounds_left = rounds;
    }

    Play_BeginSegment(0u);
    return 0u;
}

void Play_Stop(void)
{
    if (s_playing == 0u)
    {
        return;
    }

    s_playing = 0u;

    /* 停在半路的姿态可能是任何一个中间值，同步一次，
     * 让 Motion 的「当前点」和实物对上，下次发 M 才不会跳。 */
    Motion_SyncFromCurrent();
}

uint8_t Play_IsPlaying(void)
{
    return s_playing;
}

uint8_t Play_Index(void)
{
    return (s_playing != 0u) ? (uint8_t)(s_idx + 1u) : 0u;
}

/**
  * 函    数：主循环推进
  * 注    意：这里是「按时间推进」的思路和 motion.c 一致，但不用累加器：
  *           关节插值要的是「这一段走了百分之几」，直接用
  *           (now - 起点) / 段时长 更直接，也不会因为主循环抖动而在
  *           到达时多算几步。
  */
void Play_Tick(void)
{
    uint32_t now;
    uint32_t el;
    uint8_t  i;

    if (s_playing == 0u)
    {
        return;
    }

    now = HAL_GetTick();
    el  = now - s_phase_ms;

    if (s_phase == 0u)                      /* ---- 运动中 ---- */
    {
        if (el >= s_seg_ms)
        {
            Play_Apply(s_to);               /* 收尾贴到精确角度 */

            s_phase    = 1u;
            s_phase_ms = now;

            /* 停留时间也走「0 就立刻进下一点」的同一套逻辑，
             * 不特殊处理，避免出现两个分支各算一套时间。 */
            return;
        }

        /* 用梯形剖面代替线性插值：每段起停都有加减速过程。
         * 六个关节共用同一个 s，所以它们同时加速、同时减速，
         * 姿态过渡看起来是一个整体动作，而不是六路各自为政。 */
        {
            float s = Play_Profile((float)el / (float)s_seg_ms);

            for (i = 0; i < 6u; i++)
            {
                int32_t span = (int32_t)s_to[i] - (int32_t)s_from[i];
                int32_t v    = (int32_t)s_from[i] + (int32_t)((float)span * s + ((span >= 0) ? 0.5f : -0.5f));

                if (v < 0)   { v = 0; }
                if (v > 180) { v = 180; }
                Servo_Motor_SetAngleNow((uint8_t)(i + 1u), (uint16_t)v);
            }
        }
        return;
    }

    /* ---- 停留中 ---- */
    if (el < (uint32_t)s_pt[s_idx].dwell)
    {
        return;
    }

    s_idx++;
    if (s_idx >= s_count)
    {
        Play_EndRound();                    /* 一轮结束：循环或收工 */
        return;
    }

    Play_BeginSegment(s_idx);
}

uint8_t Play_PollEvent(uint8_t *index, uint8_t *total)
{
    uint8_t ev = s_ev;

    if (ev == PLAY_EV_NONE)
    {
        return PLAY_EV_NONE;
    }

    s_ev = PLAY_EV_NONE;                    /* 一次性，取走就清 */

    if (index != NULL)
    {
        *index = (ev == PLAY_EV_STEP) ? s_ev_idx : 0u;
    }
    if (total != NULL)
    {
        *total = s_count;
    }
    return ev;
}

/**
  * 函    数：把点表写进 Flash
  * 返 回 值：0 成功；1 失败
  * 注    意：整页擦除再写，不做增量 —— 点表本来就小，增量写要维护
  *           多个 slot 和垃圾回收，对这个规模不值得。
  *
  *           写之前检查 _etext：如果固件已经长到这一页，宁可报错也不擦，
  *           擦掉的可能是正在跑的代码。
  */
uint8_t Play_Save(void)
{
    const uint16_t *src;
    Play_Header     h;
    FLASH_EraseInitTypeDef er;
    uint32_t        page_err = 0u;
    uint32_t        addr;
    uint32_t        n;
    uint32_t        i;
    uint8_t         ret = 0u;

    if ((uint32_t)(&_etext) > PLAY_FLASH_PAGE)
    {
        return 1u;                          /* 代码区已经吃到这一页，拒绝 */
    }

    h.magic   = PLAY_MAGIC;
    h.version = PLAY_VERSION;
    h.count   = s_count;
    h.rsv     = 0u;
    h.crc     = Play_Crc16((const uint8_t *)s_pt,
                           (uint32_t)s_count * sizeof(Play_Point));

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.PageAddress = PLAY_FLASH_PAGE;
    er.NbPages     = 1u;
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 1u;
    }

    /* 头 12 字节 = 6 个半字 */
    src  = (const uint16_t *)&h;
    addr = PLAY_FLASH_PAGE;
    for (i = 0; i < (sizeof(Play_Header) / 2u); i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr,
                              (uint64_t)src[i]) != HAL_OK)
        {
            ret = 1u;
            break;
        }
        addr += 2u;
    }

    /* 点表：Play_Point 全是 uint16 且无填充，可以直接按半字搬 */
    if (ret == 0u)
    {
        src = (const uint16_t *)s_pt;
        n   = (uint32_t)s_count * (sizeof(Play_Point) / 2u);
        for (i = 0; i < n; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr,
                                  (uint64_t)src[i]) != HAL_OK)
            {
                ret = 1u;
                break;
            }
            addr += 2u;
        }
    }

    HAL_FLASH_Lock();
    return ret;
}
