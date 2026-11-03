/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "OLED.h"
#include "Servo_motor.h"
#include "model.h"
#include "Key.h"
#include "LED.h"
#include "uart_app.h"
#include "protocol.h"
#include "motion.h"
#include "play.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ===== 舵机标定模式 =====
 * 1 = 开启：按键单独调每一个舵机，用于标定「舵机角 → 实际物理角」的映射
 * 0 = 正常：跑逆运动学
 * 标定流程见 Calib_Loop() 上方注释。 */
#define CALIB_MODE  0

/* ===== 按键步长 =====
 * XY_STEP：按一次键，目标坐标改变多少（米）
 *
 * 末端实际怎么推进不在这里定了，全部交给 motion.c：
 * 按时间（巡航速度，米/秒，默认 MOTION_SPEED_DEFAULT）而不是按主循环圈数推进，
 * 这样 OLED 刷新和区间扫描拖慢主循环时，末端速度也不会跟着抖。 */
#define XY_STEP     0.005f

/* ===== 按键触发回放（3.0）=====
 * 1 = 允许用【KEY_3 + KEY_4 同时按住】来启动/停止回放
 * 0 = 关闭，回放只能用串口的 P / X
 *
 * 为什么是这两个键的组合，而不是单击某个键：
 *   四个键已经分别占给了 x+/x-/y+/y-，单击要留给微调，不能挪用。
 *   所以只能找一个"正常操作绝不会做"的动作。
 *
 * 为什么不用长按：KEY_LONG 和微调控件的长按连发是同一个事件，
 *   长按 KEY_4 会先连续微调 y- 再触发回放，手臂已经跑出去一段了。
 *
 * 代价（已知，可接受）：组合是分两次按下的，中间至少隔一个 20ms 扫描周期，
 *   所以先按下的那个键会先产生一次 5mm 微调（按住超过 500ms 还会进入连发）。
 *   实际无影响——回放下行的是【绝对角度】，第一个点立刻会把这丁点偏移纠正回来。
 *   想彻底消掉只能加第 5 个独立按键（Key 模块已支持 KEY_5，但要改 .ioc 和接线）。 */
#define PLAY_KEY_ENABLE     1

/* 按键启动回放时的循环遍数：0 = 无限循环 */
#define PLAY_KEY_ROUNDS     0u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 目标坐标和当前推进点现在都由 motion.c 持有（Motion_GetTarget /
 * Motion_GetCurrent 读取），main.c 不再自己维护一份副本——
 * 两份状态一旦不同步，按键、串口、OLED 显示就会互相打架。
 * 初始位姿也挪到了 motion.h 的 MOTION_START_X/Y/Z。 */

#if CALIB_MODE
/**
 * 舵机标定模式
 *
 * 目的：测出每个舵机的「指令角 → 实际物理角」关系。
 *       实物装配的零位和方向未必和 model.c 的假设一致，不标定则逆解全错。
 *
 * 按键：KEY1 切换舵机(1~4)   KEY2 +5°   KEY3 -5°   KEY4 复位到 90°
 *
 * 标定步骤（不需要量角器，目视对齐即可）：
 *   1. 选舵机 2（肩），用 KEY2/KEY3 调到【大臂水平前伸】，记下角度 → A1_H
 *   2. 选舵机 3（肘），调到【小臂与大臂成一条直线（完全伸直）】，记下角度 → A2_S
 *   3. 选舵机 4（腕），调到【夹爪与小臂成一条直线】，记下角度 → A3_S
 *   4. 再把大臂调到【竖直向上】记 A1_V，可用来验证方向（A1_V 应比 A1_H 小 90）
 *
 * 三个基准值拿到后，映射关系就能定：
 *   th1 = (A1_H - a1) * 方向  ;  th2 = (a2 - A2_S) ;  th3 = (a3 - A3_S)
 * 把实测值填进 model.c 对应位置即可。
 */
static void Calib_Loop(void)
{
    static uint8_t  sel = 2;      /* 默认选中舵机2（肩），最常需要标定 */
    static uint16_t ang = 90;

    if (Key_Check(KEY_1, KEY_DOWN))
    {
        sel = (sel >= 4) ? 1 : (uint8_t)(sel + 1);
        ang = servo_motors[sel - 1].NewAngle;   /* 切换时读该舵机当前值 */
    }
    if (Key_Check(KEY_2, KEY_DOWN))
    {
        ang = (ang + 5 > 180) ? 180 : (uint16_t)(ang + 5);
        Servo_Motor_SetAngleNow(sel, ang);
    }
    if (Key_Check(KEY_3, KEY_DOWN))
    {
        ang = (ang < 5) ? 0 : (uint16_t)(ang - 5);
        Servo_Motor_SetAngleNow(sel, ang);
    }
    if (Key_Check(KEY_4, KEY_DOWN))
    {
        ang = 90;
        Servo_Motor_SetAngleNow(sel, ang);
    }

    OLED_ShowString(0, 0,  "CALIB MODE", OLED_6X8);
    OLED_ShowString(0, 16, "SERVO:", OLED_6X8);
    OLED_ShowNum(42, 16, sel, 1, OLED_6X8);
    OLED_ShowString(0, 24, "ANGLE:", OLED_6X8);
    OLED_ShowNum(42, 24, ang, 3, OLED_6X8);
    OLED_ShowString(0, 40, "K1:sel K2:+ K3:-", OLED_6X8);
    OLED_ShowString(0, 48, "K4:reset 90", OLED_6X8);
    OLED_Update();
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  HAL_TIM_Base_Start_IT(&htim4); //开启定时器4中断

  Servo_motor_Init();//舵机初始化

  /* 运动模块：建立目标/当前位置状态并把舵机摆到初始位姿。
   * 必须在 Servo_motor_Init() 之后（它内部会调 model_now 驱动舵机）。 */
  Motion_Init();

  /* 示教模块：只把 Flash 里的点表读进 RAM，【不动舵机】。
   * 必须放在 Motion_Init() 之后——上电姿态由前两者决定，
   * 如果这里顺手恢复上次示教的姿态，上电会先弹一下再回工作位。 */
  Play_Init();

  /* 启动串口收发（USART1 = PA9/PA10，115200，DMA 循环接收）。
   * 必须放在 MX_USART1_UART_Init() 之后，且 MX_DMA_Init() 已先执行。 */
  Uart_Init(&huart1);

  /* 协议层：把串口字节流切成一行一条命令。必须在 Uart_Init() 之后。
   * 支持的指令：M x y z（米）/ G（本轮回 NOTIMPL）/ ?（查状态），详见 protocol.h。 */
  Protocol_Init();

  /* 上电自检回执：主动往外吐一行，用来分清「串口没通」和「命令没解析」。
   *
   * 调试串口时的用法：先连上位机，再按一下板子上的复位键。
   *   看到 READY 2.0 → 固件是这份、TX 通路正常，问题只可能在 RX 方向
   *   看不到        → 要么烧的不是这份固件，要么 TX 接线/驱动有问题
   *
   * 只在上电时发一次，不会周期性刷屏。 */
  Protocol_Reply("READY 3.0");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 串口指令解析：非阻塞，没数据时开销只有一次 Uart_Read。
     * 放在这里（而不是 if/else 分支里）是为了标定模式下也能查状态。
     * 它只调 Motion_SetTarget()，运动仍由下面的 Motion_Tick() 统一推进，
     * 所以串口和按键两条输入通道共用同一条执行路径。 */
    Protocol_Poll();

#if CALIB_MODE
    /* 标定模式：只调舵机，不跑逆运动学 */
    Calib_Loop();
#else
    /* 目标坐标由 motion.c 持有，这里只取一份快照用于显示和按键增量 */
    float tx, ty, tz;

    /* 按键改变目标：轻点 5mm，长按连发。
     * 每次改之前重新取一次目标，这样同时按两个键时两个增量都会生效，
     * 而不是后一个把前一个覆盖掉。
     * Motion_SetTarget 内部做 数值范围 → 终点可达 → 路径可达 三重校验，
     * 越界的目标直接被拒，不会把机械臂带偏。
     *
     * 注意：KEY_DOWN 和 KEY_REPEAT 必须【两个都读】。Key_Check 的语义是
     * 「有标志就返回 1 并清掉」（KEY_HOLD 除外），用 || 短路的话
     * KEY_DOWN 命中时 KEY_REPEAT 不会被读走，那个标志会滞留在 Key_Flag 里，
     * 下一轮再冒出来多走一步。
     *
     * 回放期间【照读不照做】：回放和按键改的是同一批舵机，两边同时下发，
     * 手臂会在两个控制律之间来回拉（串口那边 protocol.c 已用 ERR:BUSY 挡了 M）。
     * 但标志仍然要读走 —— 不读的话 Key_Flag 会一直锁存，回放一结束
     * 四个键的 DOWN 标志一次性全触发，手臂朝某个方向连跳好几步。 */

    {
        uint8_t k;
        uint8_t go = (Play_IsPlaying() == 0u) ? 1u : 0u;

#if PLAY_KEY_ENABLE
        /* ---- 按键启停回放：KEY_3 + KEY_4 同时按住 ----
         *
         * 用 KEY_HOLD 拼组合，而不是 KEY_DOWN：HOLD 是电平量，
         * Key_Check 读它【不会清除】（见 Key.c 里 `if (Flag != KEY_HOLD)`），
         * 所以每轮都能重新查一次，凑出「两个键同时按着」这个状态。
         * DOWN 是边沿量、读一次就没了，没法用来判断组合。
         *
         * 自己记 combo_prev 做边沿检测，不能直接靠 KEY_DOWN：
         * HOLD 按着不动就一直为 1，不做边沿的话会每 20ms 翻转一次启停。
         */
        {
            static uint8_t combo_prev = 0u;
            uint8_t        combo_now;

            combo_now = ((Key_Check(KEY_3, KEY_HOLD) != 0u) &&
                         (Key_Check(KEY_4, KEY_HOLD) != 0u)) ? 1u : 0u;

            if ((combo_now != 0u) && (combo_prev == 0u))
            {
                /* 把这两键本轮的下按/连发吃掉，否则同一次按键会顺带再微调一步 */
                (void)Key_Check(KEY_3, KEY_DOWN);
                (void)Key_Check(KEY_3, KEY_REPEAT);
                (void)Key_Check(KEY_4, KEY_DOWN);
                (void)Key_Check(KEY_4, KEY_REPEAT);

                if (Play_IsPlaying())
                {
                    Play_Stop();            /* 停在当前姿态，不回原位 */
                }
                else if (Play_Count() == 0u)
                {
                    /* 没有点可放（Flash 里没存过，或者刚发过 C）。
                     * 什么都不做——自动跳到「记录」是另一种误操作。 */
                }
                else if (Motion_IsBusy())
                {
                    /* 还在走一条 M，等它到位再说：两条控制律同时写舵机会抢。
                     * 用户会看到 OLED 仍是 MOV，松手再按一次即可。 */
                }
                else
                {
                    (void)Play_Start((uint8_t)PLAY_KEY_ROUNDS);
                }
            }
            combo_prev = combo_now;

            /* 两键按着的时候一律不做微调。
             * 不加这一句的话，按住组合键等手臂停下的那段时间里，
             * KEY_REPEAT 会每 100ms 送一次微调，手臂一边等一边挪。 */
            if (combo_now != 0u)
            {
                go = 0u;
            }
        }
#endif

        k  = Key_Check(KEY_1, KEY_DOWN); k |= Key_Check(KEY_1, KEY_REPEAT);
        if (go && k) { Motion_GetTarget(&tx, &ty, &tz); (void)Motion_SetTarget(tx + XY_STEP, ty, tz); }

        k  = Key_Check(KEY_2, KEY_DOWN); k |= Key_Check(KEY_2, KEY_REPEAT);
        if (go && k) { Motion_GetTarget(&tx, &ty, &tz); (void)Motion_SetTarget(tx - XY_STEP, ty, tz); }

        k  = Key_Check(KEY_3, KEY_DOWN); k |= Key_Check(KEY_3, KEY_REPEAT);
        if (go && k) { Motion_GetTarget(&tx, &ty, &tz); (void)Motion_SetTarget(tx, ty + XY_STEP, tz); }

        k  = Key_Check(KEY_4, KEY_DOWN); k |= Key_Check(KEY_4, KEY_REPEAT);
        if (go && k) { Motion_GetTarget(&tx, &ty, &tz); (void)Motion_SetTarget(tx, ty - XY_STEP, tz); }
    }

    /* 按【时间】推进末端，与主循环快慢无关。
     * 每攒够 0.5mm 走一步、重算一次逆解并立即下发，所以末端走的是直线。 */
    Motion_Tick();

    /* 示教回放：自己按关节做插值，不经过 motion.c。
     * 两者互斥（回放期间协议层拒绝 M），所以不会同时抢舵机。 */
    Play_Tick();

    Motion_GetTarget(&tx, &ty, &tz);

    /* 第0行：运动状态，固定 3 字符宽（"OK "/"MOV"/"OUT"/"PAT"/"RNG"/"PLY"），
     * 避免长字符串残留。
     *   MOV = 正在走    OUT = 上次目标不可达
     *   PAT = 上次路径中途穿出可达区    RNG = 上次坐标超范围
     *   PLY = 正在回放（右边显示第几个点）
     *   OK  = 已到位 */
    {
        const char *st;
        uint8_t err = Motion_GetLastError();

        if (Play_IsPlaying())           { st = "PLY"; }
        else if (Motion_IsBusy())       { st = "MOV"; }
        else if (err == MOTION_ERR_OUT) { st = "OUT"; }
        else if (err == MOTION_ERR_PATH){ st = "PAT"; }
        else if (err == MOTION_ERR_PARAM){ st = "RNG"; }
        else                            { st = "OK "; }

        OLED_ShowString(0, 0, "RES:", OLED_6X8);
        OLED_ShowString(24, 0, st, OLED_6X8);

        /* 回放时把「第几个点」补在后面，示教调试时不用一直盯着串口。
         * 三个数字位固定宽度，退出回放后不会留下残影。 */
        if (Play_IsPlaying())
        {
            OLED_ShowNum(42, 0, Play_Index(), 3, OLED_6X8);
        }
        else
        {
            OLED_ShowString(42, 0, "   ", OLED_6X8);
        }
    }
    /* 第1行：当前目标坐标 x / y / z，固定宽度 */
    OLED_ShowFloatNum(0,  8, tx, 1, 2, OLED_6X8);
    OLED_ShowFloatNum(32, 8, ty, 1, 2, OLED_6X8);
    OLED_ShowFloatNum(64, 8, tz, 1, 2, OLED_6X8);

    /* 第2~3行：四个舵机的目标角度（逆解下发值），固定 3 位数字
     * B = 底座(舵机1)  S = 肩(舵机2)  E = 肘(舵机3)  W = 腕(舵机4)
     * 读的是 NewAngle（解算目标值），用于判断哪一路被判定超出行程。
     * 注意：已放弃姿态约束，W 不再由外部指定，而是自动取让夹爪尽量水平的值。 */
    OLED_ShowString(0, 16, "B:", OLED_6X8);
    OLED_ShowNum(12, 16, servo_motors[0].NewAngle, 3, OLED_6X8);
    OLED_ShowString(36, 16, "S:", OLED_6X8);
    OLED_ShowNum(48, 16, servo_motors[1].NewAngle, 3, OLED_6X8);

    OLED_ShowString(0, 24, "E:", OLED_6X8);
    OLED_ShowNum(12, 24, servo_motors[2].NewAngle, 3, OLED_6X8);
    OLED_ShowString(36, 24, "W:", OLED_6X8);
    OLED_ShowNum(48, 24, servo_motors[3].NewAngle, 3, OLED_6X8);  
    /* 第4~7行：当前数据下 x / y 各自的有效区间。
     * 三连杆的腕点必须落在肩前 1~19cm 的环内（l1=10cm, l2=9cm），
     * 末端再靠 l3=17cm 的工具段伸出去，所以末端可达范围比环带大得多。
     * 区间与另一个轴是耦合的，所以 y 变了要重算 x 区间，x 变了要重算 y 区间。
     *
     * 注意：一次扫描要跑约 80×13 次逆解、耗时几十毫秒。只在【完全静止】
     * 时更新——运动或回放中扫描会把 Tick 的节奏打乱，
     * 末端速度出现肉眼可见的顿挫。等停下来再补一次即可。 */
    static float cache_x = 999.0f, cache_y = 999.0f, cache_z = 999.0f;
    static float x_min = 0.0f, x_max = 0.0f, y_min = 0.0f, y_max = 0.0f;

    if ((!Motion_IsBusy()) && (!Play_IsPlaying()))
    {
      if ((ty != cache_y) || (tz != cache_z))
      {
        model_XRange(ty, tz, &x_min, &x_max);
        cache_y = ty;
      }
      if ((tx != cache_x) || (tz != cache_z))
      {
        model_YRange(tx, tz, &y_min, &y_max);
        cache_x = tx;
      }
      cache_z = tz;
    }

    OLED_ShowString(0, 32, "xmin:", OLED_6X8);
    OLED_ShowFloatNum(30, 32, x_min, 1, 3, OLED_6X8);
    OLED_ShowString(0, 40, "xmax:", OLED_6X8);
    OLED_ShowFloatNum(30, 40, x_max, 1, 3, OLED_6X8);
    OLED_ShowString(0, 48, "ymin:", OLED_6X8);
    OLED_ShowFloatNum(30, 48, y_min, 1, 3, OLED_6X8);
    OLED_ShowString(0, 56, "ymax:", OLED_6X8);
    OLED_ShowFloatNum(30, 56, y_max, 1, 3, OLED_6X8);

    OLED_Update();
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
