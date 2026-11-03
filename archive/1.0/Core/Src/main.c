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
#include "UART.h"
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

/* ===== 平滑运动参数 =====
 * XY_STEP  ：按一次键，目标坐标改变多少（米）
 * MOVE_STEP：主循环每转一圈，末端实际推进多少（米）
 *
 * 原理：按键只改「目标」，末端按 MOVE_STEP 逐毫米逼近目标，
 *       每推进一次就重算一遍逆解并立即下发。
 *       这样末端走的是直线；而旧做法是三个关节各自以固定角速度插值，
 *       先到的关节等后到的，中间轨迹就是一条弧。 */
#define XY_STEP     0.005f
#define MOVE_STEP   0.001f
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
/* 目标坐标：由按键改变 */
float x = 0.28f;
float y = 0.10f;
float z = 0.0f;

/* 当前实际推进到的坐标：主循环每轮朝目标逼近 MOVE_STEP，
 * 用它去解算，末端才能走出直线而不是弧线 */
static float cur_x = 0.28f;
static float cur_y = 0.10f;

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if CALIB_MODE
    /* 标定模式：只调舵机，不跑逆运动学 */
    Calib_Loop();
#else
    /* 按键改变目标坐标：轻点 5mm，长按连发。
     * 先试算可达性，越界的目标直接拒绝——否则目标一旦跑到空间外，
     * 要按很多次才能拉回来。 */
    {
        uint8_t k;
        k  = Key_Check(KEY_1, KEY_DOWN); k |= Key_Check(KEY_1, KEY_REPEAT);
        if (k && model_reachable(x + XY_STEP, y, z) == 0) x += XY_STEP;

        k  = Key_Check(KEY_2, KEY_DOWN); k |= Key_Check(KEY_2, KEY_REPEAT);
        if (k && model_reachable(x - XY_STEP, y, z) == 0) x -= XY_STEP;

        k  = Key_Check(KEY_3, KEY_DOWN); k |= Key_Check(KEY_3, KEY_REPEAT);
        if (k && model_reachable(x, y + XY_STEP, z) == 0) y += XY_STEP;

        k  = Key_Check(KEY_4, KEY_DOWN); k |= Key_Check(KEY_4, KEY_REPEAT);
        if (k && model_reachable(x, y - XY_STEP, z) == 0) y -= XY_STEP;
    }

    /* 末端朝目标逐毫米推进（笛卡尔空间），每步重算逆解并立即下发。
     * 这样末端走的是直线；旧做法是三个关节各按固定角速度插值，
     * 先到的关节等后到的，中间轨迹就歪成一条弧。 */
    {
        float dx = x - cur_x;
        float dy = y - cur_y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist > 1e-5f)
        {
            float step = (dist > MOVE_STEP) ? MOVE_STEP : dist;
            cur_x += dx / dist * step;
            cur_y += dy / dist * step;
        }
    }

    /* 注意：用「当前推进到的坐标」解算，而不是目标坐标 */
    uint8_t ok = model_now(cur_x, cur_y, z);

    /* 第0行：解算结果状态，固定 3 字符宽（"OK "/"OUT"），避免长字符串残留 */
    OLED_ShowString(0, 0, "RES:", OLED_6X8);
    OLED_ShowString(24, 0, ok ? "OUT" : "OK ", OLED_6X8);
    /* 第1行：当前目标坐标 x / y / z，固定宽度 */
    OLED_ShowFloatNum(0,  8, x, 1, 2, OLED_6X8);
    OLED_ShowFloatNum(32, 8, y, 1, 2, OLED_6X8);
    OLED_ShowFloatNum(64, 8, z, 1, 2, OLED_6X8);

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
     * 注意：一次扫描要跑约 80×13 次逆解、耗时几十毫秒，如果每轮都算会
     * 拖慢平滑运动（长按连发时尤其明显）。所以只在「没有任何按键按住」
     * 时才更新——松手后补一次即可。KEY_HOLD 查询不会清除标志位。 */
    static float cache_x = 999.0f, cache_y = 999.0f;
    static float x_min = 0.0f, x_max = 0.0f, y_min = 0.0f, y_max = 0.0f;

    uint8_t any_key_held = Key_Check(KEY_1, KEY_HOLD) | Key_Check(KEY_2, KEY_HOLD)
                         | Key_Check(KEY_3, KEY_HOLD) | Key_Check(KEY_4, KEY_HOLD);

    if (!any_key_held)
    {
      if (y != cache_y)
      {
        model_XRange(y, z, &x_min, &x_max);
        cache_y = y;
      }
      if (x != cache_x)
      {
        model_YRange(x, z, &y_min, &y_max);
        cache_x = x;
      }
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
