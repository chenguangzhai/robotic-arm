#include "main.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "Key.h"

#define KEY_PRESSED				1
#define KEY_UNPRESSED			0

#define KEY_TIME_DOUBLE			200
/* 长按判定时间：从 2000ms 改为 500ms。
 * 原来要按住 2 秒才开始连发，手动调坐标时手感很钝；
 * 500ms 后进入 KEY_LONG，之后每 KEY_TIME_REPEAT(100ms) 触发一次 KEY_REPEAT。 */
#define KEY_TIME_LONG			500
#define KEY_TIME_REPEAT			100

uint8_t Key_Flag[KEY_COUNT];

/* 引脚统一取自 main.h 的 CubeMX 宏（KEYx_Pin / KEYx_GPIO_Port）。
 * 这样 CubeMX 重新分配引脚后，本文件无需修改，只要重新生成即可。
 * 当前配置：KEY1=PA11  KEY2=PA12  KEY3=PA15  KEY4=PB3 */
uint8_t Key_GetState(uint8_t n)
{
	if (n == KEY_1)
	{
		if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
		{
			return KEY_PRESSED;
		}
	}
	else if (n == KEY_2)
	{
		if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
		{
			return KEY_PRESSED;
		}
	}
	else if (n == KEY_3)
	{
		if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET)
		{
			return KEY_PRESSED;
		}
	}
	else if (n == KEY_4)
	{
		if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET)
		{
			return KEY_PRESSED;
		}
	}
#ifdef KEY5_Pin
	else if (n == KEY_5)
	{
		if (HAL_GPIO_ReadPin(KEY5_GPIO_Port, KEY5_Pin) == GPIO_PIN_RESET)
		{
			return KEY_PRESSED;
		}
	}
#endif
#ifdef KEY6_Pin
	else if (n == KEY_6)
	{
		if (HAL_GPIO_ReadPin(KEY6_GPIO_Port, KEY6_Pin) == GPIO_PIN_RESET)
		{
			return KEY_PRESSED;
		}
	}
#endif
	return KEY_UNPRESSED;
}

uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
	if (Key_Flag[n] & Flag)
	{
		if (Flag != KEY_HOLD)
		{
			Key_Flag[n] &= ~Flag;
		}
		return 1;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count, i;
	static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
	static uint8_t S[KEY_COUNT];
	static uint16_t Time[KEY_COUNT];
	
	for (i = 0; i < KEY_COUNT; i ++)
	{
		if (Time[i] > 0)
		{
			Time[i] --;
		}
	}
	
	Count ++;
	if (Count >= 20)
	{
		Count = 0;
		
		for (i = 0; i < KEY_COUNT; i ++)
		{
			PrevState[i] = CurrState[i];
			CurrState[i] = Key_GetState(i);
			
			if (CurrState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_HOLD;
			}
			else
			{
				Key_Flag[i] &= ~KEY_HOLD;
			}
			
			if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED)
			{
				Key_Flag[i] |= KEY_DOWN;
			}
			
			if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED)
			{
				Key_Flag[i] |= KEY_UP;
			}
			
			if (S[i] == 0)
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Time[i] = KEY_TIME_LONG;
					S[i] = 1;
				}
			}
			else if (S[i] == 1)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					Time[i] = KEY_TIME_DOUBLE;
					S[i] = 2;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_TIME_REPEAT;
					Key_Flag[i] |= KEY_LONG;
					S[i] = 4;
				}
			}
			else if (S[i] == 2)
			{
				if (CurrState[i] == KEY_PRESSED)
				{
					Key_Flag[i] |= KEY_DOUBLE;
					S[i] = 3;
				}
				else if (Time[i] == 0)
				{
					Key_Flag[i] |= KEY_SINGLE;
					S[i] = 0;
				}
			}
			else if (S[i] == 3)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
			}
			else if (S[i] == 4)
			{
				if (CurrState[i] == KEY_UNPRESSED)
				{
					S[i] = 0;
				}
				else if (Time[i] == 0)
				{
					Time[i] = KEY_TIME_REPEAT;
					Key_Flag[i] |= KEY_REPEAT;
					S[i] = 4;
				}
			}
		}
	}
}
