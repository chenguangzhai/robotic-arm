#include "Servo_motor.h"
#include "tim.h"
#include <stdint.h>
#include <sys/_intsup.h>
#include "model.h"
#include "math.h"
#include "OLED.h"

#define PI 3.1415926

/* 每个舵机的物理角度范围 [min, max]，单位度
 *   Num: 1 底座 / 2 肩(大臂) / 3 肘(小臂) / 4 腕 / 5 夹爪 / 6 夹爪
 *   舵机3(肘)物理限制为 45~180，其余均为 0~180 */
static const uint16_t SERVO_MIN[Motor_Num] = {  0,   0,  45,   0,   0,   0};
static const uint16_t SERVO_MAX[Motor_Num] = {180, 180, 180, 180, 180, 180};

/* 机械臂各段长度，单位米（唯一来源，逆运动学 model.c 直接读取）：
 * l0 底座立柱 / l1 肩→肘 / l2 肘→腕 / l3 腕→末端 */
length mech_length = {0.08, 0.10, 0.09, 0.17};

Servo_motor servo_motors[Motor_Num];

/**
  * 函    数:选择舵机并设置脉宽
  * 参    数Num:舵机编号1~6
  * 参    数angle: 舵机脉宽500~2500 
  * 返 回 值:无
  * 说    明:此函数无需外部调用
  */
static void SetAngle(uint8_t Num, uint16_t angle)
{
    if(angle < 500 || angle > 2500)
    {
        return;
    }
    switch(Num)
    {
        case 1:
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, angle);
            break;
        case 2:
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, angle);
            break;
        case 3:
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, angle);
            break;
        case 4:
            __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, angle);
            break;
        case 5:
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, angle);
            break;
        case 6:
            __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, angle);
            break;
        default:
            return;
    }
}

/**
  * 函    数:舵机初始化
  * 参    数:无
  * 返 回 值:无
  * 说    明:无
  */
void Servo_motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); 
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /* 上电初始姿态：四个俯仰/偏航舵机统一取中位 90°。
     *
     * 说明：舵机上电时不知道自己停在哪，PWM 一输出就会朝对应角度转过去，
     *       这一下"弹跳"是硬件行为、无法消除。取中位是为了让弹跳幅度最小
     *       （中位离任何可能位置都不会太远），随后由 Servo_Motor_Updata
     *       以 1°/50ms 平滑过渡到 model() 解出的工作姿态。
     *
     * 早期这里写的是 90/90/135/135，肘腕离工作位置很远，
     * 上电会先猛弹到 135° 再慢慢往回走，动作幅度大且耗时长。 */
    servo_motors[0].OldAngle = servo_motors[0].NewAngle = 90;
    servo_motors[1].OldAngle = servo_motors[1].NewAngle = 90;
    servo_motors[2].OldAngle = servo_motors[2].NewAngle = 90;
    servo_motors[3].OldAngle = servo_motors[3].NewAngle = 90;
    /* 舵机5 = PA6 = 旋转爪子（绕末端轴转，改变夹持朝向）。
     * 0° 是实测的「夹爪平行于地面」姿态，保持这个值不要动——
     * 改到别的角度夹爪会歪。目前没有指令控制它，固定不动。 */
    servo_motors[4].OldAngle = servo_motors[4].NewAngle = 0;
    /* 舵机6 = PA7 = 开合收紧。0° = 张开，60° = 收紧（已实测确认）*/
    servo_motors[5].OldAngle = servo_motors[5].NewAngle = 0;

    SetAngle(1, 1500);
    SetAngle(2, 1500);
    SetAngle(3, 1500);
    SetAngle(4, 1500);
    SetAngle(5, 500);    /* 旋转爪子 0°：夹爪平行于地面 */
    SetAngle(6, 500);    /* 开合：张开 0° */
}

/**
  * 函    数:舵机角度更新到硬件
  * 参    数:无
  * 返 回 值:无
  * 说    明:此函数需放在定时器调用
  */
void Servo_Motor_Updata(void)
{
    static uint8_t i = 50;//定义多久执行一次更新舵机函数
    if(--i > 0)
    {
        return;
    }
    i = 50;

    for (uint8_t i = 0; i < Motor_Num; i ++)
    {
        if (servo_motors[i].OldAngle < servo_motors[i].NewAngle)
        {
            servo_motors[i].OldAngle ++;
            SetAngle(i + 1, (2000 * (servo_motors[i].OldAngle) / 180.0 + 500.0));
        }
        else if (servo_motors[i].OldAngle > servo_motors[i].NewAngle)
        {
            servo_motors[i].OldAngle --;
            SetAngle(i + 1, (2000 * (servo_motors[i].OldAngle) / 180.0 + 500.0));
        }
    }
}

/**
  * 函    数:舵机设置角度
  * 参    数Num: 舵机编号1~6
  * 参    数angle: 舵机角度0~180
  * 返 回 值:无
  * 说    明:无
  */
void Servo_Motor_SetAngle(uint8_t Num, uint16_t angle)
{
    if (Num == 0 || Num > Motor_Num)
    {
        return;
    }
    /* 按该舵机物理范围限幅 */
    if (angle > SERVO_MAX[Num - 1])
    {
        angle = SERVO_MAX[Num - 1];
    }
    else if (angle < SERVO_MIN[Num - 1])
    {
        angle = SERVO_MIN[Num - 1];
    }
    servo_motors[Num - 1].NewAngle = angle;
}

/**
  * 函    数:立即设置舵机角度（跳过插值）
  * 参    数:Num: 舵机编号1~6
  * 参    数:angle: 舵机角度0~180
  * 返 回 值:无
  * 说    明:直接把 OldAngle 与 NewAngle 对齐并立刻写 PWM，
  *           用于标定模式下手动调角——否则每次调 5° 要等 250ms 才到位。
  *           正常运行时不要用它，会失去 Servo_Motor_Updata 的平滑效果。
  */
void Servo_Motor_SetAngleNow(uint8_t Num, uint16_t angle)
{
    if (Num == 0 || Num > Motor_Num)
    {
        return;
    }
    if (angle > SERVO_MAX[Num - 1])
    {
        angle = SERVO_MAX[Num - 1];
    }
    else if (angle < SERVO_MIN[Num - 1])
    {
        angle = SERVO_MIN[Num - 1];
    }

    servo_motors[Num - 1].OldAngle = angle;
    servo_motors[Num - 1].NewAngle = angle;
    SetAngle(Num, (uint16_t)(2000.0 * angle / 180.0 + 500.0));
}
