#ifndef __Servo_motor_H
#define __Servo_motor_H

#include "main.h"
#include <stdint.h>

#define Motor_Num 6 //舵机数量

typedef struct//舵机角度
{
    uint16_t OldAngle;//当前角度
    uint16_t NewAngle;//目标角度
}Servo_motor;

typedef struct
{
    double l0;  //底座到第二个舵机的长度
    double l1;  //第二个舵机到第三个舵机的长度
    double l2;  //第三个舵机到第四个舵机的长度
    double l3;  //第四个舵机到夹爪中心的长度
}length;//机械臂各段长度（单位：米，定义于 Servo_motor.c）

extern length mech_length;//机械臂各段长度（唯一来源，逆运动学 model() 直接读取）

extern Servo_motor servo_motors[Motor_Num];

void Servo_motor_Init(void);
void Servo_Motor_Updata(void);
void Servo_Motor_SetAngle(uint8_t Num, uint16_t angle);

/* 立即设置角度：跳过插值，直接写 PWM。仅供标定模式使用。
 * 普通控制请用 Servo_Motor_SetAngle()，否则会失去平滑运动。 */
void Servo_Motor_SetAngleNow(uint8_t Num, uint16_t angle);

void Set_Location(double x,double y,double z);

#endif
