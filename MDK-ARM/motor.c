#include "math.h"
#include "motor.h"
#include "bsp_uart.h"
#include "bsp_tim.h"
#include "pid.h"
#include "tim.h"
int wheel_rpm[2];//电机目标转数
void Chassis_movement(float target_vx,float target_vw)//底盘移动
{
    float VX,VW;
    VX = target_vx;
    VW = target_vw;
    ////////////////////////////////////两轮差速运动学解算
    wheel_rpm[0] = -VX - VW;
    wheel_rpm[1] = -VX + VW;
  
}

   