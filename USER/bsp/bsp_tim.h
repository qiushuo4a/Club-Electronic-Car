#ifndef _BSP_TIM_H_
#define _BSP_TIM_H_

#include "stm32f1xx_hal.h"


#define MOTOR_SPEED_RERATIO 30    //电机减速比51
#define PULSE_PRE_ROUND 11 //一圈多少个脉冲11
#define RADIUS_OF_TYRE 52 //轮胎半径，单位毫米
#define LINE_SPEED_C RADIUS_OF_TYRE * 2 * 3.14


typedef struct __Motor_t
{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
  int speed;
    
}Motor_t;
extern Motor_t motor[2];

void Encoder_Init(void);

#endif
