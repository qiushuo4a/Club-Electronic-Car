#include "tim.h"
#include "bsp_tim.h"
#include "gpio.h"
#include "motor.h"
#include "pid.h"
#include "bsp_uart.h"
Motor_t motor[2];

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);       //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);            //开启编码器定时器更新中断,防溢出处理
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);       //开启编码器定时器
    __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);            //开启编码器定时器更新中断,防溢出处理
    HAL_TIM_Base_Start_IT(&htim6);                         //开启5ms定时器中断
    HAL_TIM_Base_Start_IT(&htim7);                         //开启2ms定时器中断
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);             //电机驱动pwm
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);             //电机驱动pwm
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);             //电机驱动pwm
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);             //电机驱动pwm
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);             //开启PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); 
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); 
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);              //开启PWM
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);            
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); 
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
	__HAL_TIM_SET_COUNTER(&htim1, 30000);
	__HAL_TIM_SET_COUNTER(&htim2, 30000);
    
//	for(int i;i<5;i++)
//   {
//	motor[i].totalCount = 0;                              
//    motor[i].speed = 0;
//   }
    HAL_Delay(100);
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器回调函数，用于计算速度
{
    if(htim->Instance==htim6.Instance)//间隔定时器中断，是时候计算速度了
    {
        motor[0].totalCount= __HAL_TIM_GetCounter(&htim2);//获取计数值
        __HAL_TIM_SetCounter(&htim2, 30000);//清空计数值

		motor[0].totalCount=motor[0].totalCount - 30000;
		motor[0].speed = (float)motor[0].totalCount /11/21.3/4*60*200 ;//计算转速	
		
      motor[1].totalCount= __HAL_TIM_GetCounter(&htim1);//获取计数值    
		__HAL_TIM_SetCounter(&htim1, 30000);//清空计数值
		motor[1].totalCount=motor[1].totalCount - 30000;
		motor[1].speed = (float)motor[1].totalCount *11*21.3*4 ;//计算转速
    }
/*在motor[0].speed = (float)motor[0].totalCount /11/21.3/4*6000;中
13为旋转一圈所产生的脉冲数，即脉冲数/转(Pulse Per Revolution 或PPR)
60 是本减少直流电机的减速比
4.0是采用了双通道，几倍频就除以几
60*200是本中断5ms一次，60*200就可以得到一分钟的转速*/
if(htim->Instance==htim7.Instance)     //间隔定时器中断
{
//    if(SBUS_CH.ConnectState == 1)
//    { 
//        Chassis_movement((SBUS_CH.CH2 - 1024) ,(SBUS_CH.CH1 - 1024)/6 );//遥感值与运动速度的转换
//        for (int i = 0; i <= 2; i++)
//        {
//            pid_calc(&pid_speed[i], motor[i].speed ,130 );
//        }
//    }
//    else
//    {   
//        Chassis_movement( 0 , 0 );
//    }
//      Chassis_movement(600,400);//遥感值与运动速度的转换
        for (int i = 0; i <= 2; i++)
        {
            pid_calc(&pid_speed[i],motor[i].speed ,268 );
        }
        
        
    if(pid_speed[0].pos_out < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0 );
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, -pid_speed[0].pos_out);
    }
    else if( pid_speed[0].pos_out > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, pid_speed[0].pos_out);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0 );
    }
    else 
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, 0 );
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, 0 );
    }

        if(pid_speed[1].pos_out < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 0 );
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, -pid_speed[1].pos_out);
    }
    else if( pid_speed[1].pos_out > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, pid_speed[1].pos_out);
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 0 );
    }
    else 
    {
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, 0 );
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 0 );
    }

    }
	  
   ///舵机控制，遥控器中值为1024，摇杆由340-1704变化，按键上中下分别对应340，1024，1704，按键上下对应340，1704。

    
    
    if(SBUS_CH.CH5==340) //反转  
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,500);   

}
if(SBUS_CH.CH5==1704)   //正传
{ 
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,1500);   
   
}    

if(SBUS_CH.CH5==1024)
{
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);  //停
} 

}