
//内环；速度环；外环；位置环
//任务：对电机进行串级角度控制
//可用条件：电机实时角度、电机实时转速、可以控制电机电流大小
//外环目标值：需要电机达到的角度
//外环反馈值：电机的实时角度
//内环反馈值：电机的实时速度
//输出值：电机电流大小
//分析：外环负责电机角度控制，根据电机目标角度和反馈角度计算出目标转速；内环负责转速控制，根据速度反馈和目标转速计算出电流


/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include <math.h>

pid_t pid_speed[2];

#define ABS(x)		((x>0)? (x): (-x)) 

void abs_limit(float *a, float ABS_MAX)   
{
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX; 
}
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
													pid_t *pid, 
													uint32_t mode,
													uint32_t maxout,
													uint32_t intergral_limit,
													float 	kp, 
													float 	ki, 
													float 	kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set)
{
        pid->get[NOW] = get;
        pid->set[NOW] = set;
        pid->err[NOW] = set - get;	//set - measure
        if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
        return 0;
        if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

        if(pid->pid_mode == POSITION_PID) //位置式p
        {
            pid->pout = pid->p * pid->err[NOW];
            pid->iout += pid->i * pid->err[NOW];           
            pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );            
            abs_limit(&(pid->iout), pid->IntegralLimit);           
            pid->pos_out = pid->pout + pid->iout + pid->dout;          
            abs_limit(&(pid->pos_out), pid->MaxOutput);          
            pid->last_pos_out = pid->pos_out;	//update last time 
        }
        else if(pid->pid_mode == DELTA_PID)//增量式P
        {
            pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
            pid->iout = pid->i * pid->err[NOW];
            pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
            abs_limit(&(pid->iout), pid->IntegralLimit);
            pid->delta_u = pid->pout + pid->iout + pid->dout;
            pid->delta_out = pid->last_delta_out + pid->delta_u;
            abs_limit(&(pid->delta_out), pid->MaxOutput);
            pid->last_delta_out = pid->delta_out;	//update last time
        }

        pid->err[LLAST] = pid->err[LAST];
        pid->err[LAST] = pid->err[NOW];
        pid->get[LLAST] = pid->get[LAST];
        pid->get[LAST] = pid->get[NOW];
        pid->set[LLAST] = pid->set[LAST];
        pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;

}


void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;	
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
}


void PID_int(void)
{
	for (int i = 0;i<2;i++)
	{
		PID_struct_init(&pid_speed[i],  POSITION_PID,200, 200,2.6f, 0.000f,  0.0f);
	}
}

