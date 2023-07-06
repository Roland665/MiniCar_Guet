#ifndef __PID_H
#define __PID_H			  	 
#include "sys.h"

typedef struct{
	float kp;               //比率系数
	float ki;               //积分系数
	float kd;               //微分系数
    float intergral;        //积分值(实际就是每次err的累加)
    float err_old_second;   //上上次计算的差值(用于增量式)
    float err_old_first;    //上一次计算的差值
    float err_new;          //本次的新差值
    float result_inc_Max;   //增量式输出最大值
    float result_Max;       //位置式输出最大值
    float intergral_Max;    //位置式积分最大值
}PID;

PID* PID_Position_Create(float kp, float ki, float kd, float result_Max, float intergral_Max);
PID* PID_Increasing_Create(float kp, float ki, float kd);
float PID_Position(PID* pid, float err_new);
float PID_Increasing(PID* pid, float err_new, float result_inc_Max);
#endif  
