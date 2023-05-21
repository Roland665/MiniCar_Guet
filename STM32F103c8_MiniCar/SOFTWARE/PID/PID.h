#ifndef __PID_H
#define __PID_H			  	 
#include "sys.h"

typedef struct{
	float kp;           //比率系数
	float ki;           //积分系数
	float kd;           //微分系数
    float intergral;    //积分值(实际就是每次err的累加)
    float err_old;      //上一次计算的差值
    float err_new;      //本次的新差值
}PID;

float PID_Classic(PID* pid, float err_new);
PID* PID_Create_Object(float kp, float ki, float kd);
#endif  
