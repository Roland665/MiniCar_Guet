#ifndef __PID_H
#define __PID_H			  	 
#include "sys.h"

typedef struct{
	float kp;           //比率系数
	float ki;           //积分系数
	float kd;           //微分系数
    int intergral;    //积分值(实际就是每次err的累加)
    int err_old;      //上一次计算的差值
    int err_new;      //本次的新差值
}PID;

int PID_Classic(PID* pid, int err_new);
PID* PID_Create_Object(float kp, float ki, float kd);
#endif  
