#include "PID.h"
#include "stdlib.h"


/**
 * @brief    创建一个PID对象，可以在使用PID_Classic函数前调用
 * @param    kp: 比例系数
 * @param    ki: 积分系数
 * @param    kd: 微分系数
 * @retval   PID对象
 */
PID* PID_Create_Object(float kp, float ki, float kd){
    PID* object = (PID*)malloc(sizeof(PID));
    object->kp = kp;
    object->ki = ki;
    object->kd = kd;
    object->intergral = 0;
    object->err_old = 0;
    object->err_new = 0;
    return object;
}


/**
 * @brief    经典PID算法
 * @param    pid: PID对象，里面应当存储了计算必要的数据
 * @param    err_new:最新的差值
 * @retval   
 */
float PID_Classic(PID* pid, float err_new){
    float result;
    pid->err_new = err_new;
    pid->intergral += pid->err_new;
    result = pid->kp*pid->err_new + pid->ki*pid->intergral + pid->kd*(pid->err_new - pid->err_old);
    pid->err_old = pid->err_new;
    return result;
}


///**
//  * @brief    增量式myPID算法
//  * @param    newValue: 新采样的数据
//  * @retval   本次滤波输出值
//  */

//float PID_Increment(PID myPID, float temp_val){
//	
//	myPID.target_val = temp_val;//传入目标值
//	
//    myPID.err=myPID.target_val-myPID.actual_val;//计算目标值与实际值差值

//	//PID算法实现
//	float increment_val = myPID.Kp*(myPID.err - myPID.err_next) + myPID.Ki*myPID.err + myPID.Kd*(myPID.err - 2 * myPID.err_next + myPID.err_last);
//	/*累加*/
//	myPID.actual_val += increment_val;
//	/*传递误差*/
//	myPID.err_last = myPID.err_next;
//	myPID.err_next = myPID.err;
//	/*返回当前实际值*/
//	return myPID.actual_val;
//}
