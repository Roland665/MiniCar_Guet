#include "PID.h"
#include "stdlib.h"


/**
 * @brief    Create an pid-data object for positional PID algorithm 
 * @param    kp             : Proportionality coefficient
 * @param    ki             : Integral coefficient
 * @param    kd             : Differential coefficient
 * @param    result_Max     : Output limiting
 * @param    intergral_Max  : Integral limiting
 * @retval   pid-data object
 */
PID* PID_Position_Create(float kp, float ki, float kd, float result_Max, float intergral_Max){
    PID* object = (PID*)malloc(sizeof(PID));
    object->kp = kp;
    object->ki = ki;
    object->kd = kd;
    object->intergral = 0;
    object->err_old_first = 0;
    object->err_new = 0;
    object->result_Max = result_Max;
    object->intergral_Max = intergral_Max;
    return object;
}


/**
 * @brief    Create an pid-data object for incremental PID algorithm
 * @param    kp             : Proportionality coefficient
 * @param    ki             : Integral coefficient
 * @param    kd             : Differential coefficient
 * @retval   pid-data object
 */
PID* PID_Increasing_Create(float kp, float ki, float kd){
    PID* object = (PID*)malloc(sizeof(PID));
    object->kp = kp;
    object->ki = ki;
    object->kd = kd;
    object->intergral = 0;
    object->err_old_second = 0;
    object->err_old_first = 0;
    object->err_new = 0;
    return object;
}


/**
 * @brief    Positional PID algorithm 
 * @param    pid    : pid-data object
 * @param    err_new: the new error
 * @retval   the result of the calculation
 */
float PID_Position(PID* pid, float err_new){
    float result;
    //pid-data 迭代
    pid->err_old_first = pid->err_new;
    pid->err_new = err_new;
    pid->intergral += pid->err_new;
    //积分限幅
    pid->intergral = pid->intergral > pid->intergral_Max ? pid->intergral_Max : pid->intergral;
    pid->intergral = pid->intergral <-pid->intergral_Max/2 ?-pid->intergral_Max/2 : pid->intergral;
    //pid 计算
    result = pid->kp*pid->err_new + pid->ki*pid->intergral + pid->kd*(pid->err_new - pid->err_old_first);
    //输出限幅
    result = result > pid->result_Max ? pid->result_Max : result;
    result = result <-pid->result_Max ?-pid->result_Max : result;
    return result;
}

/**
 * @brief    Incremental PID algorithm
 * @param    pid            : pid-data object
 * @param    err_new        : the new error
 * @param    result_inc_Max : Incremental limiting
 * @retval   the result of the calculation
 */

float PID_Increasing(PID* pid, float err_new, float result_inc_Max){
    float result_inc;
    //pid-data 迭代
    pid->err_old_second = pid->err_old_first;
    pid->err_old_first = pid->err_new;
    pid->err_new = err_new;
    pid->result_inc_Max = result_inc_Max;
    //pid 计算
    result_inc = pid->kp*(pid->err_new - pid->err_old_first) + pid->ki*pid->err_new + pid->kd*(pid->err_new - 2*pid->err_old_first + pid->err_old_second);
    //增量限幅
    result_inc = result_inc > pid->result_inc_Max ? pid->result_inc_Max : result_inc; 
    result_inc = result_inc <-pid->result_inc_Max ?-pid->result_inc_Max : result_inc;
    return result_inc;
}
