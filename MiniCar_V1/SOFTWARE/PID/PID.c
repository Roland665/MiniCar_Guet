#include "PID.h"
#include "stdlib.h"


/**
 * @brief    创建一个位置式PID对象，在使用 PID_Position 函数前调用
 * @param    kp             : 比例系数
 * @param    ki             : 积分系数
 * @param    kd             : 微分系数
 * @param    result_Max     : 输出限幅
 * @param    intergral_Max  : 积分限幅
 * @retval   PID对象
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
 * @brief    创建一个增量式PID对象，在使用 PID_Increasing 函数前调用
 * @param    kp: 比例系数
 * @param    ki: 积分系数
 * @param    kd: 微分系数
 * @retval   PID对象
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
 * @brief    位置式PID算法
 * @param    pid: PID对象，里面应当存储了计算必要的数据
 * @param    err_new:最新的差值
 * @retval   输出PID结果
 */
float PID_Position(PID* pid, float err_new){
    float result;
    //更新pid对象内容
    pid->err_old_first = pid->err_new;
    pid->err_new = err_new;
    pid->intergral += pid->err_new;
    //积分限幅
    pid->intergral = pid->intergral > pid->intergral_Max ? pid->intergral_Max : pid->intergral; //控制积分上限幅
    pid->intergral = pid->intergral <-pid->intergral_Max/2 ?-pid->intergral_Max/2 : pid->intergral; //控制积分下限幅
    //计算pid
    result = pid->kp*pid->err_new + pid->ki*pid->intergral + pid->kd*(pid->err_new - pid->err_old_first);
    //输出限幅
    result = result > pid->result_Max ? pid->result_Max : result; //控制输出上限幅
    result = result <-pid->result_Max ?-pid->result_Max : result; //控制输出下限幅
    //输出pid结果
    return result;
}


/**
 * @brief    增量式myPID算法
 * @param    pid: PID对象，里面应当存储了计算必要的数据
 * @param    err_new:最新的差值
 * @param    result_inc_Max: 输出限幅
 * @retval   输出的增量
 */

float PID_Increasing(PID* pid, float err_new, float result_inc_Max){
    float result_inc;
    //更新pid对象内容
    pid->err_old_second = pid->err_old_first;
    pid->err_old_first = pid->err_new;
    pid->err_new = err_new;
    pid->result_inc_Max = result_inc_Max;
    //计算pid
    result_inc = pid->kp*pid->err_new - pid->ki*pid->err_old_first + pid->kd*pid->err_old_second;
    //输出限幅
    result_inc = result_inc > pid->result_inc_Max ? pid->result_inc_Max : result_inc;  //控制输出上限幅
    result_inc = result_inc <-pid->result_inc_Max ?-pid->result_inc_Max : result_inc; //控制输出下限幅
    //输出pid结果
    return result_inc;
}
