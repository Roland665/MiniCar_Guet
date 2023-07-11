#include "PID.h"
#include "stdlib.h"


/**
 * @brief    ����һ��λ��ʽPID������ʹ�� PID_Position ����ǰ����
 * @param    kp             : ����ϵ��
 * @param    ki             : ����ϵ��
 * @param    kd             : ΢��ϵ��
 * @param    result_Max     : ����޷�
 * @param    intergral_Max  : �����޷�
 * @retval   PID����
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
 * @brief    ����һ������ʽPID������ʹ�� PID_Increasing ����ǰ����
 * @param    kp: ����ϵ��
 * @param    ki: ����ϵ��
 * @param    kd: ΢��ϵ��
 * @retval   PID����
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
 * @brief    λ��ʽPID�㷨
 * @param    pid: PID��������Ӧ���洢�˼����Ҫ������
 * @param    err_new:���µĲ�ֵ
 * @retval   ���PID���
 */
float PID_Position(PID* pid, float err_new){
    float result;
    //����pid��������
    pid->err_old_first = pid->err_new;
    pid->err_new = err_new;
    pid->intergral += pid->err_new;
    //�����޷�
    pid->intergral = pid->intergral > pid->intergral_Max ? pid->intergral_Max : pid->intergral; //���ƻ������޷�
    pid->intergral = pid->intergral <-pid->intergral_Max/2 ?-pid->intergral_Max/2 : pid->intergral; //���ƻ������޷�
    //����pid
    result = pid->kp*pid->err_new + pid->ki*pid->intergral + pid->kd*(pid->err_new - pid->err_old_first);
    //����޷�
    result = result > pid->result_Max ? pid->result_Max : result; //����������޷�
    result = result <-pid->result_Max ?-pid->result_Max : result; //����������޷�
    //���pid���
    return result;
}

/**
 * @brief    ����ʽPID�㷨
 * @param    pid: PID��������Ӧ���洢�˼����Ҫ������
 * @param    err_new:���µĲ�ֵ
 * @param    result_inc_Max: ����޷�
 * @retval   ���������
 */

float PID_Increasing(PID* pid, float err_new, float result_inc_Max){
    float result_inc;
    //����pid��������
    pid->err_old_second = pid->err_old_first;
    pid->err_old_first = pid->err_new;
    pid->err_new = err_new;
    pid->result_inc_Max = result_inc_Max;
    //����pid
    result_inc = pid->kp*(pid->err_new - pid->err_old_first) + pid->ki*pid->err_old_first + pid->kd*(pid->err_new - 2*pid->err_old_first + pid->err_old_second);
    //����޷�
    result_inc = result_inc > pid->result_inc_Max ? pid->result_inc_Max : result_inc; //����������޷�
    result_inc = result_inc <-pid->result_inc_Max ?-pid->result_inc_Max : result_inc; //����������޷�
    //���pid���
    return result_inc;
}
