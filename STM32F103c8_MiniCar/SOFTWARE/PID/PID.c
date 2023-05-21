#include "PID.h"
#include "stdlib.h"


/**
 * @brief    ����һ��PID���󣬿�����ʹ��PID_Classic����ǰ����
 * @param    kp: ����ϵ��
 * @param    ki: ����ϵ��
 * @param    kd: ΢��ϵ��
 * @retval   PID����
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
 * @brief    ����PID�㷨
 * @param    pid: PID��������Ӧ���洢�˼����Ҫ������
 * @param    err_new:���µĲ�ֵ
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
//  * @brief    ����ʽmyPID�㷨
//  * @param    newValue: �²���������
//  * @retval   �����˲����ֵ
//  */

//float PID_Increment(PID myPID, float temp_val){
//	
//	myPID.target_val = temp_val;//����Ŀ��ֵ
//	
//    myPID.err=myPID.target_val-myPID.actual_val;//����Ŀ��ֵ��ʵ��ֵ��ֵ

//	//PID�㷨ʵ��
//	float increment_val = myPID.Kp*(myPID.err - myPID.err_next) + myPID.Ki*myPID.err + myPID.Kd*(myPID.err - 2 * myPID.err_next + myPID.err_last);
//	/*�ۼ�*/
//	myPID.actual_val += increment_val;
//	/*�������*/
//	myPID.err_last = myPID.err_next;
//	myPID.err_next = myPID.err;
//	/*���ص�ǰʵ��ֵ*/
//	return myPID.actual_val;
//}
