#ifndef __MOTOR_H
#define __MOTOR_H			  	 
#include "sys.h"

//����
//����ʹ��io
#define MTLEN_GPIO GPIOA
#define MTLEN_Pin GPIO_Pin_6
#define MTLEN_Clock RCC_APB2Periph_GPIOA
//������תio
#define MTLDIR_GPIO GPIOA
#define MTLDIR_Pin GPIO_Pin_2
#define MTLDIR_Clock RCC_APB2Periph_GPIOA

//�ҵ��
//�ҵ��ʹ��io
#define MTREN_GPIO GPIOA
#define MTREN_Pin GPIO_Pin_7
#define MTREN_Clock RCC_APB2Periph_GPIOA
//�ҵ����תio
#define MTRDIR_GPIO GPIOA
#define MTRDIR_Pin GPIO_Pin_3
#define MTRDIR_Clock RCC_APB2Periph_GPIOA

#define MTLEN TIM_SetCompare1//����pwm���� 
#define MTREN TIM_SetCompare2//�ҵ��pwm����

// #define MTLDIR PAout(2)//������תio
// #define MTRDIR PAout(3)//�ҵ����תio

void MTS_Init(void);
void MTL_Init(void);
void MTR_Init(void);

#endif  
