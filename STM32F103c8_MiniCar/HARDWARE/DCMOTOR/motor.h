#ifndef __MOTOR_H
#define __MOTOR_H			  	 
#include "sys.h"

//������תio
#define MTLDIR0_GPIO GPIOB
#define MTLDIR0_Pin GPIO_Pin_8
#define MTLDIR0_Clock RCC_APB2Periph_GPIOB
#define MTLDIR1_GPIO GPIOB
#define MTLDIR1_Pin GPIO_Pin_9		
#define MTLDIR1_Clock RCC_APB2Periph_GPIOB


//�ҵ����תio
#define MTRDIR1_GPIO GPIOB
#define MTRDIR1_Pin GPIO_Pin_0
#define MTRDIR1_Clock RCC_APB2Periph_GPIOB
#define MTRDIR0_GPIO GPIOA
#define MTRDIR0_Pin GPIO_Pin_12
#define MTRDIR0_Clock RCC_APB2Periph_GPIOA

#define MTLEN TIM_SetCompare1//����pwm���� 
#define MTREN TIM_SetCompare2//�ҵ��pwm����

#define MTLDIR0 PBout(8)//������תio
#define MTLDIR1 PBout(9)//������תio
#define MTRDIR1 PBout(0)//�ҵ����תio
#define MTRDIR0 PAout(12)//�ҵ����תio

void MTS_Init(void);
void MT_DirctionIO_Init(void);
void Go(void);
void Back(void);
#endif  
