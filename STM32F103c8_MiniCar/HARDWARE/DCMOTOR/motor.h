#ifndef __MOTOR_H
#define __MOTOR_H			  	 
#include "sys.h"

//左电机反转io
#define MTLDIR0_GPIO GPIOB
#define MTLDIR0_Pin GPIO_Pin_8
#define MTLDIR0_Clock RCC_APB2Periph_GPIOB
#define MTLDIR1_GPIO GPIOB
#define MTLDIR1_Pin GPIO_Pin_9		
#define MTLDIR1_Clock RCC_APB2Periph_GPIOB


//右电机正转io
#define MTRDIR1_GPIO GPIOB
#define MTRDIR1_Pin GPIO_Pin_0
#define MTRDIR1_Clock RCC_APB2Periph_GPIOB
#define MTRDIR0_GPIO GPIOA
#define MTRDIR0_Pin GPIO_Pin_12
#define MTRDIR0_Clock RCC_APB2Periph_GPIOA

#define MTLEN TIM_SetCompare1//左电机pwm控制 
#define MTREN TIM_SetCompare2//右电机pwm控制

#define MTLDIR0 PBout(8)//左电机反转io
#define MTLDIR1 PBout(9)//左电机正转io
#define MTRDIR1 PBout(0)//右电机正转io
#define MTRDIR0 PAout(12)//右电机反转io

void MTS_Init(void);
void MT_DirctionIO_Init(void);
void Go(void);
void Back(void);
#endif  
