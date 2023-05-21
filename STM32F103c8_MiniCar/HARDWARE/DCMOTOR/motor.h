#ifndef __MOTOR_H
#define __MOTOR_H			  	 
#include "sys.h"

//左电机
//左电机使能io
#define MTLEN_GPIO GPIOA
#define MTLEN_Pin GPIO_Pin_6
#define MTLEN_Clock RCC_APB2Periph_GPIOA
//左电机反转io
#define MTLDIR_GPIO GPIOA
#define MTLDIR_Pin GPIO_Pin_2
#define MTLDIR_Clock RCC_APB2Periph_GPIOA

//右电机
//右电机使能io
#define MTREN_GPIO GPIOA
#define MTREN_Pin GPIO_Pin_7
#define MTREN_Clock RCC_APB2Periph_GPIOA
//右电机正转io
#define MTRDIR_GPIO GPIOA
#define MTRDIR_Pin GPIO_Pin_3
#define MTRDIR_Clock RCC_APB2Periph_GPIOA

#define MTLEN TIM_SetCompare1//左电机pwm控制 
#define MTREN TIM_SetCompare2//右电机pwm控制

// #define MTLDIR PAout(2)//左电机反转io
// #define MTRDIR PAout(3)//右电机正转io

void MTS_Init(void);
void MTL_Init(void);
void MTR_Init(void);

#endif  
