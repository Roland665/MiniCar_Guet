#ifndef __VSENSOR_H
#define __VSENSOR_H			  	 
#include "sys.h"

#define vSensorCountMax 1000
#define PulseNumOfOneCircle  20 //码盘转一圈的脉冲数
#define TIRERATIUS 0.065//轮胎半径
#define CIRCUMFERENCE 2*3.14*TIRERATIUS
#define ONEPULSEDISTANCE CIRCUMFERENCE/20
//左电机测速传感器
#define VSENSORL_GPIO GPIOA
#define VSENSORL_Pin GPIO_Pin_4
#define VSENSORL_Clock RCC_APB2Periph_GPIOA

//右电机测速传感器
#define VSENSORR_GPIO GPIOA
#define VSENSORR_Pin GPIO_Pin_5
#define VSENSORR_Clock RCC_APB2Periph_GPIOA

void vSensors_Init(void);
void vSensorL_Init(void);
void vSensorR_Init(void);
#endif  
