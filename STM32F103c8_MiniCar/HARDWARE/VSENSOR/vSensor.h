#ifndef __VSENSOR_H
#define __VSENSOR_H			  	 
#include "sys.h"

#define vSensorCountMax 1000
#define PulseNumOfOneCircle  20 			//码盘转一圈的脉冲数
#define TIRERATIUS 65						//车轮半径(毫米)
#define CIRCUMFERENCE 2*3.14*TIRERATIUS		//车轮周长
#define ONEPULSEDISTANCE CIRCUMFERENCE/20	//码盘一个脉冲代表车走过的距离
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
