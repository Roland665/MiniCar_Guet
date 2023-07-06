#ifndef __VSENSOR_H
#define __VSENSOR_H			  	 
#include "sys.h"

#define vSensorCountMax 1000
#define PulseNumOfOneCircle  20 			//����תһȦ��������
#define TIRERATIUS 65						//���ְ뾶(����)
#define CIRCUMFERENCE 2*3.14*TIRERATIUS		//�����ܳ�
#define ONEPULSEDISTANCE CIRCUMFERENCE/20	//����һ����������߹��ľ���
//�������ٴ�����
#define VSENSORL_GPIO GPIOA
#define VSENSORL_Pin GPIO_Pin_4
#define VSENSORL_Clock RCC_APB2Periph_GPIOA

//�ҵ�����ٴ�����
#define VSENSORR_GPIO GPIOA
#define VSENSORR_Pin GPIO_Pin_5
#define VSENSORR_Clock RCC_APB2Periph_GPIOA


//���������
#define DSENSORL_GPIO GPIOB
#define DSENSORL_Pin GPIO_Pin_5
#define DSENSORL_Clock RCC_APB2Periph_GPIOB

//�Ҳ�������
#define DSENSORR_GPIO GPIOB
#define DSENSORR_Pin GPIO_Pin_4
#define DSENSORR_Clock RCC_APB2Periph_GPIOB

void vSensors_Init(void);
void vSensorL_Init(void);
void vSensorR_Init(void);
void dSensors_Init(void);
void dSensorL_Init(void);
void dSensorR_Init(void);
#endif  
