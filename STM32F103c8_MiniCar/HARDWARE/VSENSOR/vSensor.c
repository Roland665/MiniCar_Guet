#include "vSensor.h"

void vSensors_Init(){
	vSensorL_Init();
	vSensorR_Init();
}

void vSensorL_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(VSENSORL_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = VSENSORL_Pin;	    		//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	//��������
	GPIO_Init(VSENSORL_GPIO, &GPIO_InitStructure);	  		//�����趨������ʼ��
}

void vSensorR_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(VSENSORR_Clock, ENABLE);	 		//ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = VSENSORR_Pin;	    		//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	//��������
	GPIO_Init(VSENSORR_GPIO, &GPIO_InitStructure);	  		//�����趨������ʼ��
}
