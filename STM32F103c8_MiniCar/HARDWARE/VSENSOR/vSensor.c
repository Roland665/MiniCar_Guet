#include "vSensor.h"

void vSensors_Init(){
	vSensorL_Init();
	vSensorR_Init();
}

void vSensorL_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(VSENSORL_Clock, ENABLE);	 		//使能端口时钟

	GPIO_InitStructure.GPIO_Pin = VSENSORL_Pin;	    		//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	//下拉输入
	GPIO_Init(VSENSORL_GPIO, &GPIO_InitStructure);	  		//根据设定参数初始化
}

void vSensorR_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(VSENSORR_Clock, ENABLE);	 		//使能端口时钟

	GPIO_InitStructure.GPIO_Pin = VSENSORR_Pin;	    		//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	//下拉输入
	GPIO_Init(VSENSORR_GPIO, &GPIO_InitStructure);	  		//根据设定参数初始化
}


void dSensors_Init(){
	dSensorL_Init();
	dSensorR_Init();
}

void dSensorL_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(DSENSORL_Clock, ENABLE);	 		//使能端口时钟

	GPIO_InitStructure.GPIO_Pin = DSENSORL_Pin;	    		//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	//下拉输入
	GPIO_Init(DSENSORL_GPIO, &GPIO_InitStructure);	  		//根据设定参数初始化
}

void dSensorR_Init(){
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(DSENSORR_Clock, ENABLE);	 		//使能端口时钟

	GPIO_InitStructure.GPIO_Pin = DSENSORR_Pin;	    		//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	//下拉输入
	GPIO_Init(DSENSORR_GPIO, &GPIO_InitStructure);	  		//根据设定参数初始化
}
