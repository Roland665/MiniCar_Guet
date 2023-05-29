#include "metal_detection.h"
#include "exti.h"


void Metal_Detection_Init(void)
{
	//EXTI1_Init();
	GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(METAL_DET_Clock, ENABLE);	 		//使能端口时钟

	GPIO_InitStructure.GPIO_Pin = METAL_DET_Pin;	    		//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	//下拉输入
	GPIO_Init(METAL_DET_GPIO, &GPIO_InitStructure);	  		//根据设定参数初始化
}
