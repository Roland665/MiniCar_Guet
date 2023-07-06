#include "hc_sr04.h"
#include "timer.h"
void HC_SR04_Init(void){

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(HC_SR04_Clock, ENABLE);	 		//使能端口时钟

	GPIO_InitStructure.GPIO_Pin = HC_SR04_Pin;				//LED1端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(HC_SR04_GPIO, &GPIO_InitStructure);			//根据设定参数初始化
	GPIO_ResetBits(HC_SR04_GPIO,HC_SR04_Pin);					//LED1输出高

    TIM1_Init(10, 72 - 1);//计数频率为72M/72 = 1Mhz(捕获时间精度为1us),10us一次更新中断
}
