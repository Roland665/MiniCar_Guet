#include "track.h"

/**
  * @brief 循迹模块io初始化
  * @param    
  * @retval    
  */
void track_Init(){
    GPIO_InitTypeDef  GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 		//使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 	//下拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);	  		//根据设定参数初始化
}
