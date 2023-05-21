#include "motor.h"
#include "timer.h"

void MTS_Init(void){
	//在50ms周期下，占空比不宜低于30
	TIM3_PWM_Init(100-1,7200-1);//72M/7200=10Khz的计数频率,重装载值100，所以PWM频率为 10000hz/500=20hz.(周期为0.05s)
}

void MTL_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MTLEN_Clock, ENABLE);	 		//使能端口时钟
	
	GPIO_InitStructure.GPIO_Pin = MTLEN_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTLEN_GPIO, &GPIO_InitStructure);			//根据设定参数初始化

	RCC_APB2PeriphClockCmd(MTLDIR_Clock, ENABLE);	 		//使能端口时钟
	
	GPIO_InitStructure.GPIO_Pin = MTLDIR_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTLDIR_GPIO, &GPIO_InitStructure);			//根据设定参数初始化
	//MTLEN = 0;//停止旋转
}

void MTR_Init(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MTREN_Clock, ENABLE);	 		//使能端口时钟
	
	GPIO_InitStructure.GPIO_Pin = MTREN_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTREN_GPIO, &GPIO_InitStructure);			//根据设定参数初始化

	RCC_APB2PeriphClockCmd(MTRDIR_Clock, ENABLE);	 		//使能端口时钟
	
	GPIO_InitStructure.GPIO_Pin = MTRDIR_Pin;				//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//IO口速度为50MHz
	GPIO_Init(MTRDIR_GPIO, &GPIO_InitStructure);			//根据设定参数初始化
	//MTREN = 0;//停止旋转
}


/**

  * @brief    左转一点
  * @param    
  * @retval    
  */

void Turn_Letf(){
	
}
