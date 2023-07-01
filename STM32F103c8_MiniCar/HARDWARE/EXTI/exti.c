#include "exti.h"

void EXTI0_Init(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure; 
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能 GPIOA 时钟
	
    //GPIOA0初始化设置 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//A0口作为外部中断输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置成输入，默认下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化 GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //开启 AFIO 时钟

	/* 配置EXTI_Line0 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);//PA0 连接到中断线0
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//外部中断0
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
	
}

void EXTI1_Init(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure; 
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能 GPIOA 时钟
	
    //GPIOA0初始化设置 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//A1口作为外部中断输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置成输入，默认下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化 GPIO

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //开启 AFIO 时钟

	/* 配置EXTI_Line */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);//PA1 连接到中断线1
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE1
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE1
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;//外部中断0
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
	
}

void EXTI2_Init(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure; 
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能 GPIOA 时钟
	
    //GPIOA0初始化设置 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//A0口作为外部中断输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //设置成输入，默认下拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化 GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //开启 AFIO 时钟

	/* 配置EXTI_Line0 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);//PA0 连接到中断线0
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;//外部中断0
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
	
}

void EXTI3_Init(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure; 
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//使能 GPIO 时钟
	
    //GPIOA0初始化设置 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//外部中断输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成输入，上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化 GPIO
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //开启 AFIO 时钟

	/* 配置EXTI_Line3 */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);//连接到中断线3
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;//LINE3
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE3
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断3
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
	
}

void AllEXTI_Init(void){
	EXTI0_Init();
	EXTI1_Init();
	EXTI2_Init();
	EXTI3_Init();
}
/*

//外部中断0服务函数
void EXTI0_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line0)!=RESET){//判断某个线上的中断是否发生
		TI0CD = MilliSecond+20;
		while(TI0CD > MilliSecond){
			if(PAin(0) == 0) {			
				EXTI_ClearITPendingBit(EXTI_Line0); //清除 LINE 上的中断标志位
				return;
			}
		}
		if(PWMval[0] == 0){
			LED_Open(0);
		}
		else{
			LED_Close(0);
		}
    EXTI_ClearITPendingBit(EXTI_Line0); //清除 LINE 上的中断标志位
    }
}
//外部中断1服务函数
void EXTI1_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line1)!=RESET){//判断某个线上的中断是否发生
		TI1CD = MilliSecond+20;
		while(TI1CD > MilliSecond){
			if(PAin(1) == 0) {			
				EXTI_ClearITPendingBit(EXTI_Line1); //清除 LINE 上的中断标志位
				return;
			}
		}
		if(TI1CD <= MilliSecond){
			if(PWMval[1] == 0){
				LED_Open(1);
			}
			else{
				LED_Close(1);
			}
			TI1CD = MilliSecond+20;
		}
    EXTI_ClearITPendingBit(EXTI_Line1); //清除 LINE 上的中断标志位
    }
}
//外部中断2服务函数
void EXTI2_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line2)!=RESET){//判断某个线上的中断是否发生
		TI2CD = MilliSecond+20;
		while(TI2CD > MilliSecond){
			if(PAin(2) == 0) {			
				EXTI_ClearITPendingBit(EXTI_Line2); //清除 LINE 上的中断标志位
				return;
			}
		}
		if(TI2CD <= MilliSecond){
			if(PWMval[2] == 0){
				LED_Open(2);
			}
			else{
				LED_Close(2);
			}
			TI2CD = MilliSecond+20;
		}
    EXTI_ClearITPendingBit(EXTI_Line2); //清除 LINE 上的中断标志位
    }
}
//外部中断3服务函数
void EXTI3_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line3)!=RESET){//判断某个线上的中断是否发生
		TI3CD = MilliSecond+20;
		while(TI3CD > MilliSecond){
			if(PAin(3) == 0) {			
				EXTI_ClearITPendingBit(EXTI_Line3); //清除 LINE 上的中断标志位
				return;
			}
		}
		if(TI3CD <= MilliSecond){
			if(PWMval[3] == 0){
				LED_Open(3);
			}
			else{
				LED_Close(3);
			}
			TI3CD = MilliSecond+20;
		}
    EXTI_ClearITPendingBit(EXTI_Line3); //清除 LINE 上的中断标志位
    }
}

*/











