#include "PWM/myPWM.h"
#include "HC_SR04/hc_sr04.h"

/**
  * @brief    PWM模块0通道0初始化
  * @param    PWM_CLKDIV    :pwm时钟分频器(对系统时钟分频)
  * @param    period    :pwm周期(单位是1 系统时钟周期*分频系数)
  * @param	  width     :pwm脉冲宽度
  * @retval   void
  */
void M0PWM0_Init(u32 PWM_CLKDIV, u16 period){
	//配置PWM时钟（设置USEPWM_CLKDIV分频器）
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//使能时钟
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);			//使能PWM模块1时钟																	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	//使能GPIO时钟	
	
	//使能引脚复用PWM功能
	GPIOPinTypePWM(GPIOB_BASE,GPIO_PIN_6);

	//PWM信号分配
	GPIOPinConfigure(GPIO_PB6_M0PWM0);					//PB4->PWM模块0信号2																						

	//配置PWM发生器
	//模块0->发生器1->下计数，不同步
	PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	
    
	//配置PWM周期
	/*
	N = (1 / f) * SysClk。
	其中N是函数参数，即PWM周期(以系统时间为单位),最大为65535, 也可以看做计数器重装载值
	f为期望频率，
	SysClk为系统时钟频率。
	*/
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0, period);			//pwm周期													
	//使能发生器0
    PWMGenEnable(PWM0_BASE,PWM_GEN_0);
	//使能PWM通道0输出
	PWMOutputState(PWM0_BASE,PWM_OUT_0_BIT,true);

}

/**
  * @brief    PWM模块0通道1初始化
  * @param    PWM_CLKDIV    :pwm时钟分频器(对系统时钟分频)
  * @param    period    :pwm周期(单位是1 系统时钟周期*分频系数)
  * @param	  width     :pwm脉冲宽度
  * @retval   void
  */
void M0PWM1_Init(u32 PWM_CLKDIV, u16 period){
	//配置PWM时钟（设置USEPWM_CLKDIV分频器）
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//使能时钟
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);			//使能PWM模块1时钟																	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	//使能GPIO时钟	
	
	//使能引脚复用PWM功能
	GPIOPinTypePWM(GPIOB_BASE,GPIO_PIN_7);

	//PWM信号分配
	GPIOPinConfigure(GPIO_PB7_M0PWM1);					//PB4->PWM模块0信号2																						

	//配置PWM发生器
	//模块0->发生器1->下计数，立即更新
	PWMGenConfigure(PWM0_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	

	//配置PWM周期
	/*
	N = (1 / f) * SysClk。
	其中N是函数参数，即PWM周期(以系统时间为单位),最大为65535, 也可以看做计数器重装载值
	f为期望频率，
	SysClk为系统时钟频率。
	*/
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_0, period);			//pwm周期
	//使能发生器0
    PWMGenEnable(PWM0_BASE,PWM_GEN_0);

	//使能PWM通道1
	PWMOutputState(PWM0_BASE,PWM_OUT_1_BIT,true);
											
}


/**
  * @brief    PWM模块0通道2初始化
  * @param    PWM_CLKDIV    :pwm时钟分频器(对系统时钟分频)
  * @param    period    :pwm周期(单位是1 系统时钟周期*分频系数)
  * @param	  width     :pwm脉冲宽度
  * @retval   void
  */
void M0PWM2_Init(u32 PWM_CLKDIV, u16 period, u16 width){
	//配置PWM时钟（设置分频器）
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//使能时钟
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);			//使能PWM模块0时钟																	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);	    //使能GPIO时钟	
	
	//使能引脚复用PWM功能
	GPIOPinTypePWM(GPIOB_BASE, GPIO_PIN_4);

	//PWM信号分配
	GPIOPinConfigure(GPIO_PB4_M0PWM2);					//PB4->PWM模块0信号2																						

	//配置PWM发生器
	//模块0->发生器1->下计数，不同步
	PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	
    
	//配置PWM周期
	/*
	N = (1 / f) * SysClk。
	其中N是函数参数，即PWM周期(以系统时间为单位),最大为65535, 也可以看做计数器重装载值
	f为期望频率，
	SysClk为系统时钟频率。
	*/
	PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1, period);			//pwm周期													

	//配置PWM占空比
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width);		//脉冲长度为10us(也可以用上面的公式)0.00001*80000000 = 800

	//使能PWM模块0输出
	PWMOutputState(PWM0_BASE,PWM_OUT_2_BIT,true);
	PWMGenEnable(PWM0_BASE,PWM_GEN_1);

}


/**
  * @brief    PWM模块1通道4初始化
  * @param    PWM_CLKDIV    :pwm时钟分频系数(对系统时钟分频)
  * @param    period        :pwm周期(单位是1 系统时钟周期*分频系数)
  * @param	  width         :pwm脉冲宽度
  * @retval   void
  */
#include "inc/hw_gpio.h"
void M1PWM4_Init(u32 PWM_CLKDIV, u16 period, u16 width){
	//配置PWM时钟（设置分频器）
	SysCtlPWMClockSet(PWM_CLKDIV);
	
	//使能时钟
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);			//使能PWM模块时钟																	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	    //使能GPIO时钟	
	
	//解锁PF0
	HWREG(GPIOF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIOF_BASE + GPIO_O_CR)	= GPIO_PIN_0;
	//使能引脚复用PWM功能
	GPIOPinTypePWM(GPIOF_BASE, GPIO_PIN_0);

	//PWM信号分配
	GPIOPinConfigure(GPIO_PF0_M1PWM4);																						

	//配置PWM发生器
	//模块1->发生器2->下计数，不同步
	PWMGenConfigure(PWM1_BASE,PWM_GEN_2,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);	
    
	//配置PWM周期
	/*
	N = (1 / f) * SysClk。
	其中N是函数参数，即PWM周期(以系统时间为单位),最大为65535, 也可以看做计数器重装载值
	f为期望频率，
	SysClk为系统时钟频率。
	*/
	PWMGenPeriodSet(PWM1_BASE,PWM_GEN_2, period);			//pwm周期													

	//配置PWM占空比
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, width);		//脉冲长度为10us(也可以用上面的公式)0.00001*80000000 = 800

	//使能PWM输出
	PWMGenEnable(PWM1_BASE,PWM_GEN_2);
	PWMOutputState(PWM1_BASE,PWM_OUT_4_BIT,true);

}
