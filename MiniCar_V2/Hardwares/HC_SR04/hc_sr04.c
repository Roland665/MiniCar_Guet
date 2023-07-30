#include "hc_sr04.h"
#include "PWM/myPWM.h"
#include "CAP/myCap.h"
/**
  * @brief    HC_SR04 模块初始化，其实做的就是初始化PWM控制Trig引脚和初始化一个cap捕获Echo引脚的脉冲
  * @param    void
  * @retval   void
  */
void HC_SR04_Init(void){
	M1PWM4_Init(SYSCTL_PWMDIV_64, 0xFFFF, 50);//PWM周期为65535拉满，脉冲宽度为10us  10/10^6/80Mhz = 800
	
	/*这里需要特殊执行整个输入捕获的初始化，因为TO的模块A已经正在使用了*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // 启用GPIOF作为脉冲捕捉脚   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // 配置GPIO脚为使用Timer0捕捉模式   
    GPIOPinConfigure(GPIO_PF1_T0CCP1);   
    GPIOPinTypeTimer(GPIOF_BASE, GPIO_PIN_1); 
    // 为管脚配置弱上拉模式（捕获下降沿，配置为上拉）
    GPIOPadConfigSet(GPIOF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  
    // 配置使用Timer的TimerB模块为边沿触发模式|加计数模式
	// 请前往定时器0A初始化地方增加 TIMER_CFG_B_CAP_TIME_UP
    // 使用上升沿触发
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE); 
    // 设置计时范围
    TimerLoadSet(TIMER0_BASE, TIMER_B, 0xFFFF);	
    //设置中断优先级
	MAP_IntPrioritySet(INT_TIMER0B, 2);
    // 定时器中断允许，中断事件为Capture模式中边沿触发，计数到达预设值   
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
    // NVIC中允许定时器模块中断   
    MAP_IntEnable(INT_TIMER0B);
    // 启动捕捉模块
    TimerEnable(TIMER0_BASE, TIMER_B);
}

/**
  * @brief    启动 HC_SR04 模块(未验证)
  * @param    void
  * @retval   void
  */
void HC_SR04_Start(void){
	//使能PWM发生器
	PWMGenEnable(PWM1_BASE,PWM_GEN_2);
    //使能用于cap的定时器2模块A
    TimerEnable(TIMER0_BASE, TIMER_B);
}

/**
  * @brief    关闭 HC_SR04 模块(未验证)
  * @param    void
  * @retval   void
  */
void HC_SR04_Stop(void){
	//失能PWM发生器
	PWMGenDisable(PWM1_BASE,PWM_GEN_2);
    //失能用于cap的定时器2模块A
    TimerDisable(TIMER0_BASE, TIMER_B);
}
