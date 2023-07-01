#include "hc_sr04.h"
#include "PWM/myPWM.h"
#include "CAP/myCap.h"
/**
  * @brief    HC_SR04 模块初始化，其实做的就是初始化PWM控制Trig引脚和初始化一个cap捕获Echo引脚的脉冲
  * @param    void
  * @retval   void
  */
void HC_SR04_Init(void){
	M0PWM2_Init(65535, 800);//PWM周期为65535拉满，脉冲宽度为10us  10/10^6/80Mhz = 800
    T2CCP0_Init();
}

/**
  * @brief    启动 HC_SR04 模块(未验证)
  * @param    void
  * @retval   void
  */
void HC_SR04_Start(void){
	//使能PWM发生器
	PWMGenEnable(PWM0_BASE,PWM_GEN_1);
    //使能用于cap的定时器2模块A
    TimerEnable(TIMER2_BASE, TIMER_A);
}

/**
  * @brief    关闭 HC_SR04 模块(未验证)
  * @param    void
  * @retval   void
  */
void HC_SR04_Stop(void){
	//失能PWM发生器
	PWMGenDisable(PWM0_BASE,PWM_GEN_1);
    //失能用于cap的定时器2模块A
    TimerDisable(TIMER2_BASE, TIMER_A);
}
