#include "Timer/myTimer.h"
/**
  * @brief    初始化定时器0实现us级中断,作为系统计时
  * @param    reloadValue   ：重装载值
  *                       ps：计数频率默认是系统时钟频率，系统时钟频率默认是80Mhz,
  *                           所以重装载值的计算方法为
  *                           reloadValue = 80000000/(1/time) - 1)
  *                           (目标溢出时间为time，单位是秒)
  * @retval   void
  */

void Time0A_Init(u16 reloadValue){
    //使能定时器TIMER0，16/32bit
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    //配置定时器，将定时器拆分，并配置拆分后的定时器A为周期性计数
	TimerConfigure( TIMER0_BASE,  TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC_UP);
    
    //设置定时器A装载值
	TimerLoadSet( TIMER0_BASE,  TIMER_A, reloadValue);
    
    //为定时器A注册中断函数
//	TimerIntRegister( TIMER0_BASE,  TIMER_A, TIMER_IRQHandler);//启动文件中注册完了，不用注册

    //设置中断优先级
	MAP_IntPrioritySet( INT_TIMER0A,  1);

    //定时器中断允许，中断事件为计数器计数溢出
	TimerIntEnable(TIMER0_BASE,  TIMER_TIMA_TIMEOUT);

    // NVIC中允许定时器0A模块中断  
	MAP_IntEnable(INT_TIMER0A);

    // 使能系统总中断
    MAP_IntMasterEnable();

    //使能定时器
	TimerEnable( TIMER0_BASE,  TIMER_A);
}