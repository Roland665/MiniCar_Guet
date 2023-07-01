#include "CAP/myCap.h"
#include "HC_SR04/hc_sr04.h"

/**
  * @brief    初始化定时器2的模块A作为输入捕获
  * @param    void
  * @retval   void
  */
void T2CCP0_Init(void){
    // 启用Timer2模块   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
   
    // 启用GPIO_F作为脉冲捕捉脚   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
   
    // 配置GPIO脚为使用Timer2捕捉模式   
    GPIOPinConfigure(GPIO_PF4_T2CCP0);   
    GPIOPinTypeTimer(GPIOF_BASE, GPIO_PIN_4); 

    // 为管脚配置弱上拉模式（捕获下降沿，配置为上拉）
    GPIOPadConfigSet(GPIOF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  

    // 配置使用Timer2的TimerA模块为边沿触发模式|加计数模式
    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT_UP);
   
    // 使用上升沿触发
    TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); 

    // 设置计数范围为0~9
    TimerMatchSet(TIMER2_BASE, TIMER_A, 10-1);		//理论匹配周期10^-4*10=0.001s
   
    // 注册中断处理函数以响应触发事件   
//  TimerIntRegister(TIMER2_BASE, TIMER_A, (void (*)(void))TIMER2A_IRQn);//启动文件中注册完了，不用注册

    //设置中断优先级
	MAP_IntPrioritySet(INT_TIMER2A,  2);
   
    // 定时器中断允许，中断事件为Capture模式中边沿触发，计数到达预设值   
    TimerIntEnable(TIMER2_BASE, TIMER_CAPA_MATCH);
   
    // NVIC中允许定时器2A模块中断   
    MAP_IntEnable(INT_TIMER2A);
   
    // 使能系统总中断
    MAP_IntMasterEnable();

    // 启动捕捉模块
    TimerEnable(TIMER2_BASE, TIMER_A);
}
