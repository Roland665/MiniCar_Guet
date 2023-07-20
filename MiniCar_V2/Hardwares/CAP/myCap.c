#include "myCap.h"
#include "HC_SR04/hc_sr04.h"
/**
  * @brief    初始化定时器2的模块A作为输入捕获
  * @brief    https://blog.51cto.com/u_15887260/5876654（这篇文章对于输入捕获很有参考价值）
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

    //设置引脚方向，注意虽然这是输入捕获，但是这是由外界的PWM信号控制，属于硬件控制，要设置为GPIO_DIR_MODE_HW
//	GPIODirModeSet( GPIOF_BASE,  GPIO_PIN_6, GPIO_DIR_MODE_IN);
    
    // 为管脚配置弱上拉模式（捕获下降沿，配置为上拉）
   GPIOPadConfigSet(GPIOF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  

    // 配置使用Timer2的TimerA模块为边沿触发模式|加计数模式
    TimerConfigure(TIMER2_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
   
    // 使用上升沿触发
    TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); 

    // 设置计时范围
    TimerLoadSet(TIMER2_BASE, TIMER_A, 0xFFFF);	
   
    // 注册中断处理函数以响应触发事件   
//  TimerIntRegister(TIMER2_BASE, TIMER_A, (void (*)(void))TIMER2A_IRQn);//启动文件中注册完了，不用注册

    //设置中断优先级
	MAP_IntPrioritySet(INT_TIMER2A,  1 );
   
    // 定时器中断允许，中断事件为Capture模式中边沿触发，计数到达预设值   
    TimerIntEnable(TIMER2_BASE, TIMER_CAPA_EVENT);
   
    // NVIC中允许定时器2A模块中断   
    MAP_IntEnable(INT_TIMER2A);
   
    // 使能系统总中断
    MAP_IntMasterEnable();

    // 启动捕捉模块
    TimerEnable(TIMER2_BASE, TIMER_A);
}


/**
  * @brief    初始化定时器3的模块A作为输入捕获
  * @param    void
  * @retval   void
  */
void T3CCP0_Init(void){
    // 启用Timer2模块   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
   
    // 启用GPIO_F作为脉冲捕捉脚   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
   
    // 配置GPIO脚为使用Timer2捕捉模式   
    GPIOPinConfigure(GPIO_PB2_T3CCP0);   
    GPIOPinTypeTimer(GPIOB_BASE, GPIO_PIN_2); 

    //设置引脚方向，注意虽然这是输入捕获，但是这是由外界的PWM信号控制，属于硬件控制，要设置为GPIO_DIR_MODE_HW
//	GPIODirModeSet( GPIOF_BASE,  GPIO_PIN_6, GPIO_DIR_MODE_IN);
    
    // 为管脚配置弱上拉模式（捕获下降沿，配置为上拉）
    GPIOPadConfigSet(GPIOB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  

    // 配置使用Timer3的TimerA模块为边沿触发模式|加计数模式
    TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
   
    // 使用上升沿触发
    TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); 

    // 设置计时范围
    TimerLoadSet(TIMER3_BASE, TIMER_A, 0xFFFF);	
   
    // 注册中断处理函数以响应触发事件   
//  TimerIntRegister(TIMER2_BASE, TIMER_A, (void (*)(void))TIMER2A_IRQn);//启动文件中注册完了，不用注册

    //设置中断优先级
	MAP_IntPrioritySet(INT_TIMER3A,  1 );
   
    // 定时器中断允许，中断事件为Capture模式中边沿触发，计数到达预设值   
    TimerIntEnable(TIMER3_BASE, TIMER_CAPA_EVENT);
   
    // NVIC中允许定时器3A模块中断   
    MAP_IntEnable(INT_TIMER3A);
   
    // 使能系统总中断
    MAP_IntMasterEnable();

    // 启动捕捉模块
    TimerEnable(TIMER3_BASE, TIMER_A);
}
