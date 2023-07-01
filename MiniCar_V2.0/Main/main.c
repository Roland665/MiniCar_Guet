#include "sys.h"
#include "usart/usart.h"
#include "freeRTOS.h"
#include "task.h"
#include "LED/led.h"
#include "HC_SR04/hc_sr04.h"
#include "CAP/myCap.h"
#include "Timer/myTimer.h"

/**************************** 任务定义区 ********************************/
//AppCreate 任务栈深
#define AppCreate_Task_Stack_Deep 128
//AppCreate 任务堆栈
StackType_t AppCreate_Task_Stack[AppCreate_Task_Stack_Deep];
//AppCreate 任务句柄
TaskHandle_t AppCreate_Task_Handle;
//AppCreate 任务函数
void AppCreate_Task(void *pvParameters);

//LED 任务栈深
#define LED_Task_Stack_Deep 128
//LED 任务堆栈
StackType_t LED_Task_Stack[LED_Task_Stack_Deep];
//LED 任务句柄
TaskHandle_t LED_Task_Handle;
//LED 任务函数
void LED_Task(void *pvParameters);

//LED2 任务栈深
#define LED2_Task_Stack_Deep 128
//LED2 任务堆栈
StackType_t LED2_Task_Stack[LED2_Task_Stack_Deep];
//LED2 任务句柄
TaskHandle_t LED2_Task_Handle;
//LED2 任务函数
void LED2_Task(void *pvParameters);

//Ranging 任务栈深
#define Ranging_Task_Stack_Deep 128
//Ranging 任务堆栈
StackType_t Ranging_Task_Stack[Ranging_Task_Stack_Deep];
//Ranging 任务句柄
TaskHandle_t Ranging_Task_Handle;
//Ranging 任务函数
void Ranging_Task(void *pvParameters);


void setup(void) //串口0初始化
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //设置系统时钟为80MHz
	LED_Init();
	Usart0_Init(115200);
	HC_SR04_Init();
	HC_SR04_Start();
    Time0A_Init(80-1);//系统频率为80Mhz，80000000/80=1us,实现us级中断
}

int main(void)
{
	setup(); // 初始化
    printf("\r\nProgram is run-begining\r\n系统时钟频率为%u\r\n",SysCtlClockGet());
   /* 创建 AppTaskCreate 任务 */
	xTaskCreate((TaskFunction_t)	AppCreate_Task,       		// 任务函数
				(const char *)		"AppCreate_Task",       	// 任务名称
				(uint32_t)			AppCreate_Task_Stack_Deep,  // 任务堆栈大小
				(void *)			NULL,                       // 传递给任务函数的参数
				(UBaseType_t)		3,                      	// 任务优先级
				(TaskHandle_t *)	AppCreate_Task_Handle);		// 任务句柄

   printf("=====准备进入FreeRTOS!=====\r\n");
   vTaskStartScheduler(); /* 开启调度器 */
    while (1);//理论上执行不到这句话
}

void AppCreate_Task(void *pvParameters)
{
   taskENTER_CRITICAL(); //进入临界区
   //创建 LED_Task 任务
   xTaskCreate((TaskFunction_t)	LED_Task,				// 任务函数
               (const char *)	"LED_Task",				// 任务名称
               (uint16_t)		LED_Task_Stack_Deep,	// 任务堆栈大小
               (void *)			NULL,					// 传递给任务函数的参数
               (UBaseType_t)	3,						// 任务优先级
               (TaskHandle_t *)	&LED_Task_Handle);		// 任务句柄
			   
   //创建 LED2_Task 任务
   xTaskCreate((TaskFunction_t)	LED2_Task,				// 任务函数
               (const char *)	"LED2_Task",				// 任务名称
               (uint16_t)		LED2_Task_Stack_Deep,	// 任务堆栈大小
               (void *)			NULL,					// 传递给任务函数的参数
               (UBaseType_t)	3,						// 任务优先级
               (TaskHandle_t *)	&LED2_Task_Handle);		// 任务句柄

   vTaskDelete(AppCreate_Task_Handle); //删除开始任务 (2)
   taskEXIT_CRITICAL();            //退出临界区
}




void LED_Task(void *pvParameters)
{
   while (1)
   {
       LED0_RGB_R_ENABLE;
       vTaskDelay(1000);
       LED0_RGB_R_UNABLE;
       LED0_RGB_G_ENABLE;
       vTaskDelay(1000);
       LED0_RGB_G_UNABLE;
       LED0_RGB_B_ENABLE;
       vTaskDelay(1000);
       LED0_RGB_B_UNABLE;
   }
}


void LED2_Task(void *pvParameters)
{
   while (1)
   {
       LED0_RGB_R_ENABLE;
       vTaskDelay(200);
       LED0_RGB_R_UNABLE;
       LED0_RGB_G_ENABLE;
       vTaskDelay(200);
       LED0_RGB_G_UNABLE;
       LED0_RGB_B_ENABLE;
       vTaskDelay(200);
       LED0_RGB_B_UNABLE;
   }
}



u16 T2CCP0_STA = 0; //输入捕获状态 bit15表示是否完成一次脉冲捕获，bit14表示是否完成脉冲第一次变化沿，bit13~bit0表示脉冲持续时间(T2CCP0_STA++语句触发周期)
//T2CCP0的中断服务函数
void TIMER2A_Handler(void){
	TimerIntClear(TIMER2_BASE, TIMER_CAPA_MATCH);
    if(T2CCP0_STA & 0x4000){
        //捕获过一个上升沿，这次是下降沿来了
        T2CCP0_STA |= 0x8000;//标记完成一次高电平脉冲捕获
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); //重新设置为上升沿捕获 
        T2CCP0_STA &= ~0x4000;//置零    
        printf("完成一次脉冲捕获~计数次数为%d\r\n",T2CCP0_STA & 0x3FFF);
    }
    else{
        //第一次捕获上升沿
        //清空，开始计时等待下降沿
        T2CCP0_STA = 0;
        T2CCP0_STA |= 0X4000;//标记捕获到了上升沿
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE); //设置为下降沿捕获 
    }
}

void TIMER0A_Handler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//HC-SR04
	if((T2CCP0_STA & 0X8000)==0 && (T2CCP0_STA & 0X4000)){
		//还未完成捕获，但是已经捕获到高电平了
		if((T2CCP0_STA&0X3FFF) == 0X3FFF){
			//高电平太长了(持续时间大于16383*1us)
			T2CCP0_STA |= 0xFFFF;
		}
		else T2CCP0_STA++;
	}
}