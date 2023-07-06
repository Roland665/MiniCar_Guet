#include "sys.h"
#include "usart/usart.h"
#include "freeRTOS.h"
#include "task.h"
#include "LED/led.h"
#include "HC_SR04/hc_sr04.h"
#include "CAP/myCap.h"
#include "Timer/myTimer.h"
#include "Filter/Filters.h"
#include "MPU6050/mpu6050.h"
#include "PID/PID.h"

/*******************************************/
/*
@version V2.0
相较于V1系列
  - 使用无刷电机配合舵机驱动
  - 核心mcu换为TM4C123GH6

实现了
  - 
*/

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

//MPU6050_Sensor 任务栈深
#define MPU6050_Sensor_Task_Stack_Deep 128
//MPU6050_Sensor 任务堆栈
StackType_t MPU6050_Sensor_Task_Stack[MPU6050_Sensor_Task_Stack_Deep];
//MPU6050_Sensor 任务句柄
TaskHandle_t MPU6050_Sensor_Task_Handle;
//MPU6050_Sensor 任务函数
void MPU6050_Sensor_Task(void *pvParameters);

/**************************** 全局变量定义区 ********************************/
float distance = 0;//小车与前方物体间距离(单位cm)
u16 T2CCP0_STA = 0; //输入捕获状态 bit15表示是否完成一次脉冲捕获，bit14表示是否完成脉冲第一次变化沿，bit13~bit0表示脉冲持续时间(T2CCP0_STA++语句触发周期)

void setup(void) //串口0初始化
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //设置系统时钟为80MHz
	LED_Init();
	Usart0_Init(115200);
	HC_SR04_Init();
	HC_SR04_Start();
	Time0A_Init(800-1);//系统频率为80Mhz，800/80000000=10us,实现10us级中断
#ifdef MPU6050
    mpu6050_Init();
#endif
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

   //创建 Ranging_Task 任务
   xTaskCreate((TaskFunction_t)	Ranging_Task,				// 任务函数
               (const char *)	"Ranging_Task",				// 任务名称
               (uint16_t)		Ranging_Task_Stack_Deep,	// 任务堆栈大小
               (void *)			NULL,					// 传递给任务函数的参数
               (UBaseType_t)	3,						// 任务优先级
               (TaskHandle_t *)	&Ranging_Task_Handle);		// 任务句柄

#ifdef MPU6050
   //创建 MPU6050_Sensor_Task 任务
   xTaskCreate((TaskFunction_t)	MPU6050_Sensor_Task,				// 任务函数
               (const char *)	"MPU6050_Sensor_Task",				// 任务名称
               (uint16_t)		MPU6050_Sensor_Task_Stack_Deep,	// 任务堆栈大小
               (void *)			NULL,					// 传递给任务函数的参数
               (UBaseType_t)	3,						// 任务优先级
               (TaskHandle_t *)	&MPU6050_Sensor_Task_Handle);		// 任务句柄
#endif

   vTaskDelete(AppCreate_Task_Handle); //删除开始任务 (2)
   taskEXIT_CRITICAL();            //退出临界区
}

#ifdef MPU6050
/**
  * @brief    MPU6050_Sensor_Task 任务函数(未成品)
  * @brief    通过对MPU6050的数据进行融合计算，得出小车旋转角度
  */
void MPU6050_Sensor_Task(void *pvParameters){
    mpu6050_t mympu6050;
    while(1){
        printf("accx=%d  accy=%d  accz=%d\r\n",mympu6050.accX,mympu6050.accY,mympu6050.accZ);
	    printf("gyroX=%d  gyroY=%d  gyroZ=%d\r\n",mympu6050.gyroX,mympu6050.gyroY,mympu6050.gyroZ);
        mpu6050_Get_Data(&mympu6050);
		vTaskDelay(1000);
    }
}
#endif

/**
  * @brief    Ranging_Task 任务函数
  * @brief    通过计算 T2CCP0 捕获的脉冲长度得到小车与前方物体间距离
  */
void Ranging_Task(void *pvParameters){
    u16 ultrasonicTimes = 0;
    while(1){
        if(T2CCP0_STA&0x8000){
            //完成一次上升沿脉冲捕获
			ultrasonicTimes = T2CCP0_STA&0x3FFF;
			T2CCP0_STA = 0;
			distance = Filter(ultrasonicTimes*0.17, distance, 0.8);
			printf("Time=%hu, distance=%.1fcm\r\n",ultrasonicTimes,distance);
        }
    }
}

/**
  * @brief    LED_Task 任务函数
  * @brief    闪烁灯，体现程序正常跑了
  */
void LED_Task(void *pvParameters){
   while (1)
   {
       LED0_RGB_R_ENABLE;
       vTaskDelay(1000);
       LED0_RGB_R_DISABLE;
       LED0_RGB_G_ENABLE;
       vTaskDelay(1000);
       LED0_RGB_G_DISABLE;
       LED0_RGB_B_ENABLE;
       vTaskDelay(1000);
       LED0_RGB_B_DISABLE;
   }
}

/**
  * @brief    LED2_Task 任务函数
  * @brief    闪烁灯，体现程序正常跑了
  */
void LED2_Task(void *pvParameters){
   while (1)
   {
        LED0_RGB_R_ENABLE;
        vTaskDelay(200);
        LED0_RGB_R_DISABLE;
        LED0_RGB_G_ENABLE;
        vTaskDelay(200);
        LED0_RGB_G_DISABLE;
        LED0_RGB_B_ENABLE;
        vTaskDelay(200);
        LED0_RGB_B_DISABLE;
   }
}


//T2CCP0的中断服务函数
void TIMER2A_Handler(void){
    //清除中断标志位
	TimerIntClear( TIMER2_BASE,  TimerIntStatus( TIMER2_BASE,  true));
    if(T2CCP0_STA & 0x4000){
        //捕获过一个上升沿，这次是下降沿来了
        T2CCP0_STA |= 0x8000;//标记完成一次高电平脉冲捕获
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); //重新设置为上升沿捕获 
        T2CCP0_STA &= ~0x4000;//置零
    }
    else{
        //第一次捕获上升沿
        //清空，开始计时等待下降沿
        T2CCP0_STA = 0;
        T2CCP0_STA |= 0x4000;//标记捕获到了上升沿
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE); //设置为下降沿捕获 
    }
}


//10us级定时器，实现系统计时
void TIMER0A_Handler(void){
    static u16 usecond = 0;
    static u16 msecond = 0;
    static u8 TrigFlag = 0;//当TrigFlag为1时，HC_SR04_TRIG输出高电平并将TrigFlag置0，
                           //当TrigFlag为0时，HC_SR04_TRIG输出低电平并将TrigFlag置2，
                           //当TrigFlag为2时，等待TrigFlag重新被置0
                           //以上三个判断每10us只会判断生效一个
    static u16 TrigFlagCD = 100;//当TrigFlagCD等于100时，TrigFlag置1(实现的是每100ms获取超声波测距信息)
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //ms级计时
    if(usecond == 100){
        usecond = 0;
        //经过了1ms

        if(TrigFlagCD < 100)
            TrigFlagCD++;
        else{
            //激活一次超声波测距模块
            TrigFlag = 1;
            TrigFlagCD = 0;
        }
        if(msecond == 1000){
            msecond = 0;
            //经过了1s

        }
        msecond++;
    }

	//HC-SR04
	if((T2CCP0_STA & 0X8000)==0 && (T2CCP0_STA & 0X4000)){
		//还未完成捕获，但是已经捕获到高电平了
		if((T2CCP0_STA&0X3FFF) >= 0x3FFF){
			//高电平太长了(持续时间大于700*10us)(即距离太远了，就不测了。注意，距离小于5cm也会导致很长的高电平时间，所以这个模块的最短测距大致限制在了5cn)
			T2CCP0_STA |= 0xFFFF;
		}
		else T2CCP0_STA++;
	}
    if(TrigFlag == 1){
        HC_SR04_TRIG_ENABLE;
		TrigFlag = 0;
	}
    else if(TrigFlag == 0){
        HC_SR04_TRIG_DISABLE;
        TrigFlag = 2;
	}

    usecond++;
}
