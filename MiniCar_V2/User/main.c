#include "sys.h"
#include "usart/usart.h"
#include "freeRTOS.h"
#include "task.h"
#include "queue.h"
#include "LED/led.h"
#include "HC_SR04/hc_sr04.h"
#include "CAP/myCap.h"
#include "Timer/myTimer.h"
#include "Filter/Filters.h"
#include "PID/PID.h"
#include "BEEP/beep.h"
#include "MOTOR/motor.h"
#include "u8g2_user.h"
#include "Ganv_Track/Ganv_Track.h"
#include "PWM/myPWM.h"
#include "Key/key.h"
/*******************************************/
/*
@version V2.1
相较于V2.0系列
  - 几乎更换了所有驱动外部硬件的外设复用，为了方便PCB布线

实现了
  - 
*/

/**************************** 内核对象定义区 ********************************/


/**************************** 任务定义区 ********************************/
/* AppCreate 任务 */
// 任务栈深
#define AppCreate_Task_Stack_Deep 128
// 任务堆栈
StackType_t AppCreate_Task_Stack[AppCreate_Task_Stack_Deep];
// 任务句柄
TaskHandle_t AppCreate_Task_Handle;
// 任务函数
void AppCreate_Task(void *pvParameters);

/* U3RX_Analyzing 任务 */
// 任务栈深
#define U3RX_Analyzing_Task_Stack_Deep 128
// 任务堆栈
StackType_t U3RX_Analyzing_Task_Stack[U3RX_Analyzing_Task_Stack_Deep];
// 任务句柄
TaskHandle_t U3RX_Analyzing_Task_Handle;
// 任务函数
void U3RX_Analyzing_Task(void *pvParameters);

/* LED 任务 */
// 任务栈深
#define LED_Task_Stack_Deep 128
// 任务堆栈
StackType_t LED_Task_Stack[LED_Task_Stack_Deep];
// 任务句柄
TaskHandle_t LED_Task_Handle;
// 任务函数
void LED_Task(void *pvParameters);

/* LED2 任务 */
// 任务栈深
#define LED2_Task_Stack_Deep 128
// 任务堆栈
StackType_t LED2_Task_Stack[LED2_Task_Stack_Deep];
// 任务句柄
TaskHandle_t LED2_Task_Handle;
// 任务函数
void LED2_Task(void *pvParameters);

/* Ranging 任务 */
// 任务栈深
#define Ranging_Task_Stack_Deep 128
// 任务堆栈
StackType_t Ranging_Task_Stack[Ranging_Task_Stack_Deep];
// 任务句柄
TaskHandle_t Ranging_Task_Handle;
// 任务函数
void Ranging_Task(void *pvParameters);

/* Run 任务 */
// 任务栈深
#define Run_Task_Stack_Deep 128
// 任务堆栈
StackType_t Run_Task_Stack[Run_Task_Stack_Deep];
// 任务句柄
TaskHandle_t Run_Task_Handle;
// 任务函数
void Run_Task(void *pvParameters);

/* OLEDShowing 任务 */
// 任务栈深
#define OLEDShowing_Task_Stack_Deep 256
// 任务堆栈
StackType_t OLEDShowing_Task_Stack[OLEDShowing_Task_Stack_Deep];
// 任务句柄
TaskHandle_t OLEDShowing_Task_Handle;
// 任务函数
void OLEDShowing_Task(void *pvParameters);

/* SpeedDetection 任务 */
// 任务栈深
#define SpeedDetection_Task_Stack_Deep 256
// 任务堆栈
StackType_t SpeedDetection_Task_Stack[SpeedDetection_Task_Stack_Deep];
// 任务句柄
TaskHandle_t SpeedDetection_Task_Handle;
// 任务函数
void SpeedDetection_Task(void *pvParameters);

/**************************** 全局变量定义区 ********************************/
float distance = 0;//小车与前方物体间距离(单位cm)
u16 T0CCP1_STA = 0; //输入捕获状态 bit15表示是否完成一次脉冲捕获，bit14表示是否完成脉冲第一次变化沿，bit13~bit0表示脉冲持续时间(T0CCP1_STA++语句触发周期)

int16_t roll; //绕X轴旋转
int16_t pitch;//绕Y轴旋转
int16_t yaw;  //绕Z轴旋转

u8 trackState;//循迹8路数字量bit0对应OUT1~bit7对应OUT8

u8 commands[UART3_REC_LEN+1];//Zigbee接收无线命令缓冲区,commands[0]是无线命令是否堆积标志位，置一表示正在堆积

int16_t lPwmVal;// PWM 占空比
int16_t rPwmVal;// PWM 占空比

u8 carMode = 0;//小车运行模式，0表示外部控制，1表示寻迹模式，2为其他

u8 ackFlag = 0;// 应答标志位，收到应答时置一，处理完归零

u32 lPulseCounter = 0;//左轮编码器脉冲计数器
u32 rPulseCounter = 0;//右轮编码器脉冲计数器
u16 lPulseCounterTime = 0xFF;//左轮编码器脉冲计数周期
u16 rPulseCounterTime = 0xFF;//右轮编码器脉冲计数周期

float lSpeed;
float rSpeed;
float speed;
void setup(void) //串口0初始化
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //设置系统时钟为80MHz
	LED_Init();
    BEEP_Init();
	Uart0_Init(115200);//调试用
	Uart3_Init(115200);
	Uart5_Init(115200);//外挂MPU6050模块串口
	Time0A_Init(800-1);//系统频率为80Mhz，800/80000000=10us,实现10us级中断
}

int main(void)
{
	setup(); // 初始化
	int Sysclock = SysCtlClockGet();
    printf("\r\nProgram is Run-begining\r\n系统时钟频率为%u\r\n",Sysclock);
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
                                               
    //创建 U3RX_Analyzing 任务
    xTaskCreate((TaskFunction_t)	U3RX_Analyzing_Task,				// 任务函数
                (const char *)	"U3RX_Analyzing_Task",				    // 任务名称
                (uint16_t)		U3RX_Analyzing_Task_Stack_Deep,	    // 任务堆栈大小
                (void *)			NULL,					// 传递给任务函数的参数
                (UBaseType_t)	3,						    // 任务优先级
                (TaskHandle_t *)	&U3RX_Analyzing_Task_Handle);		// 任务句柄
    
    //创建 LED_Task 任务
    xTaskCreate((TaskFunction_t)	LED_Task,				// 任务函数
                (const char *)	"LED_Task",				    // 任务名称
                (uint16_t)		LED_Task_Stack_Deep,	    // 任务堆栈大小
                (void *)			NULL,					// 传递给任务函数的参数
                (UBaseType_t)	3,						    // 任务优先级
                (TaskHandle_t *)	&LED_Task_Handle);		// 任务句柄

    //创建 LED2_Task 任务
    xTaskCreate((TaskFunction_t)	LED2_Task,				// 任务函数
                (const char *)	"LED2_Task",				// 任务名称
                (uint16_t)		LED2_Task_Stack_Deep,	    // 任务堆栈大小
                (void *)			NULL,					// 传递给任务函数的参数
                (UBaseType_t)	3,						    // 任务优先级
                (TaskHandle_t *)	&LED2_Task_Handle);		// 任务句柄

    //创建 Ranging_Task 任务
    xTaskCreate((TaskFunction_t)	Ranging_Task,				// 任务函数
                (const char *)	"Ranging_Task",				    // 任务名称
                (uint16_t)		Ranging_Task_Stack_Deep,	    // 任务堆栈大小
                (void *)			NULL,					    // 传递给任务函数的参数
                (UBaseType_t)	3,						        // 任务优先级
                (TaskHandle_t *)	&Ranging_Task_Handle);		// 任务句柄

    //创建 Run_Task 任务
    xTaskCreate((TaskFunction_t)	Run_Task,				// 任务函数
                (const char *)	"Run_Task",				    // 任务名称
                (uint16_t)		Run_Task_Stack_Deep,	    // 任务堆栈大小
                (void *)			NULL,					// 传递给任务函数的参数
                (UBaseType_t)	3,						    // 任务优先级
                (TaskHandle_t *)	&Run_Task_Handle);		// 任务句柄

    //创建 OLEDShowing_Task 任务
    xTaskCreate((TaskFunction_t)	OLEDShowing_Task,		    // 任务函数
                (const char *)	"OLEDShowing_Task",				// 任务名称
                (uint16_t)		OLEDShowing_Task_Stack_Deep,	// 任务堆栈大小
                (void *)			NULL,					    // 传递给任务函数的参数
                (UBaseType_t)	3,						        // 任务优先级
                (TaskHandle_t *)	&OLEDShowing_Task_Handle);	// 任务句柄 

    //创建 SpeedDetection_Task 任务
    xTaskCreate((TaskFunction_t)	SpeedDetection_Task,		    // 任务函数
                (const char *)	"SpeedDetection_Task",				// 任务名称
                (uint16_t)		SpeedDetection_Task_Stack_Deep,	    // 任务堆栈大小
                (void *)			NULL,					        // 传递给任务函数的参数
                (UBaseType_t)	3,						            // 任务优先级
                (TaskHandle_t *)	&SpeedDetection_Task_Handle);	// 任务句柄 

    vTaskDelete(AppCreate_Task_Handle);//删除 AppCreate 任务
    taskEXIT_CRITICAL();//退出临界区
}

/**
  * @brief 测速任务主体
  */
static void SpeedDetection_Task(void *parameter){
    u16 PulseCounterTime_;
    u16 PulseCounter_;
    HallEncoder_Cap_Init();
    while(1){
        if(lPulseCounterTime >= 50){
            PulseCounter_ = lPulseCounter;
            lPulseCounter = 0;
            PulseCounterTime_ = lPulseCounterTime;
            lPulseCounterTime = 0;
            //理论上是每50ms计算一次当前速度
            lSpeed = PulseCounter_*ONEPULSE_FOR_DISTANCE/PulseCounterTime_;//计算出速度，单位是m/s
        }
        if(rPulseCounterTime >= 50){
            PulseCounter_ = rPulseCounter;
            rPulseCounter = 0;
            PulseCounterTime_ = rPulseCounterTime;
            rPulseCounterTime = 0;
            //理论上是每50ms计算一次当前速度
            rSpeed = PulseCounter_*ONEPULSE_FOR_DISTANCE/PulseCounterTime_;//计算出速度，单位是m/s
        }
        speed = (rSpeed+lSpeed)/2;
		if(speed > 0)   
			printf("当前车速为:%.2lfm/s,左车轮为:%.2lfm/s,右车轮为:%.2lfm/s\r\n", speed, lSpeed, rSpeed);
        vTaskDelay(30);
    }
}

/**
  * @brief OLED显示任务主体
  */
static void OLEDShowing_Task(void *parameter)
{
    char str[128] = {0}; // 需要显示的字符串临时存放处
    // 初始化u8g2
    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);     // send init sequence to the display, display is in sleep mode after this
    u8g2_SetPowerSave(&u8g2, 0); // place 1 means open power-saveing, you`ll see nothing in the screem
    while (1) {
        u8g2_ClearBuffer(&u8g2); // clear the u8g2 buffer
        if(carMode == 1){
            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf);
            sprintf(str, "%d %d %d %d %d %d %d %d", (trackState>>7)&1,
                                                    (trackState>>6)&1,
                                                    (trackState>>5)&1, 
                                                    (trackState>>4)&1, 
                                                    (trackState>>3)&1, 
                                                    (trackState>>2)&1, 
                                                    (trackState>>1)&1, 
                                                    (trackState>>0)&1);
            u8g2_DrawStr(&u8g2, 5, 37, str);
            // sprintf(str, "%d  %d  %.2lfm/s", lPwmVal, rPwmVal, speed/100);
            sprintf(str, "%d  %d  %d", pitch, roll, yaw);
            u8g2_DrawStr(&u8g2, 10, 60, str);
        }
        else{
            //手动控制模式显示动画 
            static short i,j;
            static short xTemp = 0;
            static short yTemp = 8-1;
            static u8 widTemp = 8;
            static u8 highTemp = 8;
            u8g2_SetDrawColor(&u8g2, 1); // 设置有色
            i = xTemp;
            j = 8-1;
            while(j >= yTemp){
                if(j > yTemp)
                    i = 16-1;
                else
                    i = xTemp;
                while(i >= 0){
                    u8g2_DrawBox(&u8g2, i*widTemp, j*highTemp, widTemp, highTemp); // 绘制方块
                    i--;
                }
                j--;
            }
            if(yTemp > 0 || xTemp < 16-1){
                if(xTemp < 16-1)
                    xTemp++;
                else{
                    xTemp = 0;
                    yTemp--;
                }
            }
            else{
                xTemp = 0;
                yTemp = 8-1;
                u8g2_SendBuffer(&u8g2); // 同步屏幕
                vTaskDelay(300);
                u8g2_ClearBuffer(&u8g2); // clear the u8g2 buffer
                u8g2_SendBuffer(&u8g2); // 同步屏幕
                vTaskDelay(100);
            }

        }
        u8g2_SendBuffer(&u8g2); // 同步屏幕
    }
}

/**
  * @brief    Run_Task 任务函数
  * @brief    对电极的一个闭环控制流程任务，获得目标速度 -> PID -> 输出目标速度
  */
void Run_Task(void *pvParameters){
    int8_t trackErr;
    int16_t commonPwmVal = 20;
    // int8_t lPwmVal = commonPwmVal,rPwmVal = commonPwmVal;
    rPwmVal = commonPwmVal;
    lPwmVal = commonPwmVal;
    int16_t pidResult;
    PID *trackPID = PID_Position_Create(10,0,0,100,200);
    Ganv_Track_Init();// 初始化寻迹模块
    Motor_Init();// 初始化电机驱动IO，一个是正反转，一个是PWM调速，周期为3200，初始占空比为0，频率为25Khz
    while(1){
        if(carMode == 1){
			if(Ganv_Get_Err_State() == 1){
                //循迹故障
                //倒车
                Motor_SetSpeed(-10,-10);
            }
            else{
                //Get the track state
                trackState = Ganv_Get_DD();
                trackErr = Ganv_Calc_DD_Err(trackState);
                if(trackErr == 66){
                    rPwmVal = 0;
                    lPwmVal = 0;
                    // lPwmVal = -motor_PWMPeriod*0.4;
                    // rPwmVal = -motor_PWMPeriod*0.4;
                }
                else{
                    pidResult = PID_Position(trackPID, trackErr);
                    rPwmVal = commonPwmVal + pidResult;
                    lPwmVal = commonPwmVal - pidResult;
                }
                Motor_SetSpeed(lPwmVal,rPwmVal);
            }
        }
    }
}

/**
  * @brief    Ranging_Task 任务函数
  * @brief    通过计算 T2CCP0 捕获的脉冲长度得到小车与前方物体间距离
  */
void Ranging_Task(void *pvParameters){
    u16 ultrasonicTimes = 0;
	HC_SR04_Init();
    while(1){
        if(T0CCP1_STA&0x8000){
            //完成一次上升沿脉冲捕获
			ultrasonicTimes = T0CCP1_STA&0x3FFF;
			T0CCP1_STA = 0;
			distance = Filter(ultrasonicTimes*0.17, distance, 0.8);
			printf("Time=%hu, distance=%.1fcm\r\n",ultrasonicTimes,distance);
        }
    }
}

/**
  * @brief    LED2_Task 任务函数
  * @brief    闪烁灯，体现程序正常跑了
  */
void LED2_Task(void *pvParameters){
	Key_ALL_Init();
    while (1){
		LED0_RGB_B_DISABLE;
		LED0_RGB_R_ENABLE;
		vTaskDelay(40);
		LED0_RGB_R_DISABLE;
		LED0_RGB_G_ENABLE;
		vTaskDelay(40);
		LED0_RGB_G_DISABLE;
		LED0_RGB_B_ENABLE;
		vTaskDelay(40);
		switch(Key_Scan()){
			case 1://按键1被按下，切换小车模式
				if(carMode == 0)
					carMode = 1;
				else if(carMode == 1){
					carMode = 0;
					vTaskDelay(20);
					Motor_Stop();
				}
				break;
			case 2://未定
				break;
			default:
				break;
		}
    }
}

/**
  * @brief    LED_Task 任务函数
  * @brief    闪烁灯，体现程序正常跑了
  */
void LED_Task(void *pvParameters){
    while (1){
        LED0_RGB_B_DISABLE;
        LED0_RGB_R_ENABLE;
        vTaskDelay(500);
        LED0_RGB_R_DISABLE;
        LED0_RGB_G_ENABLE;
        vTaskDelay(500);
        LED0_RGB_G_DISABLE;
        LED0_RGB_B_ENABLE;
        vTaskDelay(500);
    }
}

/**
  * @brief    U3RX_Analyzing 任务函数
  * @brief    负责处理无线通信传输的数据
  */
void U3RX_Analyzing_Task(void *pvParameters){
    while (1){
        while(commands[0] != 1);//等待标志位被置1
        BEEP_ENABLE;
        switch (commands[1]){
            case 0x00:
                carMode = 0;
                vTaskDelay(20);
                Motor_Stop();
                break;
            case 0x01:
                carMode = 1;
                break;
            case 0x02:
                carMode = 0;
                Motor_SetSpeed(commands[2], commands[3]);
                break;
            case 0x03:
                carMode = 0;
                Motor_Straight(commands[2]);
                break;
            case 0x04:
                carMode = 0;
                Motor_Back(commands[2]);
                break;
            case 0xFF:
                if(commands[2] == 0x00 && commands[3] == 0x00){
                    //收到应答
                    ackFlag = 1;
                    break;
                }
            default:
                //指令错误
                vTaskDelay(100);
                BEEP_DISABLE;
                vTaskDelay(100);
                BEEP_ENABLE;
                vTaskDelay(100);
                BEEP_DISABLE;
                vTaskDelay(100);
                BEEP_ENABLE;
                break;
        }

        vTaskDelay(100);
        BEEP_DISABLE;
        commands[0] = 0;//清零标志位
    }
}


//T0CCP1的中断服务函数
void TIMER0B_Handler(void){
    //清除中断标志位
	TimerIntClear(TIMER0_BASE,  TimerIntStatus(TIMER0_BASE, true));
    if(T0CCP1_STA & 0x4000){
        //捕获过一个上升沿，这次是下降沿来了
        T0CCP1_STA |= 0x8000;//标记完成一次高电平脉冲捕获
        TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE); //重新设置为上升沿捕获 
        T0CCP1_STA &= ~0x4000;//置零
    }
    else{
        //第一次捕获上升沿
        //清空，开始计时等待下降沿
        T0CCP1_STA = 0;
        T0CCP1_STA |= 0x4000;//标记捕获到了上升沿
        TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE); //设置为下降沿捕获 
    }
}

//T1CCP0的中断服务函数
//服务于捕获右轮霍尔编码脉冲
void TIMER1A_Handler(void){
    uint32_t flag = TimerIntStatus(TIMER1_BASE, true);
    if(flag == TIMER_CAPA_EVENT){
        //累计脉冲数
        rPulseCounter++;
    }
    TimerIntClear(TIMER1_BASE,  flag);
}

//T1CCP1的中断服务函数
//服务于捕获左轮霍尔编码脉冲
void TIMER1B_Handler(void){
    uint32_t flag = TimerIntStatus(TIMER1_BASE, true);
    if(flag == TIMER_CAPB_EVENT){
        //累计脉冲数
        lPulseCounter++;
    }
    TimerIntClear(TIMER1_BASE,  flag);
}


//10us级定时器，实现系统计时
void TIMER0A_Handler(void){
    u8 i;
    static u16 usecond = 0;
    static u16 msecond = 0;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //ms级计时
    if(usecond == 100){
        usecond = 0;
        //经过了1ms

		if(lPulseCounterTime < 0xFFFF)
            lPulseCounterTime++;
		if(rPulseCounterTime < 0xFFFF)
            rPulseCounterTime++;
		
        if(uart0RXTime < 30){
            uart0RXTime++;
        }
        else if(uart0RXTime == 30){
            // 接收完了一串串口消息
            if (UART0_RX_BUF[0] == 0xA1 && UART0_RX_BUF[1] == 0xA1 && UART0_RX_BUF[UART0_RX_STA-1] == 0xB1 && UART0_RX_BUF[UART0_RX_STA-2] == 0xB1) {                
                if(commands[0] == 0){
                    //无线命令缓冲区为空，可以写入数据 
                    for (i = 0; i < UART0_RX_STA-4; i++) {
                        commands[1+i] = UART0_RX_BUF[2+i];
                    }
                    commands[0] = 1;//无线命令缓冲区非空
                }
            }
            UART0_RX_STA = 0;
            uart0RXTime  = 0xFF; // 把时间拉满，表示没有收到新的消息
        }

        if(uart3RXTime < 50){
            uart3RXTime++;
        }
        else if(uart3RXTime == 50){
            // 接收完了一串串口消息
            if (UART3_RX_BUF[0] == 0xA1 && UART3_RX_BUF[1] == 0xA1 && UART3_RX_BUF[UART3_RX_STA-1] == 0xB1 && UART3_RX_BUF[UART3_RX_STA-2] == 0xB1) {
                if(commands[0] == 0){
                    //无线命令缓冲区为空，可以写入数据 
                    for (i = 0; i < UART3_RX_STA-4; i++) {
                        commands[1+i] = UART3_RX_BUF[2+i];
                    }
                    commands[0] = 1;//无线命令缓冲区非空
                }
            }
            UART3_RX_STA = 0;
            uart3RXTime  = 0xFF; // 把时间拉满，表示没有收到新的消息
        }

        if(uart5RXTime < 20){
            uart5RXTime++;
        }
        else if(uart5RXTime == 20){
            // 接收完了一串串口消息
            if (UART5_RX_BUF[0] == 0xA2 && UART5_RX_BUF[1] == 0xA2 && UART5_RX_BUF[8] == 0xB2 && UART5_RX_BUF[9] == 0xB2) {
                pitch   = (((int16_t)UART5_RX_BUF[2])<<8)+UART5_RX_BUF[3];
                roll    = (((int16_t)UART5_RX_BUF[4])<<8)+UART5_RX_BUF[5];
                yaw     = (((int16_t)UART5_RX_BUF[6])<<8)+UART5_RX_BUF[7];
            }
            UART5_RX_STA = 0;
            uart5RXTime  = 0xFF; // 把时间拉满，表示没有收到新的消息
        }

        if(msecond == 1000){
            msecond = 0;
            //经过了1s
        }
        msecond++;
    }
	
	//HC-SR04
	if((T0CCP1_STA & 0X8000)==0 && (T0CCP1_STA & 0X4000)){
		//还未完成捕获，但是已经捕获到高电平了
		if((T0CCP1_STA&0X3FFF) >= 0x3FFF){
			//高电平太长了(持续时间大于700*10us)(即距离太远了，就不测了。注意，距离小于5cm也会导致很长的高电平时间，所以这个模块的最短测距大致限制在了5cn)
			T0CCP1_STA |= 0xFFFF;
		}
		else T0CCP1_STA++;
	}

    usecond++;
}

//串口0中断
void UART0_Handler(void){
    uint32_t flag = UARTIntStatus(UART0_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART0_BASE)){
		    //if FIFO have data
            if (UART0_RX_STA < UART_REC_LEN) {
                uart0RXTime = 0;
                UART0_RX_BUF[UART0_RX_STA] = UARTCharGet(UART0_BASE); // 读取接收到的数据
                UART0_RX_STA++;
            } else {
                UART0_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART0_BASE,flag);
}

//串口3中断
void UART3_Handler(void){
    uint32_t flag = UARTIntStatus(UART3_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART3_BASE)){
		    //if FIFO have data
            if (UART3_RX_STA < UART_REC_LEN) {
                uart3RXTime = 0;
                UART3_RX_BUF[UART3_RX_STA] = UARTCharGet(UART3_BASE); // 读取接收到的数据
                UART3_RX_STA++;
            } else {
                UART3_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART3_BASE,flag);
}

//串口5中断
void UART5_Handler(void){
    uint32_t flag = UARTIntStatus(UART5_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART5_BASE)){
		    //if FIFO have data
            if (UART5_RX_STA < UART5_REC_LEN) {
                uart5RXTime = 0;
                UART5_RX_BUF[UART5_RX_STA] = UARTCharGet(UART5_BASE); // 读取接收到的数据
                UART5_RX_STA++;
            } else {
                UART5_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART5_BASE,flag);
}






