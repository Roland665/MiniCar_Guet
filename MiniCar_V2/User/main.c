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

/*******************************************/
/*
@version V2.0
相较于V1系列
  - 使用无刷电机配合舵机驱动
  - 核心mcu换为TM4C123GH6
  - 

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

/* U1RX_Analyzing 任务 */
// 任务栈深
#define U1RX_Analyzing_Task_Stack_Deep 128
// 任务堆栈
StackType_t U1RX_Analyzing_Task_Stack[U1RX_Analyzing_Task_Stack_Deep];
// 任务句柄
TaskHandle_t U1RX_Analyzing_Task_Handle;
// 任务函数
void U1RX_Analyzing_Task(void *pvParameters);

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
u16 T2CCP0_STA = 0; //输入捕获状态 bit15表示是否完成一次脉冲捕获，bit14表示是否完成脉冲第一次变化沿，bit13~bit0表示脉冲持续时间(T2CCP0_STA++语句触发周期)

float roll; //绕X轴旋转
float pitch;//绕Y轴旋转
float yaw;  //绕Z轴旋转

u8 trackState;//循迹8路数字量bit0对应OUT1~bit7对应OUT8

u8 commands[UART1_REC_LEN+1];//Zigbee接收无线命令缓冲区,commands[0]是无线命令是否堆积标志位，置一表示正在堆积

int16_t lPwmVal;
int16_t rPwmVal;

u8 carMode = 0;//小车运行模式，0表示外部控制，1表示寻迹模式，2为其他

u8 ackFlag = 0;// 应答标志位，收到应答时置一，处理完归零

u32 lPulseCounter = 0;//左轮编码器脉冲计数器
u32 rPulseCounter = 0;//右轮编码器脉冲计数器
u8 lPulseCounterTime = 0xFF;//左轮编码器脉冲计数周期
u8 rPulseCounterTime = 0xFF;//右轮编码器脉冲计数周期

float lSpeed;
float rSpeed;
float speed;
void setup(void) //串口0初始化
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //设置系统时钟为80MHz
	LED_Init();
    BEEP_Init();
	Uart0_Init(115200);
	Uart1_Init(115200);
	Uart2_Init(115200);
	Time0A_Init(800-1);//系统频率为80Mhz，800/80000000=10us,实现10us级中断
}

int main(void)
{
	setup(); // 初始化
    printf("\r\nProgram is Run-begining\r\n系统时钟频率为%u\r\n",SysCtlClockGet());
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
                                               
    //创建 U1RX_Analyzing 任务
    xTaskCreate((TaskFunction_t)	U1RX_Analyzing_Task,				// 任务函数
                (const char *)	"U1RX_Analyzing_Task",				    // 任务名称
                (uint16_t)		U1RX_Analyzing_Task_Stack_Deep,	    // 任务堆栈大小
                (void *)			NULL,					// 传递给任务函数的参数
                (UBaseType_t)	3,						    // 任务优先级
                (TaskHandle_t *)	&U1RX_Analyzing_Task_Handle);		// 任务句柄
    
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
  * @brief OLED显示任务主体
  */
static void SpeedDetection_Task(void *parameter){
    T3CCP0_Init();
    while(1){
        if(lPulseCounterTime >= 50){
            //理论上是每50ms计算一次当前速度
            lSpeed = lPulseCounter*ONEPULSE_FOR_DISTANCE/lPulseCounterTime;//计算出速度，单位是m/s
            lPulseCounterTime = 0;
            lPulseCounter = 0;
        }
        if(rPulseCounterTime >= 50){
            //理论上是每50ms计算一次当前速度
            rSpeed = rPulseCounter*ONEPULSE_FOR_DISTANCE/rPulseCounterTime;//计算出速度，单位是m/s
            rPulseCounterTime = 0;
            rPulseCounter = 0;
        }
        speed = (rSpeed+lSpeed)/2;
        printf("当前车速为:%.1lfcm/s,左车轮为:%.1lfcm/s,右车轮为:%.1lfcm/s\r\n", speed, lSpeed, rSpeed);
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
        if(carMode == 0){
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
            sprintf(str, "%d  %d  %d", lPwmVal, rPwmVal, (int)(lSpeed*100));
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
    int16_t commonPwmVal = motor_PWMPeriod*0.4;
    // int8_t lPwmVal = commonPwmVal,rPwmVal = commonPwmVal;
    rPwmVal = commonPwmVal;
    lPwmVal = commonPwmVal;
    int16_t pidResult;
    PID *trackPID = PID_Position_Create(400,0,0,motor_PWMPeriod,motor_PWMPeriod+7*400);
    Ganv_Track_Init();// 初始化寻迹模块
    Motor_Init();// 初始化电机驱动IO，一个是正反转，一个是PWM调速，周期为3200，初始占空比为0，频率为25Khz
    while(1){
        if(carMode == 1){
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
  * @brief    LED2_Task 任务函数
  * @brief    闪烁灯，体现程序正常跑了
  */
void LED2_Task(void *pvParameters){
    while (1){
            LED0_RGB_R_ENABLE;
            vTaskDelay(200);
            LED0_RGB_R_DISABLE;
            LED0_RGB_G_ENABLE;
            BEEP_DISABLE;
            vTaskDelay(200);
            LED0_RGB_B_ENABLE;
            vTaskDelay(200);
            LED0_RGB_B_DISABLE;
    }
}

/**
  * @brief    LED_Task 任务函数
  * @brief    闪烁灯，体现程序正常跑了
  */
void LED_Task(void *pvParameters){
	
    while (1){
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
  * @brief    U1RX_Analyzing 任务函数
  * @brief    负责处理无线通信传输的数据
  */
void U1RX_Analyzing_Task(void *pvParameters){
    while (1){
        while(commands[0] != 1);//等待标志位被置1
        BEEP_ENABLE;
        switch (commands[1]){
            case 0x00:
                carMode = 0;
                Motor_Stop();
                break;
            case 0x01:
                carMode = 1;
                break;
            case 0x02:
                carMode = 0;
                Motor_SetSpeed(motor_PWMPeriod*0.01*commands[2], motor_PWMPeriod*0.01*commands[3]);
                break;
            case 0x03:
                Motor_Back(motor_PWMPeriod*0.01*commands[2]);
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


//T2CCP0的中断服务函数
void TIMER2A_Handler(void){
    //清除中断标志位
	TimerIntClear(TIMER2_BASE,  TimerIntStatus(TIMER2_BASE, true));
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

//T3CCP0的中断服务函数
//服务于捕获左轮霍尔编码脉冲
void TIMER3A_Handler(void){
    uint32_t flag = TimerIntStatus(TIMER3_BASE, true);
    if(flag == TIMER_CAPA_EVENT){
        //累计脉冲数
        lPulseCounter++;
    }
    TimerIntClear(TIMER3_BASE,  flag);
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

		if(lPulseCounterTime < 0xFF)
            lPulseCounterTime++;
		if(rPulseCounterTime < 0xFF)
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

        if(uart1RXTime < 50){
            uart1RXTime++;
        }
        else if(uart1RXTime == 50){
            // 接收完了一串串口消息
            if (UART1_RX_BUF[0] == 0xA1 && UART1_RX_BUF[1] == 0xA1 && UART1_RX_BUF[UART1_RX_STA-1] == 0xB1 && UART1_RX_BUF[UART1_RX_STA-2] == 0xB1) {
                if(commands[0] == 0){
                    //无线命令缓冲区为空，可以写入数据 
                    for (i = 0; i < UART1_RX_STA-4; i++) {
                        commands[1+i] = UART1_RX_BUF[2+i];
                    }
                    commands[0] = 1;//无线命令缓冲区非空
                }
            }
            UART1_RX_STA = 0;
            uart1RXTime  = 0xFF; // 把时间拉满，表示没有收到新的消息
        }

        if(uart2RXTime < 50){
            uart2RXTime++;
        }
        else if(uart2RXTime == 50){
            // 接收完了一串串口消息
            if (UART2_RX_BUF[0] == 0xA2 && UART2_RX_BUF[1] == 0xA2 && UART2_RX_BUF[5] == 0xB2 && UART2_RX_BUF[6] == 0xB2) {
                roll = UART2_RX_BUF[2];
                pitch = UART2_RX_BUF[3];
                yaw = UART2_RX_BUF[4];
            }
            UART2_RX_STA = 0;
            uart2RXTime  = 0xFF; // 把时间拉满，表示没有收到新的消息
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

//串口1中断
void UART1_Handler(void){
    uint32_t flag = UARTIntStatus(UART1_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART1_BASE)){
		    //if FIFO have data
            if (UART1_RX_STA < UART_REC_LEN) {
                uart1RXTime = 0;
                UART1_RX_BUF[UART1_RX_STA] = UARTCharGet(UART1_BASE); // 读取接收到的数据
                UART1_RX_STA++;
            } else {
                UART1_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART1_BASE,flag);
}

//串口2中断
void UART2_Handler(void){
    uint32_t flag = UARTIntStatus(UART2_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART2_BASE)){
		    //if FIFO have data
            if (UART2_RX_STA < UART2_REC_LEN) {
                uart2RXTime = 0;
                UART2_RX_BUF[UART2_RX_STA] = UARTCharGet(UART2_BASE); // 读取接收到的数据
                UART2_RX_STA++;
            } else {
                UART2_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART2_BASE,flag);
}






