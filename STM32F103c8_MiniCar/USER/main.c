#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sys.h"
#include "usart.h"
#include <stdarg.h> //标准C库文件,让函数能够接收可变参数
#include "led.h"
#include "key.h"
#include "motor.h"
#include "vSensor.h"
#include "PID.h"
#include "OLED.h"
#include "metal_detection.h"
#include "beep.h"
#include "track.h"
/*******************************************/
/*
@version v1.1
相较于v1.0修改：
- 合并了以下两个有偏序关系的任务
  1. 判断循迹情况，输出目标速度<--------+
  2. 电机PWM开始变化-------------------+
- 优化了指令种类，删除了原指令01
- 取消了浮点数，都由整型数替代
- 取消了PID，开环控制对于1.x版本的小车循迹模块更合适
- 补全了完成题目要求的所有必要通信命令

缺点:
- 感觉测速还可以优化，合并和测速有关的任务，取消信号量，直接顺序执行
- 电机稳定性低，难以线性调速
*/

/******************************* 宏定义 ************************************/
/*最多可以存储 2 个 u8类型变量的队列 */
#define REPLACE_V_QUEUE_LENGTH 2
#define REPLACE_V_ITEM_SIZE    sizeof(u8)

/*最多可以存储 7 个 u8类型变量的队列 */
#define WIRELESSCOMMAND_QUEUE_LENGTH 7
#define WIRELESSCOMMAND_ITEM_SIZE    sizeof(u8)

/*最多可以存储 2 个 u8类型变量的队列 */
#define CUSTOMDATA_QUEUE_LENGTH 2
#define CUSTOMDATA_ITEM_SIZE    sizeof(u8)
/**************************** 任务句柄 ********************************/
/* AppTaskCreate 任务句柄 */
static TaskHandle_t AppTaskCreate_Handle = NULL;
/* LED 任务句柄 */
static TaskHandle_t LED_Task_Handle = NULL;
/* ListeningSensors 任务句柄*/
static TaskHandle_t ListeningSensors_Task_Handle = NULL;
/* lCalcVelocity 任务句柄*/
static TaskHandle_t lCalcVelocity_Task_Handle = NULL;
/* rCalcVelocity 任务句柄*/
static TaskHandle_t rCalcVelocity_Task_Handle = NULL;
/* AnalyseCommand 任务句柄*/
static TaskHandle_t AnalyseCommand_Task_Handle = NULL;
/* Run 任务句柄*/
static TaskHandle_t Run_Task_Handle = NULL;
/* OLEDShowing 任务句柄*/
static TaskHandle_t OLEDShowing_Task_Handle = NULL;
/* MetalDetection 任务句柄*/
static TaskHandle_t MetalDetection_Task_Handle = NULL;
/* SendCustomData 任务句柄*/
static TaskHandle_t SendCustomData_Task_Handle = NULL;
/********************************** 内核对象句柄 *********************************/
/*左测速传感器计数信号量句柄*/
static SemaphoreHandle_t vSensorLCountHandle = NULL;
/*右测速传感器计数信号量句柄*/
static SemaphoreHandle_t vSensorRCountHandle = NULL;
/*左轮待修正速度消息队列句柄*/
static QueueHandle_t ReplaceVHandle = NULL;
/*无线控制命令消息队列句柄*/
static QueueHandle_t wirelessCommandHandle = NULL;
/*待发送私有协议消息队列句柄*/
static QueueHandle_t customDataHandle = NULL;
/******************************* 全局变量声明 ************************************/
/* 空闲任务任务堆栈 */
static StackType_t Idle_Task_Stack[configMINIMAL_STACK_SIZE];
/* 定时器任务堆栈 */
static StackType_t Timer_Task_Stack[configTIMER_TASK_STACK_DEPTH];
/* 空闲任务控制块 */
static StaticTask_t Idle_Task_TCB;
/* 定时器任务控制块 */
static StaticTask_t Timer_Task_TCB;

/* AppTaskCreate 任务堆栈 */
static StackType_t AppTaskCreate_Stack[128];
/* LED 任务堆栈 */
static StackType_t LED_Task_Stack[128];
/* ListeningSensors 任务堆栈 */
static StackType_t ListeningSensors_Task_Stack[128];
/* lCalcVelocity 任务堆栈 */
static StackType_t lCalcVelocity_Task_Stack[128];
/* rCalcVelocity 任务堆栈 */
static StackType_t rCalcVelocity_Task_Stack[128];
/* AnalyseCommand 任务堆栈 */
static StackType_t AnalyseCommand_Task_Stack[128];
/* Run 任务堆栈 */
#define RunStackDeep 512
static StackType_t Run_Task_Stack[RunStackDeep];
/* OLEDShowing 任务堆栈 */
#define OLEDShowingStackDeep 256
static StackType_t OLEDShowing_Task_Stack[OLEDShowingStackDeep];
/* MetalDetection 任务堆栈 */
static StackType_t MetalDetection_Task_Stack[128];
/* SendCustomData 任务堆栈 */
#define SendCustomDataStackDeep 128
static StackType_t SendCustomData_Task_Stack[SendCustomDataStackDeep];

/* AppTaskCreate 任务控制块 */
static StaticTask_t AppTaskCreate_TCB;
/* LED 任务控制块 */
static StaticTask_t LED_Task_TCB;
/* ListeningSensors 任务控制块 */
static StaticTask_t ListeningSensors_Task_TCB;
/* lCalcVelocity 任务控制块 */
static StaticTask_t lCalcVelocity_Task_TCB;
/* rCalcVelocity 任务控制块 */
static StaticTask_t rCalcVelocity_Task_TCB;
/* AnalyseCommand 任务控制块 */
static StaticTask_t AnalyseCommand_Task_TCB;
/* Run 任务控制块 */
static StaticTask_t Run_Task_TCB;
/* OLEDShowing 任务控制块 */
static StaticTask_t OLEDShowing_Task_TCB;
/* MetalDetection 任务控制块 */
static StaticTask_t MetalDetection_Task_TCB;
/* SendCustomData 任务控制块 */
static StaticTask_t SendCustomData_Task_TCB;

/* 信号量数据结构指针 */
static StaticSemaphore_t vSensorLCount_Structure; /*左测速传感器计数信号量*/
static StaticSemaphore_t vSensorRCount_Structure; /*右测速传感器计数信号量*/

/* 消息队列数据结构指针 */
static StaticQueue_t ReplaceV_Structure;
static StaticQueue_t wirelessCommand_Structure;
static StaticQueue_t customData_Structure;

/* 消息队列的存储区域，大小至少有 uxQueueLength * uxItemSize 个字节 */
uint8_t ReplaceVStorageArea[REPLACE_V_QUEUE_LENGTH * REPLACE_V_ITEM_SIZE];
uint8_t wirelessCommandStorageArea[WIRELESSCOMMAND_QUEUE_LENGTH * WIRELESSCOMMAND_ITEM_SIZE];
uint8_t customDataStorageArea[CUSTOMDATA_QUEUE_LENGTH * CUSTOMDATA_ITEM_SIZE];

/* 普通全局变量 */
int lVelocity; // 左轮速度(m/s)
int rVelocity; // 右轮速度(m/s)

int lReplaceV; // 目标左轮速度
int rReplaceV; // 目标右轮速度

int lTargetV; // 待替换的左轮速度
int rTargetV; // 待替换的右轮速度

u16 lTime = 0; // 左轮1脉冲计时
u16 rTime = 0; // 右轮1脉冲计时

u16 PWMVal[2] = {0};

u8 carMode = 0; // 小车运行模式 0-受Zigbee协调器控制 1-受循迹外设控制

u8 exti1WaitTime; // used for exti1 debounce

u8 metalDiscoveryFlag = 0; // set 1 means this car found metal

u8 coinCounter; // coin number counter

u8 carState     = 0; // means the car`s move state, 0 is along the line, 1 is turn left slowly, 2 is turn left quickly, 3 is turn right slowly, 4 is turn right quickly
u8 trackingFlag = 0; // Six bit of one byte was used to means the state of car in road. The reflector is 1, the other is 0

u8 nodeCounter    = 0; // When car meet the black node in the line, this counter++
u16 nodeCounterCD = 0; // the nodeCounter CD
int v1[2]         = {0x12, 0x38};
int v2[2]         = {0x11, 0x40};
int v3[2]         = {0x09, 0x50};
int v4[3]         = {0x05, 0x64};
int pwmerr        = 4;
int lV            = 13; // 沿线行驶时左轮推荐速度
int rV            = 40; // 沿线行驶时右轮推荐速度

u8 runRoad = 1;// this variable set 1 means car will along the out line, set 2 this car will along the in line
u16 CHANGEROADTIME_MAX = 0x4a*10;
u16 changeRoadTime = 0xffff;//This variable means the car begin change time to end change time  

u8 AckFlag = 0;//This flag will be set 1 when the car recive another device`s answer back
/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize);

int Filter(float newValue, float oldValue, float alpha);
static void AppTaskCreate(void);                    /* 用于创建任务 */
static void LED_Task(void *parameter);              // LED任务
static void ListeningSensors_Task(void *parameter); // 监听两个测速传感器
static void lCalcVelocity_Task(void *parameter);    // 计算左轮速度任务
static void rCalcVelocity_Task(void *parameter);    // 计算右轮速度任务
static void AnalyseCommand_Task(void *parameter);   // 分析命令任务
static void Run_Task(void *parameter);              // 自巡航任务
static void OLEDShowing_Task(void *parameter);      // OLED显示任务
static void MetalDetection_Task(void *parameter);   // task of detecting the metal
static void SendCustomData_Task(void *parameter);   // task of send custom data
static void Setup(void);                            /* 用于初始化板载相关资源 */

int main(void)
{
    Setup(); // 初始化
    /* 创建 AppTaskCreate 任务 */
    AppTaskCreate_Handle = xTaskCreateStatic((TaskFunction_t)AppTaskCreate,       // 任务函数
                                             (const char *)"AppTaskCreate",       // 任务名称
                                             (uint32_t)128,                       // 任务堆栈大小
                                             (void *)NULL,                        // 传递给任务函数的参数
                                             (UBaseType_t)3,                      // 任务优先级
                                             (StackType_t *)AppTaskCreate_Stack,  // 任务堆栈
                                             (StaticTask_t *)&AppTaskCreate_TCB); // 任务控制块

    if (AppTaskCreate_Handle != NULL) {
        // 任务创建成功
        printf("=====准备进入FreeRTOS!=====\r\n");
        vTaskStartScheduler(); /* 开启调度器 */
    }
    while (1);
}

/**
 * @brief    板载外设初始化
 * @param    void
 * @retval   void
 */
static void Setup(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 4bit都用于设置抢占优先级
    MTS_Init();
    LED_Init();
    BEEP_Init();
    USART1_Init(115200);
    LED_Init();
    BEEP_Init();
    USART1_Init(115200);
    USART2_Init(115200);
    TIM2_Int_Init(10 - 1, 7200 - 1); // 定时器时钟72M，分频系数7200，所以72M/7200=10Khz的计数频率，计数10次为1ms
    vSensors_Init();
    Metal_Detection_Init();
    track_Init();
}

static void AppTaskCreate(void)
{
    taskENTER_CRITICAL(); // 进入临界区

    // 记录左车轮光电测速传感器获取的脉冲
    vSensorLCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,           // 最大计数值
                                                         0,                         // 初始计数值
                                                         &vSensorLCount_Structure); // 信号量的数据结构体
    if (vSensorLCountHandle != NULL)                                                /* 创建成功 */
        printf("vSensorLCount 计数信号量创建成功!\r\n");
    else
        printf("vSensorLCount 计数信号量创建失败!\r\n");

    // 记录右车轮光电测速传感器获取的脉冲
    vSensorRCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,           // 最大计数值
                                                         0,                         // 初始计数值
                                                         &vSensorRCount_Structure); // 信号量的数据结构体
    if (vSensorRCountHandle != NULL)                                                /* 创建成功 */
        printf("vSensorRCount 计数信号量创建成功!\r\n");
    else
        printf("vSensorRCount 计数信号量创建失败!\r\n");

    // 存储车轮待修正车速值
    ReplaceVHandle = xQueueCreateStatic(REPLACE_V_QUEUE_LENGTH, // 队列深度
                                        REPLACE_V_ITEM_SIZE,    // 队列数据单元的单位
                                        ReplaceVStorageArea,    // 队列的存储区域
                                        &ReplaceV_Structure     // 队列的数据结构
    );
    if (ReplaceVHandle != NULL) /* 创建成功 */
        printf("ReplaceV队列创建成功!\r\n");
    else
        printf("ReplaceV队列创建失败!\r\n");

    // 存储控制命令
    wirelessCommandHandle = xQueueCreateStatic(WIRELESSCOMMAND_QUEUE_LENGTH, // 队列深度
                                               WIRELESSCOMMAND_ITEM_SIZE,    // 队列数据单元的单位
                                               wirelessCommandStorageArea,   // 队列的存储区域
                                               &wirelessCommand_Structure    // 队列的数据结构
    );
    if (wirelessCommandHandle != NULL) /* 创建成功 */
        printf("wirelessCommand队列创建成功!\r\n");
    else
        printf("wirelessCommand队列创建失败!\r\n");

    // 存储待发出私有协议数据
    customDataHandle = xQueueCreateStatic(CUSTOMDATA_QUEUE_LENGTH, // 队列深度
                                               CUSTOMDATA_ITEM_SIZE,    // 队列数据单元的单位
                                               customDataStorageArea,   // 队列的存储区域
                                               &customData_Structure    // 队列的数据结构
    );
    if (customDataHandle != NULL) /* 创建成功 */
        printf("customData队列创建成功!\r\n");
    else
        printf("customData队列创建失败!\r\n");

    /* 创建LED_Task任务 */
    LED_Task_Handle = xTaskCreateStatic((TaskFunction_t)LED_Task,       // 任务函数
                                        (const char *)"LED_Task",       // 任务名称
                                        (uint32_t)128,                  // 任务栈深
                                        (void *)NULL,                   // 传递给任务函数的参数
                                        (UBaseType_t)4,                 // 任务优先级
                                        (StackType_t *)LED_Task_Stack,  // 任务堆栈
                                        (StaticTask_t *)&LED_Task_TCB); // 任务控制块
    if (LED_Task_Handle != NULL)                                        /* 创建成功 */
        printf("LED_Task任务创建成功!\r\n");
    else
        printf("LED_Task任务创建失败!\r\n");

    /* 创建 ListeningSensors_Task 任务 */
    ListeningSensors_Task_Handle = xTaskCreateStatic((TaskFunction_t)ListeningSensors_Task,       // 任务函数
                                                     (const char *)"ListeningSensors_Task",       // 任务名称
                                                     (uint32_t)128,                               // 任务栈深
                                                     (void *)NULL,                                // 传递给任务函数的参数
                                                     (UBaseType_t)4,                              // 任务优先级
                                                     (StackType_t *)ListeningSensors_Task_Stack,  // 任务堆栈
                                                     (StaticTask_t *)&ListeningSensors_Task_TCB); // 任务控制块
    if (ListeningSensors_Task_Handle != NULL)                                                     /* 创建成功 */
        printf("ListeningSensors_Task任务创建成功!\r\n");
    else
        printf("ListeningSensors_Task任务创建失败!\r\n");

    /* 创建 lCalcVelocity_Task 任务 */
    lCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t)lCalcVelocity_Task,       // 任务函数
                                                  (const char *)"lCalcVelocity_Task",       // 任务名称
                                                  (uint32_t)128,                            // 任务栈深
                                                  (void *)NULL,                             // 传递给任务函数的参数
                                                  (UBaseType_t)4,                           // 任务优先级
                                                  (StackType_t *)lCalcVelocity_Task_Stack,  // 任务堆栈
                                                  (StaticTask_t *)&lCalcVelocity_Task_TCB); // 任务控制块
    if (lCalcVelocity_Task_Handle != NULL)                                                  /* 创建成功 */
        printf("lCalcVelocity_Task任务创建成功!\r\n");
    else
        printf("lCalcVelocity_Task任务创建失败!\r\n");

    /* 创建 rCalcVelocity_Task 任务 */
    rCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t)rCalcVelocity_Task,       // 任务函数
                                                  (const char *)"rCalcVelocity_Task",       // 任务名称
                                                  (uint32_t)128,                            // 任务栈深
                                                  (void *)NULL,                             // 传递给任务函数的参数
                                                  (UBaseType_t)4,                           // 任务优先级
                                                  (StackType_t *)rCalcVelocity_Task_Stack,  // 任务堆栈
                                                  (StaticTask_t *)&rCalcVelocity_Task_TCB); // 任务控制块
    if (rCalcVelocity_Task_Handle != NULL)                                                  /* 创建成功 */
        printf("rCalcVelocity_Task任务创建成功!\r\n");
    else
        printf("rCalcVelocity_Task任务创建失败!\r\n");

    /* 创建 AnalyseCommand_Task 任务 */
    AnalyseCommand_Task_Handle = xTaskCreateStatic((TaskFunction_t)AnalyseCommand_Task,       // 任务函数
                                                   (const char *)"AnalyseCommand_Task",       // 任务名称
                                                   (uint32_t)128,                             // 任务栈深
                                                   (void *)NULL,                              // 传递给任务函数的参数
                                                   (UBaseType_t)4,                            // 任务优先级
                                                   (StackType_t *)AnalyseCommand_Task_Stack,  // 任务堆栈
                                                   (StaticTask_t *)&AnalyseCommand_Task_TCB); // 任务控制块
    if (AnalyseCommand_Task_Handle != NULL)                                                   /* 创建成功 */
        printf("AnalyseCommand任务创建成功!\r\n");
    else
        printf("AnalyseCommand任务创建失败!\r\n");

    /* 创建 Run_Task 任务 */
    Run_Task_Handle = xTaskCreateStatic((TaskFunction_t)Run_Task,       // 任务函数
                                        (const char *)"Run_Task",       // 任务名称
                                        (uint32_t)RunStackDeep,         // 任务栈深
                                        (void *)NULL,                   // 传递给任务函数的参数
                                        (UBaseType_t)4,                 // 任务优先级
                                        (StackType_t *)Run_Task_Stack,  // 任务堆栈
                                        (StaticTask_t *)&Run_Task_TCB); // 任务控制块
    if (Run_Task_Handle != NULL)                                        /* 创建成功 */
        printf("Run任务创建成功!\r\n");
    else
        printf("Run任务创建失败!\r\n");

    /* 创建 OLEDShowing_Task 任务 */
    OLEDShowing_Task_Handle = xTaskCreateStatic((TaskFunction_t)OLEDShowing_Task,       // 任务函数
                                                (const char *)"OLEDShowing_Task",       // 任务名称
                                                (uint32_t)OLEDShowingStackDeep,         // 任务栈深
                                                (void *)NULL,                           // 传递给任务函数的参数
                                                (UBaseType_t)4,                         // 任务优先级
                                                (StackType_t *)OLEDShowing_Task_Stack,  // 任务堆栈
                                                (StaticTask_t *)&OLEDShowing_Task_TCB); // 任务控制块
    if (OLEDShowing_Task_Handle != NULL)                                                /* 创建成功 */
        printf("OLEDShowing任务创建成功!\r\n");
    else
        printf("OLEDShowing任务创建失败!\r\n");

    /* 创建 MetalDetection_Task 任务 */
    MetalDetection_Task_Handle = xTaskCreateStatic((TaskFunction_t)MetalDetection_Task,       // 任务函数
                                                   (const char *)"MetalDetection_Task",       // 任务名称
                                                   (uint32_t)128,                             // 任务栈深
                                                   (void *)NULL,                              // 传递给任务函数的参数
                                                   (UBaseType_t)4,                            // 任务优先级
                                                   (StackType_t *)MetalDetection_Task_Stack,  // 任务堆栈
                                                   (StaticTask_t *)&MetalDetection_Task_TCB); // 任务控制块
    if (MetalDetection_Task_Handle != NULL)                                                   /* 创建成功 */
        printf("MetalDetection任务创建成功!\r\n");
    else
        printf("MetalDetection任务创建失败!\r\n");

    /* 创建 SendCustomData_Task 任务 */
    SendCustomData_Task_Handle = xTaskCreateStatic((TaskFunction_t)SendCustomData_Task,       // 任务函数
                                                   (const char *)"SendCustomData_Task",       // 任务名称
                                                   (uint32_t)SendCustomDataStackDeep,                             // 任务栈深
                                                   (void *)NULL,                              // 传递给任务函数的参数
                                                   (UBaseType_t)4,                            // 任务优先级
                                                   (StackType_t *)SendCustomData_Task_Stack,  // 任务堆栈
                                                   (StaticTask_t *)&SendCustomData_Task_TCB); // 任务控制块
    if (SendCustomData_Task_Handle != NULL)                                                   /* 创建成功 */
        printf("SendCustomData任务创建成功!\r\n");
    else
        printf("SendCustomData任务创建失败!\r\n");

    vTaskDelete(AppTaskCreate_Handle); // 删除AppTaskCreate任务
    printf("======进入FreeRTOS!======\r\n");
    taskEXIT_CRITICAL(); // 退出临界区
}

/**
 * @brief main body of send custom data
 */
static void SendCustomData_Task(void *parameter){
    UBaseType_t xReturn = pdPASS;
    u8 i = 0;
    u8 Data[CUSTOMDATA_QUEUE_LENGTH];
    u8 breakCounter = 0;
    while(1){
        for (i = 0; i < CUSTOMDATA_QUEUE_LENGTH; i++) {
            xReturn = xQueueReceive(customDataHandle, &Data[i], portMAX_DELAY);
        }
        if (xReturn != pdPASS)
            printf("命令消息接收失败\r\n");
        AckFlag = 0;
        breakCounter = 0;
        while(AckFlag != 1 && breakCounter < 6){
            //循环发送数据，直到收到应答，1.8秒内没收到就不发了
			USART_SendData(USART2, 0xC1);         //向串口1发送数据
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
			USART_SendData(USART2, 0xC2);         //向串口1发送数据
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
			USART_SendData(USART2, 0xC3);         //向串口1发送数据
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
			USART_SendData(USART2, 0xC4);         //向串口1发送数据
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
            for(i = 0; i < CUSTOMDATA_QUEUE_LENGTH; i++){
                USART_SendData(USART2, Data[i]);         //向串口1发送数据
                while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//等待发送结束
            }
            vTaskDelay(300);
            breakCounter++;
        }
    }
}

/**
 * @brief main body of metal-detection
 */
static void MetalDetection_Task(void *parameter)
{
    UBaseType_t xReturn = pdPASS;
    u8 beSendData[2] = {0x09,0x00};
	u8 i = 0;
    while (1) {
        if (nodeCounter == 0 && 1 == GPIO_ReadInputDataBit(METAL_DET_GPIO, METAL_DET_Pin)) {
            MTLEN(TIM3, 0);
            MTREN(TIM3, 0);
            coinCounter++;
            // 挂起任务
            vTaskSuspend(Run_Task_Handle);
            // 同步硬币数量
            beSendData[1] = coinCounter;
			for(i = 0; i < CUSTOMDATA_QUEUE_LENGTH; i++){				
				xReturn = xQueueSend(customDataHandle, &beSendData[i], 1000/portTICK_PERIOD_MS);
				if(xReturn != pdPASS)
					printf("硬币数量消息发送失败\r\n");
			}
            metalDiscoveryFlag = 1;
            // 响两秒蜂鸣器
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(1000);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP               = 0;
            metalDiscoveryFlag = 0;
            pwmerr             = 2;
            // 解挂任务
            vTaskResume(Run_Task_Handle);
            while (1 == GPIO_ReadInputDataBit(METAL_DET_GPIO, METAL_DET_Pin))
                ;
            vTaskDelay(300);
            pwmerr = 3;
        }
    }
}

/**

  * @brief OLED显示任务主体
  * @brief
  * @param
  * @retval
  */
static void OLEDShowing_Task(void *parameter)
{
    char str[128] = {0}; // 需要显示的字符串临时存放处
    // 初始化u8g2
    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay);
    u8g2_InitDisplay(&u8g2);     // send init sequence to the display, display is in sleep mode after this
    u8g2_SetPowerSave(&u8g2, 0); // place 1 means open power-saveing, you`ll see nothing in the screem
    int v = 0;
    while (1) {
        u8g2_ClearBuffer(&u8g2); // clear the u8g2 buffer
        if (metalDiscoveryFlag == 0) {
            vTaskDelay(10);
            v = (lVelocity + rVelocity) / 2;
            u8g2_ClearBuffer(&u8g2);                         // wake up display
            u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t); // 图标大全
            u8g2_DrawGlyph(&u8g2, 82, 20, 0x0047);           // 硬币数量显示图标
            u8g2_DrawGlyph(&u8g2, 0, 63, 0x029E);            // 小电驴图标

            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf); // 9像素点高字符库
            u8g2_DrawStr(&u8g2, 21, 59, ":");        // 小电驴后冒号，表示速度
            u8g2_DrawStr(&u8g2, 103, 15, ":");       // 硬币图标后冒号，表示硬币数量
            sprintf(str, "0.%dm/s", v);
            u8g2_DrawStr(&u8g2, 32, 59, str); // 小车实际速度
            sprintf(str, "%d", coinCounter);
            u8g2_DrawStr(&u8g2, 114, 15, str); // 硬币实际数量
        } else {
            u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t); // 图标大全
            u8g2_DrawGlyph(&u8g2, 82, 20, 0x0047);           // 硬币数量显示图标

            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf); // 9像素点高字符库
            u8g2_DrawStr(&u8g2, 103, 15, ":");       // 硬币图标后冒号，表示硬币数量
            sprintf(str, "%d", coinCounter);
            u8g2_DrawStr(&u8g2, 114, 15, str); // 硬币实际数量

            u8g2_SetFont(&u8g2, u8g2_font_emoticons21_tr); // 表情大全
            u8g2_DrawGlyphX2(&u8g2, 40, 50, 0x0030);       // 检测到硬币显示表情
        }
        if (nodeCounter == 3) {
            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf); // 9像素点高字符库
            u8g2_DrawStr(&u8g2, 100, 59, "speed up!");
        }
        u8g2_SendBuffer(&u8g2); // 同步屏幕
    }
}

/**
  * @brief Run 任务主体
  */

static void Run_Task(void *parameter)
{
    u16 GPIO_Pins[6] = {GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15}; // 红外模块引脚
    u8 i;

    int ignoreErr = 0; // 忽视速度上的误差值
    while (1) {
        // Base on the mode of the car to do thing
        // if mode is 1, means the car was Run
        if (carMode == 1) {
            // Detecting state of the car
            trackingFlag = 0;
            for (i = 0; i < 6; i++) {
                trackingFlag |= GPIO_ReadInputDataBit(GPIOB, GPIO_Pins[i]);
                trackingFlag <<= 1;
            }
            trackingFlag >>= 1;
			
			if (runRoad == 2 && changeRoadTime < CHANGEROADTIME_MAX){
				lTargetV = v3[0]+6;
				rTargetV = v3[1]+4;
			}
			else {
				// along the line
				if (trackingFlag == 0x0C || trackingFlag == 0x00) { // 0b001100 0b000000
					lTargetV = lV;
					rTargetV = rV;
				}
				// Turn left
				else if (trackingFlag == 0x08) { // 0b001000
					// the car shifted a bit to the right
					lTargetV = v1[0];
					rTargetV = v1[1];
				} else if (trackingFlag == 0x18 || trackingFlag == 0x10) { // 0b011000 || 0b010000
					// the car shifted to the right
					lTargetV = v2[0];
					rTargetV = v2[1];
				} else if (trackingFlag == 0x30 || trackingFlag == 0x20) { // 0b110000 || 0b100000
					// the car shifted a lot to the right
					lTargetV = v3[0];
					rTargetV = v3[1];
				}
				// Turn right
				else if (trackingFlag == 0x04) { // 0b000100
					// the car shifted a bit to the left
					rTargetV = v1[0] + 11;
					lTargetV = v1[1] - 17;
				} else if (trackingFlag == 0x02 || trackingFlag == 0x06) { // 0b000010 || 0b000110
					// the car shifted to the left
					rTargetV = v2[0] + 16;
					lTargetV = v2[1] - 23;
				} else if (trackingFlag == 0x03 || trackingFlag == 0x01) { // 0b000011 || 0b000001
					// the car shifted a lot to the left
					rTargetV = v3[0] + 19;
					lTargetV = v3[1] - 27;
				} else {
					if (nodeCounterCD >= 1000) {
						if(nodeCounter == 0)
							nodeCounter = 1;
						else if(nodeCounter == 3)
							nodeCounter = 2;
						else if(nodeCounter == 4)
							nodeCounter = 5;
						
					}
					// 处于不正常状态，speed down
					lTargetV = lV-7;
					rTargetV = rV-8;
				}
            }
        }
        rReplaceV = rTargetV;
        lReplaceV = lTargetV;

        // Adjust pwm duty ratio to speed control
        // left
        if (lTargetV != 0) {
            if (lReplaceV < lVelocity - ignoreErr && PWMVal[0] > 14) {
                PWMVal[0] -= pwmerr;
                MTLEN(TIM3, PWMVal[0]);
            } else if (lReplaceV > lVelocity + ignoreErr && PWMVal[0] < 100) {
                PWMVal[0] += pwmerr;
                MTLEN(TIM3, PWMVal[0]);
            }
			else{
				if(PWMVal[0] < 14){
					PWMVal[0] = 14;
					MTLEN(TIM3, PWMVal[0]);
				}
				else if(PWMVal[0] > 100){
					PWMVal[0] = 100;
					MTLEN(TIM3,PWMVal[0]);
				}
			}
        } else {
            PWMVal[0] = 0;
            MTLEN(TIM3, PWMVal[0]);
        }
        // right
        if (rTargetV != 0) {
            if (rReplaceV < rVelocity - ignoreErr && PWMVal[1] > 10) {
                PWMVal[1] -= pwmerr;
                MTREN(TIM3, PWMVal[1]);
            } else if (rReplaceV > rVelocity + ignoreErr && PWMVal[1] < 100) {
                PWMVal[1] += pwmerr;
                MTREN(TIM3, PWMVal[1]);
            }
            else{
				if(PWMVal[1] < 14){
					PWMVal[1] = 14;
					MTREN(TIM3, PWMVal[1]);
				}
				else if(PWMVal[1] > 100){
					PWMVal[1] = 100;
					MTREN(TIM3,PWMVal[1]);
				}
			}
			printf("左轮速度为:0.%dm/s  右轮速度为:0.%dm/s  左轮待替换速度为:0.%dm/s  右轮待替换速度为:0.%dm/s\r\n", lVelocity, rVelocity, lReplaceV, rReplaceV);
        } else {
            PWMVal[1] = 0;
            MTREN(TIM3, PWMVal[1]);
        }
    }
}

/**
 * @brief    分析控制命令任务主体
 */
static void AnalyseCommand_Task(void *parameter)
{
    BaseType_t xReturn = pdPASS;
    u8 commands[WIRELESSCOMMAND_QUEUE_LENGTH];
    u8 i;
    while (1) {
        for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
            xReturn = xQueueReceive(wirelessCommandHandle, &commands[i], portMAX_DELAY);
        }
        if (xReturn != pdPASS)
            printf("命令消息接收失败\r\n");
        printf("成功接收控制命令：%d\r\n", commands[0]);

        if (commands[0] == 0x00) {
            carMode  = 0; // 手动挡
            lTargetV = 0;
            rTargetV = 0;
            // 有空了注释下面两行看看停车及不及时
            MTLEN(TIM3, 0);
            MTREN(TIM3, 0);
            printf("停车~\r\n");
        } else if (commands[0] == 0x01) {
            carMode  = 0; // 手动挡
            lTargetV = commands[1];
            rTargetV = commands[2];
            printf("调速成功~左右车轮目标车速为：0.%dm/s  0.%dm/s\r\n", lTargetV, rTargetV);
        } else if (commands[0] == 0x02) {
            carMode = 1;
            printf("开车！自巡航模式\r\n");
			pwmerr = 1;
			vTaskDelay(300);
			pwmerr = 2;
			vTaskDelay(600);
			pwmerr = 3;
        } else if (commands[0] == 0x03) {
            v1[0] = commands[1];
            v1[1] = commands[2];
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        } else if (commands[0] == 0x04) {
            v2[0] = commands[1];
            v2[1] = commands[2];
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        } else if (commands[0] == 0x05) {
            v3[0] = commands[1];
            v3[1] = commands[2];
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        } else if (commands[0] == 0x06) {
            CHANGEROADTIME_MAX = commands[1]*10;
            BEEP   = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        } else if (commands[0] == 0x07) {
            lV += 15;
            rV += 23;
            v1[0] += 13;
            v1[1] += 25;
            v2[0] += 16;
            v2[1] += 30;
            v3[0] += 16;
            v3[1] += 31;
            pwmerr += 15;
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        } else if (commands[0] == 0x08) {
            lV -= 15;
            rV -= 23;
            v1[0] -= 13;
            v1[1] -= 25;
            v2[0] -= 16;
            v2[1] -= 30;
            v3[0] -= 16;
            v3[1] -= 31;
            pwmerr -= 15;
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        }
		else if (commands[0] == 0x0A){
			runRoad = 2;
			changeRoadTime = 0;
			v2[0] -= 1;
			v2[1] += 3;
			v3[0] -= 2;
			v3[1] += 4;
		}
        else if(commands[0] == 0xFF){
            AckFlag = 1;//收到应答
        }
    }
}

float filterNum = 0.5f; // 低通滤波系数
/**
 * @brief    计算速度任务主体
 * @brief    计算每秒测速传感器
 */
static void lCalcVelocity_Task(void *parameter)
{
    u16 lCounter = 0;
    u16 i;
    while (1) {
        xSemaphoreTake(vSensorLCountHandle, portMAX_DELAY);
        lCounter = uxSemaphoreGetCount(vSensorLCountHandle);
        for (i = 0; i < lCounter; i++) {
            xSemaphoreTake(vSensorLCountHandle, 0);
        }
        lCounter++;
        // 计算轮子的速度
        lVelocity = Filter((ONEPULSEDISTANCE * lCounter) * 100 / lTime, lVelocity, filterNum);
        lTime     = 0;
    }
}

/**
 * @brief    计算速度任务主体
 * @brief    计算每秒测速传感器
 */
static void rCalcVelocity_Task(void *parameter)
{
    u16 rCounter;
    u16 i;
    while (1) {
        xSemaphoreTake(vSensorRCountHandle, portMAX_DELAY);
        rCounter = uxSemaphoreGetCount(vSensorRCountHandle);
        for (i = 0; i < rCounter; i++) {
            xSemaphoreTake(vSensorRCountHandle, 0);
        }
        rCounter++;
        // 计算轮子的速度
        rVelocity = Filter((ONEPULSEDISTANCE * rCounter) * 100 / rTime, rVelocity, filterNum);
        rTime     = 0;
    }
}

/**
 * @brief    监听测速传感器任务主体
 * @param    parameter
 * @retval   void
 */
static void ListeningSensors_Task(void *parameter)
{
    BaseType_t xReturn = pdPASS;
    u8 L               = 0;
    u8 R               = 0;
    while (1) {
        if (L != GPIO_ReadInputDataBit(VSENSORL_GPIO, VSENSORL_Pin)) {
            if (L == 1) {
                xReturn = xSemaphoreGive(vSensorLCountHandle);
                if (xReturn != pdTRUE)
                    printf("左测速器信号量赋值失败，错误码为：%d\r\n", xReturn);
            }
            L = !L;
        }
        if (R != GPIO_ReadInputDataBit(VSENSORR_GPIO, VSENSORR_Pin)) {
            if (R == 1) {
                xReturn = xSemaphoreGive(vSensorRCountHandle);
                if (xReturn != pdTRUE)
                    printf("右测速器信号量赋值失败，错误码为：%d\r\n", xReturn);
            }
            R = !R;
        }
    }
}

/**
 * @brief    LED_Task任务主体
 * @param    parameter
 * @retval   void
 */
static void LED_Task(void *parameter)
{
    while (1) {
        TestLED = 0;
        vTaskDelay(500 / portTICK_PERIOD_MS); /*延时500ms*/
        TestLED = 1;
        vTaskDelay(500 / portTICK_PERIOD_MS); /*延时500ms*/
    }
}

/**
 * @brief    一阶低通滤波算法
 * @param    newValue: 新采样的数据
 * @param    oldValue: 上一个滤波输出值
 * @param    alpha   : 滤波系数
 * @retval   本次滤波输出值
 */
int Filter(float newValue, float oldValue, float alpha)
{
    newValue = (int)(alpha * newValue + (1 - alpha) * oldValue);
    return newValue;
}



/**
 * @brief    获取空闲任务的任务堆栈和任务控制块内存
 * @param    ppxIdleTaskTCBBuffer		:	任务控制块内存
 * @param    ppxIdleTaskStackBuffer	:	任务堆栈内存
 * @param    pulIdleTaskStackSize		:	任务堆栈大小
 * @retval   void
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &Idle_Task_TCB;           /* 任务控制块内存 */
    *ppxIdleTaskStackBuffer = Idle_Task_Stack;          /* 任务堆栈内存 */
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE; /* 任务堆栈大小 */
}

/**
 * @brief    获取定时器任务的任务堆栈和任务控制块内存
 * @param    ppxTimerTaskTCBBuffer	:		任务控制块内存
 * @param    ppxTimerTaskStackBuffer	:	任务堆栈内存
 * @param    pulTimerTaskStackSize	:		任务堆栈大小
 * @retval   void
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    *ppxTimerTaskTCBBuffer   = &Timer_Task_TCB;              /* 任务控制块内存 */
    *ppxTimerTaskStackBuffer = Timer_Task_Stack;             /* 任务堆栈内存 */
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH; /* 任务堆栈大小 */
}

/**
 * @brief		串口1中断服务程序
 * @param		void
 * @retval		void
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { // 接收中断
        if (USART1_RX_STA < USART_REC_LEN) {
            usart1RXTime                 = 0;
            USART1_RX_BUF[USART1_RX_STA] = USART_ReceiveData(USART1); // 读取接收到的数据
            USART1_RX_STA++;
        } else {
            USART1_RX_STA = 0;
        }
    }
}

/**
 * @brief		串口2中断服务程序
 * @param		void
 * @retval		void
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) { // 接收中断
        if (USART2_RX_STA < USART_REC_LEN) {
            usart2RXTime                 = 0;
            USART2_RX_BUF[USART2_RX_STA] = USART_ReceiveData(USART2); // 读取接收到的数据
            USART2_RX_STA++;
        } else {
            USART2_RX_STA = 0;
        }
    }
}

// 定时器2中断服务函数
// 每毫秒触发一次中断
void TIM2_IRQHandler(void)
{
    BaseType_t xReturn = pdPASS;
    u8 i;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) { // 溢出中断
        if (lTime < 300)
            lTime++;
        else
            lVelocity = 0;

        if (rTime < 300)
            rTime++;
        else
            rVelocity = 0;

        if (usart1RXTime < 10)
            usart1RXTime++;

        if (usart2RXTime < 10)
            usart2RXTime++;

        if (nodeCounterCD < 1000)
            nodeCounterCD++;

		if(changeRoadTime < CHANGEROADTIME_MAX)
			changeRoadTime++;

        if (nodeCounter == 1) {
            // speed up
            lV += 5;
            rV += 7;
            v1[0] += 5;
            v1[1] += 9;
            v2[0] += 5;
            v2[1] += 11;
            v3[0] += 4;
            v3[1] += 8;
            pwmerr += 4;
			nodeCounter = 3;
			nodeCounterCD = 0;
        } else if (nodeCounter == 2){
            // speed down
            lV -= 5;
            rV -= 7;
            v1[0] -= 5;
            v1[1] -= 9;
            v2[0] -= 5;
            v2[1] -= 11;
            v3[0] -= 4;
            v3[1] -= 8;
            pwmerr -= 4;
			nodeCounter = 4;
			nodeCounterCD = 0;
        }
		else if(nodeCounter == 5){
			// BEEP = ;
			runRoad = 2;
			changeRoadTime = 0;
			v2[0] -= 1;
			v2[1] += 3;
			v3[0] -= 2;
			v3[1] += 4;
			nodeCounter = 6;
			nodeCounterCD = 0;	
		}
        if (usart1RXTime == 10) {
            // 接收完了一串串口消息
            if (USART1_RX_BUF[0] == 0xC1 && USART1_RX_BUF[1] == 0xC2 && USART1_RX_BUF[2] == 0xC3 && USART1_RX_BUF[3] == 0xC4) {
                for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
                    xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART1_RX_BUF[4 + i], NULL);
                }
                if (xReturn != pdTRUE)
                    printf("控制命令消息发送失败");
            }
            USART1_RX_STA = 0;
            usart1RXTime  = 0xFF; // 把时间拉满，表示没有收到新的消息
        }
        if (usart2RXTime == 10) {
            // 接收完了一串串口消息
            if (USART2_RX_BUF[0] == 0xC1 && USART2_RX_BUF[1] == 0xC2 && USART2_RX_BUF[2] == 0xC3 && USART2_RX_BUF[3] == 0xC4) {
                for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
                    xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART2_RX_BUF[4 + i], NULL);
                }
                if (xReturn != pdTRUE)
                    printf("控制命令消息发送失败");
            }
            USART2_RX_STA = 0;
            usart2RXTime  = 0xFF; // 把时间拉满，表示没有收到新的消息
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志位
    }
}
