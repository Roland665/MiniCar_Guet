#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sys.h"
#include "usart.h"
#include  <stdarg.h>//标准C库文件,让函数能够接收可变参数
#include "led.h"
#include "key.h"
#include "motor.h"
#include "vSensor.h"
#include "PID.h"

/******************************* 宏定义 ************************************/
/*最多可以存储 2 个 u8类型变量的队列 */
#define REPLACE_V_QUEUE_LENGTH 2
#define REPLACE_V_ITEM_SIZE sizeof(u8)

/*最多可以存储 3 个 u8类型变量的队列 */
#define WIRELESSCOMMAND_QUEUE_LENGTH 3
#define WIRELESSCOMMAND_ITEM_SIZE sizeof(u8)
/**************************** 任务句柄 ********************************/L。
/* AppTaskCreate 任务句柄 */
static TaskHandle_t AppTaskCreate_Handle = NULL;
/* LED 任务句柄 */
static TaskHandle_t LED_Task_Handle = NULL;
/* KeyScan 任务句柄*/
//static TaskHandle_t KeyScan_Task_Handle = NULL;
/* Motor 任务句柄*/
static TaskHandle_t Motor_Task_Handle = NULL;
/* ListeningSensors 任务句柄*/
static TaskHandle_t ListeningSensors_Task_Handle = NULL;
/* lCalcVelocity 任务句柄*/
static TaskHandle_t lCalcVelocity_Task_Handle = NULL;
/* rCalcVelocity 任务句柄*/
static TaskHandle_t rCalcVelocity_Task_Handle = NULL;
/* AnalyseCommand 任务句柄*/
static TaskHandle_t AnalyseCommand_Task_Handle = NULL;
/* PIDCalculator 任务句柄*/
static TaskHandle_t PIDCalculator_Task_Handle = NULL;
/********************************** 内核对象句柄 *********************************/
static SemaphoreHandle_t printfSemphr_Handle = NULL;/* 串口打印互斥信号量句柄*/

static SemaphoreHandle_t vSensorLCountHandle = NULL;//左测速传感器计数信号量句柄
static SemaphoreHandle_t vSensorRCountHandle = NULL;//右测速传感器计数信号量句柄

static QueueHandle_t ReplaceVHandle = NULL;//左轮待修正速度消息队列句柄

static QueueHandle_t wirelessCommandHandle = NULL;//无线控制命令消息队列句柄
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
/* KeyScan 任务堆栈 */
//static StackType_t KeyScan_Task_Stack[128];
/* Motor 任务堆栈 */
static StackType_t Motor_Task_Stack[128];
/* ListeningSensors 任务堆栈 */
static StackType_t ListeningSensors_Task_Stack[128];
/* lCalcVelocity 任务堆栈 */
static StackType_t lCalcVelocity_Task_Stack[128];
/* rCalcVelocity 任务堆栈 */
static StackType_t rCalcVelocity_Task_Stack[128];
/* AnalyseCommand 任务堆栈 */
static StackType_t AnalyseCommand_Task_Stack[128];
/* PIDCalculator 任务堆栈 */
static StackType_t PIDCalculator_Task_Stack[128];

/* AppTaskCreate 任务控制块 */
static StaticTask_t AppTaskCreate_TCB;
/* LED 任务控制块 */
static StaticTask_t LED_Task_TCB;
/* KeyScan 任务控制块 */
//static StaticTask_t KeyScan_Task_TCB;
/* Motor 任务控制块 */
static StaticTask_t Motor_Task_TCB;
/* ListeningSensors 任务控制块 */
static StaticTask_t ListeningSensors_Task_TCB;
/* lCalcVelocity 任务控制块 */
static StaticTask_t lCalcVelocity_Task_TCB;
/* rCalcVelocity 任务控制块 */
static StaticTask_t rCalcVelocity_Task_TCB;
/* AnalyseCommand 任务控制块 */
static StaticTask_t AnalyseCommand_Task_TCB;
/* PIDCalculator 任务控制块 */
static StaticTask_t PIDCalculator_Task_TCB;


/* 信号量数据结构指针 */
static StaticSemaphore_t printfSemphr_Structure;/*串口打印互斥信号量*/
static StaticSemaphore_t vSensorLCount_Structure;/*左测速传感器计数信号量*/
static StaticSemaphore_t vSensorRCount_Structure;/*右测速传感器计数信号量*/

/* 消息队列数据结构指针 */
static StaticQueue_t ReplaceV_Structure;
static StaticQueue_t wirelessCommand_Structure;

/* 消息队列的存储区域，大小至少有 uxQueueLength * uxItemSize 个字节 */
uint8_t ReplaceVStorageArea[ REPLACE_V_QUEUE_LENGTH * REPLACE_V_ITEM_SIZE ];
uint8_t wirelessCommandStorageArea[ WIRELESSCOMMAND_QUEUE_LENGTH * WIRELESSCOMMAND_ITEM_SIZE ]; 

/* 普通全局变量 */
portFLOAT lVelocity;//左轮速度(m/s)
portFLOAT rVelocity;//右轮速度(m/s)

portFLOAT lReplaceV;//目标左轮速度
portFLOAT rReplaceV;//目标右轮速度

portFLOAT lTargetV;//待替换的左轮速度
portFLOAT rTargetV;//待替换的右轮速度

u16 lTime = 0;//左轮1脉冲计时
u16 rTime = 0;//右轮1脉冲计时

u16 PWMVal[2] = {0};
/*
*************************************************************************
*                             函数声明
*************************************************************************
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, 
								   StackType_t **ppxIdleTaskStackBuffer, 
								   uint32_t *pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, 
									StackType_t **ppxTimerTaskStackBuffer, 
									uint32_t *pulTimerTaskStackSize);
static void xPrintf(char *format, ...);
portFLOAT Filter(portFLOAT newValue, portFLOAT oldValue, portFLOAT alpha);
static void AppTaskCreate(void);/* 用于创建任务 */
static void LED_Task(void* parameter);//LED任务
//static void KeyScan_Task(void* parameter);//按键扫描任务
static void Motor_Task(void* parameter);//电机控制任务
static void ListeningSensors_Task(void* parameter);//监听两个测速传感器
static void lCalcVelocity_Task(void* parameter);//计算左轮速度任务
static void rCalcVelocity_Task(void* parameter);//计算右轮速度任务
static void AnalyseCommand_Task(void* parameter);//分析命令任务
static void PIDCalculator_Task(void* parameter);//分析命令任务
static void Setup(void);/* 用于初始化板载相关资源 */



int main(void){	
	Setup();//初始化
	/* 创建 AppTaskCreate 任务 */
	AppTaskCreate_Handle = xTaskCreateStatic((TaskFunction_t	)AppTaskCreate,		//任务函数
                                         	(const char* 	)"AppTaskCreate",		//任务名称
											(uint32_t 		)128,					//任务堆栈大小
											(void* 		  	)NULL,					//传递给任务函数的参数
											(UBaseType_t 	)3, 					//任务优先级
											(StackType_t*   )AppTaskCreate_Stack,	//任务堆栈
											(StaticTask_t*  )&AppTaskCreate_TCB);	//任务控制块	
										
	if(AppTaskCreate_Handle != NULL){
		//任务创建成功
		printf("=====准备进入FreeRTOS!=====\r\n");
		vTaskStartScheduler();   /* 开启调度器 */
	}
	while(1);
}

/**
  * @brief    板载外设初始化
  * @param    void
  * @retval   void
  */
static void Setup(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//4bit都用于设置抢占优先级
	delay_init();
	MTS_Init();
	LED_Init();	  		//LED初始化
	USART1_Init(115200);//串口1初始化
	Key_Init();
	TIM2_Int_Init(10-1,7200-1);//定时器时钟72M，分频系数7200，所以72M/7200=10Khz的计数频率，计数10次为1ms
	vSensors_Init();
	USART2_Init(115200);
//	Zigbee_Change_Mode(1);
}

static void AppTaskCreate(void){
	taskENTER_CRITICAL();           //进入临界区
	//串口打印互斥信号量
	printfSemphr_Handle = xSemaphoreCreateMutexStatic(&printfSemphr_Structure);//创建互斥信号量
	if(printfSemphr_Handle != NULL)
		printf("串口打印互斥信号量创建成功~\r\n");
	else
		printf("串口打印互斥创建失败~\r\n");

	//记录左车轮光电测速传感器获取的脉冲
	vSensorLCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,			//最大计数值
														 0,							//初始计数值
														 &vSensorLCount_Structure);	//信号量的数据结构体
	if(vSensorLCountHandle != NULL)/* 创建成功 */
		printf("vSensorLCount 计数信号量创建成功!\r\n");
	else
		printf("vSensorLCount 计数信号量创建失败!\r\n");

	//记录右车轮光电测速传感器获取的脉冲
	vSensorRCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,			//最大计数值
														 0,							//初始计数值
														 &vSensorRCount_Structure);	//信号量的数据结构体
	if(vSensorRCountHandle != NULL)/* 创建成功 */
		printf("vSensorRCount 计数信号量创建成功!\r\n");
	else
		printf("vSensorRCount 计数信号量创建失败!\r\n");
	
	//存储车轮待修正车速值
	ReplaceVHandle = xQueueCreateStatic(REPLACE_V_QUEUE_LENGTH,//队列深度
										REPLACE_V_ITEM_SIZE,	//队列数据单元的单位
										ReplaceVStorageArea,	//队列的存储区域
										&ReplaceV_Structure	//队列的数据结构
										);	
	if(ReplaceVHandle != NULL)/* 创建成功 */
		printf("ReplaceV队列创建成功!\r\n");
	else
		printf("ReplaceV队列创建失败!\r\n");

	//存储控制命令
	wirelessCommandHandle = xQueueCreateStatic(	WIRELESSCOMMAND_QUEUE_LENGTH,	//队列深度
												WIRELESSCOMMAND_ITEM_SIZE,		//队列数据单元的单位
												wirelessCommandStorageArea,		//队列的存储区域
												&wirelessCommand_Structure		//队列的数据结构
										);
	if(wirelessCommandHandle != NULL)/* 创建成功 */
		printf("wirelessCommand队列创建成功!\r\n");
	else
		printf("wirelessCommand队列创建失败!\r\n");

	/* 创建LED_Task任务 */
	LED_Task_Handle = xTaskCreateStatic((TaskFunction_t	)LED_Task,			//任务函数
                                        (const char* 	)"LED_Task",		//任务名称
                                        (uint32_t 		)128,				//任务栈深
                                        (void* 		  	)NULL,				//传递给任务函数的参数
                                        (UBaseType_t 	)4, 				//任务优先级
                                        (StackType_t*   )LED_Task_Stack,	//任务堆栈
                                        (StaticTask_t*  )&LED_Task_TCB);	//任务控制块   
	if(LED_Task_Handle != NULL)/* 创建成功 */
		printf("LED_Task任务创建成功!\r\n");
	else
		printf("LED_Task任务创建失败!\r\n");

	/* 创建Motor_Task任务 */
	Motor_Task_Handle = xTaskCreateStatic((TaskFunction_t	)Motor_Task,			//任务函数
                                        (const char* 	)"Motor_Task",		//任务名称
                                        (uint32_t 		)128,				//任务栈深
                                        (void* 		  	)NULL,				//传递给任务函数的参数
                                        (UBaseType_t 	)4, 				//任务优先级
                                        (StackType_t*   )Motor_Task_Stack,	//任务堆栈
                                        (StaticTask_t*  )&Motor_Task_TCB);	//任务控制块   
	if(Motor_Task_Handle != NULL)/* 创建成功 */
		printf("Motor_Task任务创建成功!\r\n");
	else
		printf("Motor_Task任务创建失败!\r\n");
	
	/* 创建 ListeningSensors_Task 任务 */
	ListeningSensors_Task_Handle = xTaskCreateStatic((TaskFunction_t	)ListeningSensors_Task,			//任务函数
                                        (const char* 	)"ListeningSensors_Task",		//任务名称
                                        (uint32_t 		)128,				//任务栈深
                                        (void* 		  	)NULL,				//传递给任务函数的参数
                                        (UBaseType_t 	)4, 				//任务优先级
                                        (StackType_t*   )ListeningSensors_Task_Stack,	//任务堆栈
                                        (StaticTask_t*  )&ListeningSensors_Task_TCB);	//任务控制块   
	if(ListeningSensors_Task_Handle != NULL)/* 创建成功 */
		printf("ListeningSensors_Task任务创建成功!\r\n");
	else
		printf("ListeningSensors_Task任务创建失败!\r\n");
	
	/* 创建 lCalcVelocity_Task 任务 */
	lCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t	)lCalcVelocity_Task,			//任务函数
                                        (const char* 	)"lCalcVelocity_Task",		//任务名称
                                        (uint32_t 		)128,				//任务栈深
                                        (void* 		  	)NULL,				//传递给任务函数的参数
                                        (UBaseType_t 	)4, 				//任务优先级
                                        (StackType_t*   )lCalcVelocity_Task_Stack,	//任务堆栈
                                        (StaticTask_t*  )&lCalcVelocity_Task_TCB);	//任务控制块   
	if(lCalcVelocity_Task_Handle != NULL)/* 创建成功 */
		printf("lCalcVelocity_Task任务创建成功!\r\n");
	else
		printf("lCalcVelocity_Task任务创建失败!\r\n");
	
	/* 创建 rCalcVelocity_Task 任务 */
	rCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t	)rCalcVelocity_Task,			//任务函数
                                        (const char* 	)"rCalcVelocity_Task",		//任务名称
                                        (uint32_t 		)128,				//任务栈深
                                        (void* 		  	)NULL,				//传递给任务函数的参数
                                        (UBaseType_t 	)4, 				//任务优先级
                                        (StackType_t*   )rCalcVelocity_Task_Stack,	//任务堆栈
                                        (StaticTask_t*  )&rCalcVelocity_Task_TCB);	//任务控制块   
	if(rCalcVelocity_Task_Handle != NULL)/* 创建成功 */
		printf("rCalcVelocity_Task任务创建成功!\r\n");
	else
		printf("rCalcVelocity_Task任务创建失败!\r\n");
	
	/* 创建 AnalyseCommand_Task 任务 */
	AnalyseCommand_Task_Handle = xTaskCreateStatic((TaskFunction_t	)AnalyseCommand_Task,			//任务函数
                                        (const char* 	)"AnalyseCommand_Task",		//任务名称
                                        (uint32_t 		)128,				//任务栈深
                                        (void* 		  	)NULL,				//传递给任务函数的参数
                                        (UBaseType_t 	)4, 				//任务优先级
                                        (StackType_t*   )AnalyseCommand_Task_Stack,	//任务堆栈
                                        (StaticTask_t*  )&AnalyseCommand_Task_TCB);	//任务控制块   
	if(AnalyseCommand_Task_Handle != NULL)/* 创建成功 */
		printf("AnalyseCommand任务创建成功!\r\n");
	else
		printf("AnalyseCommand任务创建失败!\r\n");
	
	/* 创建 PIDCalculator_Task 任务 */
	PIDCalculator_Task_Handle = xTaskCreateStatic((TaskFunction_t	)PIDCalculator_Task,			//任务函数
                                        (const char* 	)"PIDCalculator_Task",		//任务名称
                                        (uint32_t 		)128,				//任务栈深
                                        (void* 		  	)NULL,				//传递给任务函数的参数
                                        (UBaseType_t 	)4, 				//任务优先级
                                        (StackType_t*   )PIDCalculator_Task_Stack,	//任务堆栈
                                        (StaticTask_t*  )&PIDCalculator_Task_TCB);	//任务控制块   
	if(PIDCalculator_Task_Handle != NULL)/* 创建成功 */
		printf("PIDCalculator任务创建成功!\r\n");
	else
		printf("PIDCalculator任务创建失败!\r\n");

	vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
	vTaskSuspend(Motor_Task_Handle);//挂起电机任务，因为车还没启动
	vTaskSuspend(lCalcVelocity_Task_Handle);//挂起测速任务，因为车还没启动
	vTaskSuspend(rCalcVelocity_Task_Handle);//挂起测速任务，因为车还没启动
	printf("======进入FreeRTOS!======\r\n");
	taskEXIT_CRITICAL();            //退出临界区
}

/**

  * @brief PID计算器 任务主体
  * @brief 时刻计算需要计算的数据   
  * @param    
  * @retval    
  */
static void PIDCalculator_Task(void* parameter){
	float kp = 0.5;
	float ki = 0.01;
	float kd = 0;
	PID* lVelocityPID = PID_Create_Object(kp,ki,kd);
	PID* rVelocityPID = PID_Create_Object(kp,ki,kd);
	while(1){
		if(lTargetV != lReplaceV){
			lReplaceV += PID_Classic(lVelocityPID, lTargetV-lReplaceV);
		}
		if(rTargetV != rReplaceV){
			rReplaceV += PID_Classic(rVelocityPID, rTargetV-rReplaceV);
		}
	}
}


/**
  * @brief    分析控制命令任务主体
  * @param    parameter
  * @retval   void
  */
static void AnalyseCommand_Task(void* parameter){
	BaseType_t xReturn = pdPASS;
	u8 commands[WIRELESSCOMMAND_QUEUE_LENGTH];
	u8 i;
	while(1){
		for(i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++){
			xReturn = xQueueReceive(wirelessCommandHandle, &commands[i], portMAX_DELAY);
		}
		if(xReturn != pdPASS)
			xPrintf("命令消息接收失败\r\n");
		printf("成功接收控制命令：%d\r\n",commands[1]);
		
		if(commands[0] == 0x00){
			xPrintf("停车~\r\n");
			vTaskSuspend(lCalcVelocity_Task_Handle);//挂起测速任务，因为车还没启动
			vTaskSuspend(rCalcVelocity_Task_Handle);//挂起测速任务，因为车还没启动
			PWMVal[0] = 0;
			PWMVal[1] = 0;
			MTLEN(TIM3,0);
			MTREN(TIM3,0);
			lVelocity = 0;
			rVelocity = 0;
			lTargetV = 0;
			rTargetV = 0;
			vTaskSuspend(Motor_Task_Handle);//挂起电机任务，因为车还没启动
		}
		else if(commands[0] == 0x01){
			if(commands[1] == 0){
				printf("开车失败，速度需大于0~\r\n");
			}
			else{
				lTargetV = commands[1]/100 + (commands[1]%100)*0.01;
				rTargetV = lTargetV;
				printf("开车！目标车速：%.2lfm/s\r\n",lTargetV);
				vTaskResume(lCalcVelocity_Task_Handle);//解挂测速任务，车已启动
				vTaskResume(rCalcVelocity_Task_Handle);//解挂测速任务，车已启动
				vTaskResume(Motor_Task_Handle);//解挂电机任务
			}
		}
		else if(commands[0] == 0x02){
			lTargetV = commands[1]/100 + (commands[1]%100)*0.01;
			rTargetV = commands[2]/100 + (commands[2]%100)*0.01;
			printf("调速成功~左右车轮目标车速为：%.2lfm/s  %.2lfm/s\r\n",lTargetV,rTargetV);
			vTaskResume(lCalcVelocity_Task_Handle);//解挂测速任务，车已启动
			vTaskResume(rCalcVelocity_Task_Handle);//解挂测速任务，车已启动
			vTaskResume(Motor_Task_Handle);//解挂电机任务
		}
	}
}

float filterNum = 0.01f;
/**
  * @brief    计算速度任务主体
  * @brief    计算每秒测速传感器
  * @param    parameter
  * @retval   void
  */
static void lCalcVelocity_Task(void* parameter){
	u16 lCounter = 0;
	u16 i;
	while(1){
		xSemaphoreTake(vSensorLCountHandle,portMAX_DELAY);
		lCounter = uxSemaphoreGetCount(vSensorLCountHandle);
		for(i = 0; i < lCounter; i++){
			xSemaphoreTake(vSensorLCountHandle,0);
		}
		lCounter++;
		//计算两个轮子的速度
		lVelocity = Filter((ONEPULSEDISTANCE * lCounter * 1000) / lTime, lVelocity, filterNum);
		lTime = 0;
	}
}

/**
  * @brief    计算速度任务主体
  * @brief    计算每秒测速传感器
  * @param    parameter
  * @retval   void
  */
static void rCalcVelocity_Task(void* parameter){
	u16 rCounter;
	u16 i;
	while(1){
		xSemaphoreTake(vSensorRCountHandle,portMAX_DELAY);
		rCounter = uxSemaphoreGetCount(vSensorRCountHandle);
		for(i = 0; i < rCounter; i++){
			xSemaphoreTake(vSensorRCountHandle,0);
		}
		rCounter++;
		//计算两个轮子的速度
		rVelocity = Filter((ONEPULSEDISTANCE*rCounter * rCounter * 1000) / rTime, rVelocity, filterNum);
		rTime = 0;
	}
}

/**
  * @brief    监听测速传感器任务主体
  * @param    parameter
  * @retval   void
  */
static void ListeningSensors_Task(void* parameter){
	BaseType_t xReturn = pdPASS;
	u8 L = 0;
	u8 R = 0;
	while(1){
		if(L != GPIO_ReadInputDataBit(VSENSORL_GPIO,VSENSORL_Pin)){
			if(L == 1){
				xReturn = xSemaphoreGive(vSensorLCountHandle);
				if(xReturn != pdTRUE)
					xPrintf("左测速器信号量赋值失败，错误码为：%d\r\n",xReturn);
			}
			L = !L;
		}
		if(R != GPIO_ReadInputDataBit(VSENSORR_GPIO,VSENSORR_Pin)){
			if(R == 1){
				xReturn = xSemaphoreGive(vSensorRCountHandle);
				if(xReturn != pdTRUE)
					xPrintf("右测速器信号量赋值失败，错误码为：%d\r\n",xReturn);
			}
			R = !R;
		}
	}
}

/**
  * @brief    电机转速控制任务主体
  * @param    parameter
  * @retval   void
  */

static void Motor_Task(void* parameter){
	while(1){
		vTaskDelay(1);
		if(lReplaceV < lVelocity && PWMVal[0] > 0){
			PWMVal[0]--;
			MTLEN(TIM3,PWMVal[0]);
		}
		else if(lReplaceV > lVelocity && PWMVal[0] < 100){
			PWMVal[0]++;
			MTLEN(TIM3,PWMVal[0]);
		}

		if(rReplaceV < rVelocity && PWMVal[1] > 0){
			PWMVal[1]--;
			MTREN(TIM3,PWMVal[1]);
		}
		else if(rReplaceV > rVelocity && PWMVal[1] < 100){
			PWMVal[1]++;
			MTREN(TIM3,PWMVal[1]);
		}
		printf("左轮速度为:%lfm/s  右轮速度为:%lfm/s\r\n",lVelocity,rVelocity);
	}
}


/**
  * @brief    LED_Task任务主体
  * @param    parameter
  * @retval   void
  */
static void LED_Task(void* parameter){
	while(1){
		TestLED = 0;
		vTaskDelay(500/portTICK_PERIOD_MS);/*延时500ms*/
		TestLED = 1;
		vTaskDelay(500/portTICK_PERIOD_MS);/*延时500ms*/
	}
}


/**
  * @brief    优化FreeRTOS的printf函数
  * @param    parameter
  * @retval   void
  */
void xPrintf(char *format, ...)
{
    char  buf_str[200 + 1];
    va_list   v_args;

    va_start(v_args, format);
   (void)vsnprintf((char       *)&buf_str[0],
                   (size_t      ) sizeof(buf_str),
                   (char const *) format,
                                  v_args);
    va_end(v_args);

	/* 互斥信号量 */
	xSemaphoreTake(printfSemphr_Handle, portMAX_DELAY);

    printf("%s", buf_str);

  	xSemaphoreGive(printfSemphr_Handle);
}


/**
  * @brief    一阶低通滤波算法
  * @param    newValue: 新采样的数据
  * @param    oldValue: 上一个滤波输出值
  * @param    alpha   : 滤波系数
  * @retval   本次滤波输出值
  */
portFLOAT Filter(portFLOAT newValue, portFLOAT oldValue, portFLOAT alpha){
	newValue = alpha*newValue+(1-alpha)*oldValue;
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
	*ppxIdleTaskTCBBuffer=&Idle_Task_TCB;/* 任务控制块内存 */
	*ppxIdleTaskStackBuffer=Idle_Task_Stack;/* 任务堆栈内存 */
	*pulIdleTaskStackSize=configMINIMAL_STACK_SIZE;/* 任务堆栈大小 */
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
	*ppxTimerTaskTCBBuffer=&Timer_Task_TCB;/* 任务控制块内存 */
	*ppxTimerTaskStackBuffer=Timer_Task_Stack;/* 任务堆栈内存 */
	*pulTimerTaskStackSize=configTIMER_TASK_STACK_DEPTH;/* 任务堆栈大小 */
}

/**
  * @brief		串口1中断服务程序
  * @param		void
  * @retval		void
  */
void USART1_IRQHandler(void){
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){//接收中断
		if(USART1_RX_STA < USART_REC_LEN){
			usart1RXTime = 0;
			USART1_RX_BUF[USART1_RX_STA] = USART_ReceiveData(USART1);//读取接收到的数据
			USART1_RX_STA++;
		}
		else{
			USART1_RX_STA = 0;
		}
	}
}

/**
  * @brief		串口2中断服务程序
  * @param		void
  * @retval		void
  */
void USART2_IRQHandler(void){
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){//接收中断
		if(USART2_RX_STA < USART_REC_LEN){
			usart2RXTime = 0;
			USART2_RX_BUF[USART2_RX_STA] = USART_ReceiveData(USART2);//读取接收到的数据
			USART2_RX_STA++;
		}
		else{
			USART2_RX_STA = 0;
		}
	}
}

//定时器2中断服务函数
//每毫秒触发一次中断
void TIM2_IRQHandler(void){
	BaseType_t xReturn = pdPASS;
	u8 i;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET){ //溢出中断
		lTime++;
		rTime++;
		if(lTime == 500)
			lVelocity = 0;
		if(rTime == 500)
			rVelocity = 0;
		if(usart1RXTime == 10){
			//测试，会将串口收到的所有内容返回
			printf("Usart1 Recive over~\r\n");
			for(u8 i = 0; i < USART1_RX_STA; i++){
				printf("%lX ", USART1_RX_BUF[i]);
			}
			printf("\r\n");
			/*这下面写串口消息接收完的逻辑判断*/
//			if(USART1_RX_BUF[0] == 0x55)
//				Zigbee_Analyse_Command_Data();
			if(USART1_RX_BUF[0] == 0xC1 && USART1_RX_BUF[1] == 0xC2 && USART1_RX_BUF[2] == 0xC3 && USART1_RX_BUF[3] == 0xC4){
				for(i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++){
					xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART1_RX_BUF[4+i], NULL);
				}
				if(xReturn != pdTRUE)
					printf("控制命令消息发送失败");
			}
			USART1_RX_STA = 0;
			usart1RXTime = 0xFF;//把时间拉满，表示没有收到新的消息
		}
		
		if(usart2RXTime == 10){
			//测试，会将串口收到的所有内容返回
			printf("Usart2 Recive over~\r\n");
			for(u8 i = 0; i < USART2_RX_STA; i++){
				printf("%lX ", USART2_RX_BUF[i]);
			}
			printf("\r\n");
			/*这下面写串口消息接收完的逻辑判断*/
//			if(USART1_RX_BUF[0] == 0x55)
//				Zigbee_Analyse_Command_Data();
			if(USART2_RX_BUF[0] == 0xC1 && USART2_RX_BUF[1] == 0xC2 && USART2_RX_BUF[2] == 0xC3 && USART2_RX_BUF[3] == 0xC4){
				for(i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++){
					xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART2_RX_BUF[4+i], NULL);
				}
				if(xReturn != pdTRUE)
					printf("控制命令消息发送失败");
			}
			USART2_RX_STA = 0;
			usart2RXTime = 0xFF;//把时间拉满，表示没有收到新的消息
		}

		if(usart1RXTime < 10)
			usart1RXTime++;

		if(usart2RXTime < 10)
			usart2RXTime++;
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
	}
}
