#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sys.h"
#include "usart.h"
#include  <stdarg.h>//��׼C���ļ�,�ú����ܹ����տɱ����
#include "led.h"
#include "key.h"
#include "motor.h"
#include "vSensor.h"
#include "PID.h"

/******************************* �궨�� ************************************/
/*�����Դ洢 2 �� u8���ͱ����Ķ��� */
#define REPLACE_V_QUEUE_LENGTH 2
#define REPLACE_V_ITEM_SIZE sizeof(u8)

/*�����Դ洢 3 �� u8���ͱ����Ķ��� */
#define WIRELESSCOMMAND_QUEUE_LENGTH 3
#define WIRELESSCOMMAND_ITEM_SIZE sizeof(u8)
/**************************** ������ ********************************/L��
/* AppTaskCreate ������ */
static TaskHandle_t AppTaskCreate_Handle = NULL;
/* LED ������ */
static TaskHandle_t LED_Task_Handle = NULL;
/* KeyScan ������*/
//static TaskHandle_t KeyScan_Task_Handle = NULL;
/* Motor ������*/
static TaskHandle_t Motor_Task_Handle = NULL;
/* ListeningSensors ������*/
static TaskHandle_t ListeningSensors_Task_Handle = NULL;
/* lCalcVelocity ������*/
static TaskHandle_t lCalcVelocity_Task_Handle = NULL;
/* rCalcVelocity ������*/
static TaskHandle_t rCalcVelocity_Task_Handle = NULL;
/* AnalyseCommand ������*/
static TaskHandle_t AnalyseCommand_Task_Handle = NULL;
/* PIDCalculator ������*/
static TaskHandle_t PIDCalculator_Task_Handle = NULL;
/********************************** �ں˶����� *********************************/
static SemaphoreHandle_t printfSemphr_Handle = NULL;/* ���ڴ�ӡ�����ź������*/

static SemaphoreHandle_t vSensorLCountHandle = NULL;//����ٴ����������ź������
static SemaphoreHandle_t vSensorRCountHandle = NULL;//�Ҳ��ٴ����������ź������

static QueueHandle_t ReplaceVHandle = NULL;//���ִ������ٶ���Ϣ���о��

static QueueHandle_t wirelessCommandHandle = NULL;//���߿���������Ϣ���о��
/******************************* ȫ�ֱ������� ************************************/
/* �������������ջ */
static StackType_t Idle_Task_Stack[configMINIMAL_STACK_SIZE];
/* ��ʱ�������ջ */
static StackType_t Timer_Task_Stack[configTIMER_TASK_STACK_DEPTH];
/* ����������ƿ� */
static StaticTask_t Idle_Task_TCB;	
/* ��ʱ��������ƿ� */
static StaticTask_t Timer_Task_TCB;

/* AppTaskCreate �����ջ */
static StackType_t AppTaskCreate_Stack[128];
/* LED �����ջ */
static StackType_t LED_Task_Stack[128];
/* KeyScan �����ջ */
//static StackType_t KeyScan_Task_Stack[128];
/* Motor �����ջ */
static StackType_t Motor_Task_Stack[128];
/* ListeningSensors �����ջ */
static StackType_t ListeningSensors_Task_Stack[128];
/* lCalcVelocity �����ջ */
static StackType_t lCalcVelocity_Task_Stack[128];
/* rCalcVelocity �����ջ */
static StackType_t rCalcVelocity_Task_Stack[128];
/* AnalyseCommand �����ջ */
static StackType_t AnalyseCommand_Task_Stack[128];
/* PIDCalculator �����ջ */
static StackType_t PIDCalculator_Task_Stack[128];

/* AppTaskCreate ������ƿ� */
static StaticTask_t AppTaskCreate_TCB;
/* LED ������ƿ� */
static StaticTask_t LED_Task_TCB;
/* KeyScan ������ƿ� */
//static StaticTask_t KeyScan_Task_TCB;
/* Motor ������ƿ� */
static StaticTask_t Motor_Task_TCB;
/* ListeningSensors ������ƿ� */
static StaticTask_t ListeningSensors_Task_TCB;
/* lCalcVelocity ������ƿ� */
static StaticTask_t lCalcVelocity_Task_TCB;
/* rCalcVelocity ������ƿ� */
static StaticTask_t rCalcVelocity_Task_TCB;
/* AnalyseCommand ������ƿ� */
static StaticTask_t AnalyseCommand_Task_TCB;
/* PIDCalculator ������ƿ� */
static StaticTask_t PIDCalculator_Task_TCB;


/* �ź������ݽṹָ�� */
static StaticSemaphore_t printfSemphr_Structure;/*���ڴ�ӡ�����ź���*/
static StaticSemaphore_t vSensorLCount_Structure;/*����ٴ����������ź���*/
static StaticSemaphore_t vSensorRCount_Structure;/*�Ҳ��ٴ����������ź���*/

/* ��Ϣ�������ݽṹָ�� */
static StaticQueue_t ReplaceV_Structure;
static StaticQueue_t wirelessCommand_Structure;

/* ��Ϣ���еĴ洢���򣬴�С������ uxQueueLength * uxItemSize ���ֽ� */
uint8_t ReplaceVStorageArea[ REPLACE_V_QUEUE_LENGTH * REPLACE_V_ITEM_SIZE ];
uint8_t wirelessCommandStorageArea[ WIRELESSCOMMAND_QUEUE_LENGTH * WIRELESSCOMMAND_ITEM_SIZE ]; 

/* ��ͨȫ�ֱ��� */
portFLOAT lVelocity;//�����ٶ�(m/s)
portFLOAT rVelocity;//�����ٶ�(m/s)

portFLOAT lReplaceV;//Ŀ�������ٶ�
portFLOAT rReplaceV;//Ŀ�������ٶ�

portFLOAT lTargetV;//���滻�������ٶ�
portFLOAT rTargetV;//���滻�������ٶ�

u16 lTime = 0;//����1�����ʱ
u16 rTime = 0;//����1�����ʱ

u16 PWMVal[2] = {0};
/*
*************************************************************************
*                             ��������
*************************************************************************
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, 
								   StackType_t **ppxIdleTaskStackBuffer, 
								   uint32_t *pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, 
									StackType_t **ppxTimerTaskStackBuffer, 
									uint32_t *pulTimerTaskStackSize);
static void xPrintf(char *format, ...);
portFLOAT Filter(portFLOAT newValue, portFLOAT oldValue, portFLOAT alpha);
static void AppTaskCreate(void);/* ���ڴ������� */
static void LED_Task(void* parameter);//LED����
//static void KeyScan_Task(void* parameter);//����ɨ������
static void Motor_Task(void* parameter);//�����������
static void ListeningSensors_Task(void* parameter);//�����������ٴ�����
static void lCalcVelocity_Task(void* parameter);//���������ٶ�����
static void rCalcVelocity_Task(void* parameter);//���������ٶ�����
static void AnalyseCommand_Task(void* parameter);//������������
static void PIDCalculator_Task(void* parameter);//������������
static void Setup(void);/* ���ڳ�ʼ�����������Դ */



int main(void){	
	Setup();//��ʼ��
	/* ���� AppTaskCreate ���� */
	AppTaskCreate_Handle = xTaskCreateStatic((TaskFunction_t	)AppTaskCreate,		//������
                                         	(const char* 	)"AppTaskCreate",		//��������
											(uint32_t 		)128,					//�����ջ��С
											(void* 		  	)NULL,					//���ݸ��������Ĳ���
											(UBaseType_t 	)3, 					//�������ȼ�
											(StackType_t*   )AppTaskCreate_Stack,	//�����ջ
											(StaticTask_t*  )&AppTaskCreate_TCB);	//������ƿ�	
										
	if(AppTaskCreate_Handle != NULL){
		//���񴴽��ɹ�
		printf("=====׼������FreeRTOS!=====\r\n");
		vTaskStartScheduler();   /* ���������� */
	}
	while(1);
}

/**
  * @brief    ���������ʼ��
  * @param    void
  * @retval   void
  */
static void Setup(void){
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//4bit������������ռ���ȼ�
	delay_init();
	MTS_Init();
	LED_Init();	  		//LED��ʼ��
	USART1_Init(115200);//����1��ʼ��
	Key_Init();
	TIM2_Int_Init(10-1,7200-1);//��ʱ��ʱ��72M����Ƶϵ��7200������72M/7200=10Khz�ļ���Ƶ�ʣ�����10��Ϊ1ms
	vSensors_Init();
	USART2_Init(115200);
//	Zigbee_Change_Mode(1);
}

static void AppTaskCreate(void){
	taskENTER_CRITICAL();           //�����ٽ���
	//���ڴ�ӡ�����ź���
	printfSemphr_Handle = xSemaphoreCreateMutexStatic(&printfSemphr_Structure);//���������ź���
	if(printfSemphr_Handle != NULL)
		printf("���ڴ�ӡ�����ź��������ɹ�~\r\n");
	else
		printf("���ڴ�ӡ���ⴴ��ʧ��~\r\n");

	//��¼���ֹ����ٴ�������ȡ������
	vSensorLCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,			//������ֵ
														 0,							//��ʼ����ֵ
														 &vSensorLCount_Structure);	//�ź��������ݽṹ��
	if(vSensorLCountHandle != NULL)/* �����ɹ� */
		printf("vSensorLCount �����ź��������ɹ�!\r\n");
	else
		printf("vSensorLCount �����ź�������ʧ��!\r\n");

	//��¼�ҳ��ֹ����ٴ�������ȡ������
	vSensorRCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,			//������ֵ
														 0,							//��ʼ����ֵ
														 &vSensorRCount_Structure);	//�ź��������ݽṹ��
	if(vSensorRCountHandle != NULL)/* �����ɹ� */
		printf("vSensorRCount �����ź��������ɹ�!\r\n");
	else
		printf("vSensorRCount �����ź�������ʧ��!\r\n");
	
	//�洢���ִ���������ֵ
	ReplaceVHandle = xQueueCreateStatic(REPLACE_V_QUEUE_LENGTH,//�������
										REPLACE_V_ITEM_SIZE,	//�������ݵ�Ԫ�ĵ�λ
										ReplaceVStorageArea,	//���еĴ洢����
										&ReplaceV_Structure	//���е����ݽṹ
										);	
	if(ReplaceVHandle != NULL)/* �����ɹ� */
		printf("ReplaceV���д����ɹ�!\r\n");
	else
		printf("ReplaceV���д���ʧ��!\r\n");

	//�洢��������
	wirelessCommandHandle = xQueueCreateStatic(	WIRELESSCOMMAND_QUEUE_LENGTH,	//�������
												WIRELESSCOMMAND_ITEM_SIZE,		//�������ݵ�Ԫ�ĵ�λ
												wirelessCommandStorageArea,		//���еĴ洢����
												&wirelessCommand_Structure		//���е����ݽṹ
										);
	if(wirelessCommandHandle != NULL)/* �����ɹ� */
		printf("wirelessCommand���д����ɹ�!\r\n");
	else
		printf("wirelessCommand���д���ʧ��!\r\n");

	/* ����LED_Task���� */
	LED_Task_Handle = xTaskCreateStatic((TaskFunction_t	)LED_Task,			//������
                                        (const char* 	)"LED_Task",		//��������
                                        (uint32_t 		)128,				//����ջ��
                                        (void* 		  	)NULL,				//���ݸ��������Ĳ���
                                        (UBaseType_t 	)4, 				//�������ȼ�
                                        (StackType_t*   )LED_Task_Stack,	//�����ջ
                                        (StaticTask_t*  )&LED_Task_TCB);	//������ƿ�   
	if(LED_Task_Handle != NULL)/* �����ɹ� */
		printf("LED_Task���񴴽��ɹ�!\r\n");
	else
		printf("LED_Task���񴴽�ʧ��!\r\n");

	/* ����Motor_Task���� */
	Motor_Task_Handle = xTaskCreateStatic((TaskFunction_t	)Motor_Task,			//������
                                        (const char* 	)"Motor_Task",		//��������
                                        (uint32_t 		)128,				//����ջ��
                                        (void* 		  	)NULL,				//���ݸ��������Ĳ���
                                        (UBaseType_t 	)4, 				//�������ȼ�
                                        (StackType_t*   )Motor_Task_Stack,	//�����ջ
                                        (StaticTask_t*  )&Motor_Task_TCB);	//������ƿ�   
	if(Motor_Task_Handle != NULL)/* �����ɹ� */
		printf("Motor_Task���񴴽��ɹ�!\r\n");
	else
		printf("Motor_Task���񴴽�ʧ��!\r\n");
	
	/* ���� ListeningSensors_Task ���� */
	ListeningSensors_Task_Handle = xTaskCreateStatic((TaskFunction_t	)ListeningSensors_Task,			//������
                                        (const char* 	)"ListeningSensors_Task",		//��������
                                        (uint32_t 		)128,				//����ջ��
                                        (void* 		  	)NULL,				//���ݸ��������Ĳ���
                                        (UBaseType_t 	)4, 				//�������ȼ�
                                        (StackType_t*   )ListeningSensors_Task_Stack,	//�����ջ
                                        (StaticTask_t*  )&ListeningSensors_Task_TCB);	//������ƿ�   
	if(ListeningSensors_Task_Handle != NULL)/* �����ɹ� */
		printf("ListeningSensors_Task���񴴽��ɹ�!\r\n");
	else
		printf("ListeningSensors_Task���񴴽�ʧ��!\r\n");
	
	/* ���� lCalcVelocity_Task ���� */
	lCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t	)lCalcVelocity_Task,			//������
                                        (const char* 	)"lCalcVelocity_Task",		//��������
                                        (uint32_t 		)128,				//����ջ��
                                        (void* 		  	)NULL,				//���ݸ��������Ĳ���
                                        (UBaseType_t 	)4, 				//�������ȼ�
                                        (StackType_t*   )lCalcVelocity_Task_Stack,	//�����ջ
                                        (StaticTask_t*  )&lCalcVelocity_Task_TCB);	//������ƿ�   
	if(lCalcVelocity_Task_Handle != NULL)/* �����ɹ� */
		printf("lCalcVelocity_Task���񴴽��ɹ�!\r\n");
	else
		printf("lCalcVelocity_Task���񴴽�ʧ��!\r\n");
	
	/* ���� rCalcVelocity_Task ���� */
	rCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t	)rCalcVelocity_Task,			//������
                                        (const char* 	)"rCalcVelocity_Task",		//��������
                                        (uint32_t 		)128,				//����ջ��
                                        (void* 		  	)NULL,				//���ݸ��������Ĳ���
                                        (UBaseType_t 	)4, 				//�������ȼ�
                                        (StackType_t*   )rCalcVelocity_Task_Stack,	//�����ջ
                                        (StaticTask_t*  )&rCalcVelocity_Task_TCB);	//������ƿ�   
	if(rCalcVelocity_Task_Handle != NULL)/* �����ɹ� */
		printf("rCalcVelocity_Task���񴴽��ɹ�!\r\n");
	else
		printf("rCalcVelocity_Task���񴴽�ʧ��!\r\n");
	
	/* ���� AnalyseCommand_Task ���� */
	AnalyseCommand_Task_Handle = xTaskCreateStatic((TaskFunction_t	)AnalyseCommand_Task,			//������
                                        (const char* 	)"AnalyseCommand_Task",		//��������
                                        (uint32_t 		)128,				//����ջ��
                                        (void* 		  	)NULL,				//���ݸ��������Ĳ���
                                        (UBaseType_t 	)4, 				//�������ȼ�
                                        (StackType_t*   )AnalyseCommand_Task_Stack,	//�����ջ
                                        (StaticTask_t*  )&AnalyseCommand_Task_TCB);	//������ƿ�   
	if(AnalyseCommand_Task_Handle != NULL)/* �����ɹ� */
		printf("AnalyseCommand���񴴽��ɹ�!\r\n");
	else
		printf("AnalyseCommand���񴴽�ʧ��!\r\n");
	
	/* ���� PIDCalculator_Task ���� */
	PIDCalculator_Task_Handle = xTaskCreateStatic((TaskFunction_t	)PIDCalculator_Task,			//������
                                        (const char* 	)"PIDCalculator_Task",		//��������
                                        (uint32_t 		)128,				//����ջ��
                                        (void* 		  	)NULL,				//���ݸ��������Ĳ���
                                        (UBaseType_t 	)4, 				//�������ȼ�
                                        (StackType_t*   )PIDCalculator_Task_Stack,	//�����ջ
                                        (StaticTask_t*  )&PIDCalculator_Task_TCB);	//������ƿ�   
	if(PIDCalculator_Task_Handle != NULL)/* �����ɹ� */
		printf("PIDCalculator���񴴽��ɹ�!\r\n");
	else
		printf("PIDCalculator���񴴽�ʧ��!\r\n");

	vTaskDelete(AppTaskCreate_Handle); //ɾ��AppTaskCreate����
	vTaskSuspend(Motor_Task_Handle);//������������Ϊ����û����
	vTaskSuspend(lCalcVelocity_Task_Handle);//�������������Ϊ����û����
	vTaskSuspend(rCalcVelocity_Task_Handle);//�������������Ϊ����û����
	printf("======����FreeRTOS!======\r\n");
	taskEXIT_CRITICAL();            //�˳��ٽ���
}

/**

  * @brief PID������ ��������
  * @brief ʱ�̼�����Ҫ���������   
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
  * @brief    ��������������������
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
			xPrintf("������Ϣ����ʧ��\r\n");
		printf("�ɹ����տ������%d\r\n",commands[1]);
		
		if(commands[0] == 0x00){
			xPrintf("ͣ��~\r\n");
			vTaskSuspend(lCalcVelocity_Task_Handle);//�������������Ϊ����û����
			vTaskSuspend(rCalcVelocity_Task_Handle);//�������������Ϊ����û����
			PWMVal[0] = 0;
			PWMVal[1] = 0;
			MTLEN(TIM3,0);
			MTREN(TIM3,0);
			lVelocity = 0;
			rVelocity = 0;
			lTargetV = 0;
			rTargetV = 0;
			vTaskSuspend(Motor_Task_Handle);//������������Ϊ����û����
		}
		else if(commands[0] == 0x01){
			if(commands[1] == 0){
				printf("����ʧ�ܣ��ٶ������0~\r\n");
			}
			else{
				lTargetV = commands[1]/100 + (commands[1]%100)*0.01;
				rTargetV = lTargetV;
				printf("������Ŀ�공�٣�%.2lfm/s\r\n",lTargetV);
				vTaskResume(lCalcVelocity_Task_Handle);//��Ҳ������񣬳�������
				vTaskResume(rCalcVelocity_Task_Handle);//��Ҳ������񣬳�������
				vTaskResume(Motor_Task_Handle);//��ҵ������
			}
		}
		else if(commands[0] == 0x02){
			lTargetV = commands[1]/100 + (commands[1]%100)*0.01;
			rTargetV = commands[2]/100 + (commands[2]%100)*0.01;
			printf("���ٳɹ�~���ҳ���Ŀ�공��Ϊ��%.2lfm/s  %.2lfm/s\r\n",lTargetV,rTargetV);
			vTaskResume(lCalcVelocity_Task_Handle);//��Ҳ������񣬳�������
			vTaskResume(rCalcVelocity_Task_Handle);//��Ҳ������񣬳�������
			vTaskResume(Motor_Task_Handle);//��ҵ������
		}
	}
}

float filterNum = 0.01f;
/**
  * @brief    �����ٶ���������
  * @brief    ����ÿ����ٴ�����
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
		//�����������ӵ��ٶ�
		lVelocity = Filter((ONEPULSEDISTANCE * lCounter * 1000) / lTime, lVelocity, filterNum);
		lTime = 0;
	}
}

/**
  * @brief    �����ٶ���������
  * @brief    ����ÿ����ٴ�����
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
		//�����������ӵ��ٶ�
		rVelocity = Filter((ONEPULSEDISTANCE*rCounter * rCounter * 1000) / rTime, rVelocity, filterNum);
		rTime = 0;
	}
}

/**
  * @brief    �������ٴ�������������
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
					xPrintf("��������ź�����ֵʧ�ܣ�������Ϊ��%d\r\n",xReturn);
			}
			L = !L;
		}
		if(R != GPIO_ReadInputDataBit(VSENSORR_GPIO,VSENSORR_Pin)){
			if(R == 1){
				xReturn = xSemaphoreGive(vSensorRCountHandle);
				if(xReturn != pdTRUE)
					xPrintf("�Ҳ������ź�����ֵʧ�ܣ�������Ϊ��%d\r\n",xReturn);
			}
			R = !R;
		}
	}
}

/**
  * @brief    ���ת�ٿ�����������
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
		printf("�����ٶ�Ϊ:%lfm/s  �����ٶ�Ϊ:%lfm/s\r\n",lVelocity,rVelocity);
	}
}


/**
  * @brief    LED_Task��������
  * @param    parameter
  * @retval   void
  */
static void LED_Task(void* parameter){
	while(1){
		TestLED = 0;
		vTaskDelay(500/portTICK_PERIOD_MS);/*��ʱ500ms*/
		TestLED = 1;
		vTaskDelay(500/portTICK_PERIOD_MS);/*��ʱ500ms*/
	}
}


/**
  * @brief    �Ż�FreeRTOS��printf����
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

	/* �����ź��� */
	xSemaphoreTake(printfSemphr_Handle, portMAX_DELAY);

    printf("%s", buf_str);

  	xSemaphoreGive(printfSemphr_Handle);
}


/**
  * @brief    һ�׵�ͨ�˲��㷨
  * @param    newValue: �²���������
  * @param    oldValue: ��һ���˲����ֵ
  * @param    alpha   : �˲�ϵ��
  * @retval   �����˲����ֵ
  */
portFLOAT Filter(portFLOAT newValue, portFLOAT oldValue, portFLOAT alpha){
	newValue = alpha*newValue+(1-alpha)*oldValue;
	return newValue;
}


/**
  * @brief    ��ȡ��������������ջ��������ƿ��ڴ�
  * @param    ppxIdleTaskTCBBuffer		:	������ƿ��ڴ�
  * @param    ppxIdleTaskStackBuffer	:	�����ջ�ڴ�
  * @param    pulIdleTaskStackSize		:	�����ջ��С
  * @retval   void
  */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, 
								   StackType_t **ppxIdleTaskStackBuffer, 
								   uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer=&Idle_Task_TCB;/* ������ƿ��ڴ� */
	*ppxIdleTaskStackBuffer=Idle_Task_Stack;/* �����ջ�ڴ� */
	*pulIdleTaskStackSize=configMINIMAL_STACK_SIZE;/* �����ջ��С */
}

/**
  * @brief    ��ȡ��ʱ������������ջ��������ƿ��ڴ�
  * @param    ppxTimerTaskTCBBuffer	:		������ƿ��ڴ�
  * @param    ppxTimerTaskStackBuffer	:	�����ջ�ڴ�
  * @param    pulTimerTaskStackSize	:		�����ջ��С
  * @retval   void
  */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, 
									StackType_t **ppxTimerTaskStackBuffer, 
									uint32_t *pulTimerTaskStackSize)
{
	*ppxTimerTaskTCBBuffer=&Timer_Task_TCB;/* ������ƿ��ڴ� */
	*ppxTimerTaskStackBuffer=Timer_Task_Stack;/* �����ջ�ڴ� */
	*pulTimerTaskStackSize=configTIMER_TASK_STACK_DEPTH;/* �����ջ��С */
}

/**
  * @brief		����1�жϷ������
  * @param		void
  * @retval		void
  */
void USART1_IRQHandler(void){
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){//�����ж�
		if(USART1_RX_STA < USART_REC_LEN){
			usart1RXTime = 0;
			USART1_RX_BUF[USART1_RX_STA] = USART_ReceiveData(USART1);//��ȡ���յ�������
			USART1_RX_STA++;
		}
		else{
			USART1_RX_STA = 0;
		}
	}
}

/**
  * @brief		����2�жϷ������
  * @param		void
  * @retval		void
  */
void USART2_IRQHandler(void){
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){//�����ж�
		if(USART2_RX_STA < USART_REC_LEN){
			usart2RXTime = 0;
			USART2_RX_BUF[USART2_RX_STA] = USART_ReceiveData(USART2);//��ȡ���յ�������
			USART2_RX_STA++;
		}
		else{
			USART2_RX_STA = 0;
		}
	}
}

//��ʱ��2�жϷ�����
//ÿ���봥��һ���ж�
void TIM2_IRQHandler(void){
	BaseType_t xReturn = pdPASS;
	u8 i;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET){ //����ж�
		lTime++;
		rTime++;
		if(lTime == 500)
			lVelocity = 0;
		if(rTime == 500)
			rVelocity = 0;
		if(usart1RXTime == 10){
			//���ԣ��Ὣ�����յ����������ݷ���
			printf("Usart1 Recive over~\r\n");
			for(u8 i = 0; i < USART1_RX_STA; i++){
				printf("%lX ", USART1_RX_BUF[i]);
			}
			printf("\r\n");
			/*������д������Ϣ��������߼��ж�*/
//			if(USART1_RX_BUF[0] == 0x55)
//				Zigbee_Analyse_Command_Data();
			if(USART1_RX_BUF[0] == 0xC1 && USART1_RX_BUF[1] == 0xC2 && USART1_RX_BUF[2] == 0xC3 && USART1_RX_BUF[3] == 0xC4){
				for(i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++){
					xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART1_RX_BUF[4+i], NULL);
				}
				if(xReturn != pdTRUE)
					printf("����������Ϣ����ʧ��");
			}
			USART1_RX_STA = 0;
			usart1RXTime = 0xFF;//��ʱ����������ʾû���յ��µ���Ϣ
		}
		
		if(usart2RXTime == 10){
			//���ԣ��Ὣ�����յ����������ݷ���
			printf("Usart2 Recive over~\r\n");
			for(u8 i = 0; i < USART2_RX_STA; i++){
				printf("%lX ", USART2_RX_BUF[i]);
			}
			printf("\r\n");
			/*������д������Ϣ��������߼��ж�*/
//			if(USART1_RX_BUF[0] == 0x55)
//				Zigbee_Analyse_Command_Data();
			if(USART2_RX_BUF[0] == 0xC1 && USART2_RX_BUF[1] == 0xC2 && USART2_RX_BUF[2] == 0xC3 && USART2_RX_BUF[3] == 0xC4){
				for(i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++){
					xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART2_RX_BUF[4+i], NULL);
				}
				if(xReturn != pdTRUE)
					printf("����������Ϣ����ʧ��");
			}
			USART2_RX_STA = 0;
			usart2RXTime = 0xFF;//��ʱ����������ʾû���յ��µ���Ϣ
		}

		if(usart1RXTime < 10)
			usart1RXTime++;

		if(usart2RXTime < 10)
			usart2RXTime++;
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
	}
}
