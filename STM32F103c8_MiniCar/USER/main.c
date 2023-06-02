#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sys.h"
#include "usart.h"
#include <stdarg.h> //��׼C���ļ�,�ú����ܹ����տɱ����
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
�����v1.0�޸ģ�
- �ϲ�������������ƫ���ϵ������
  1. �ж�ѭ����������Ŀ���ٶ�<--------+
  2. ���PWM��ʼ�仯-------------------+
- �Ż���ָ�����࣬ɾ����ԭָ��01
- ȡ���˸��������������������
- ȡ����PID���������ƶ���1.x�汾��С��ѭ��ģ�������
- ��ȫ�������ĿҪ������б�Ҫͨ������

ȱ��:
- �о����ٻ������Ż����ϲ��Ͳ����йص�����ȡ���ź�����ֱ��˳��ִ��
- ����ȶ��Եͣ��������Ե���
*/

/******************************* �궨�� ************************************/
/*�����Դ洢 2 �� u8���ͱ����Ķ��� */
#define REPLACE_V_QUEUE_LENGTH 2
#define REPLACE_V_ITEM_SIZE    sizeof(u8)

/*�����Դ洢 7 �� u8���ͱ����Ķ��� */
#define WIRELESSCOMMAND_QUEUE_LENGTH 7
#define WIRELESSCOMMAND_ITEM_SIZE    sizeof(u8)

/*�����Դ洢 2 �� u8���ͱ����Ķ��� */
#define CUSTOMDATA_QUEUE_LENGTH 2
#define CUSTOMDATA_ITEM_SIZE    sizeof(u8)
/**************************** ������ ********************************/
/* AppTaskCreate ������ */
static TaskHandle_t AppTaskCreate_Handle = NULL;
/* LED ������ */
static TaskHandle_t LED_Task_Handle = NULL;
/* ListeningSensors ������*/
static TaskHandle_t ListeningSensors_Task_Handle = NULL;
/* lCalcVelocity ������*/
static TaskHandle_t lCalcVelocity_Task_Handle = NULL;
/* rCalcVelocity ������*/
static TaskHandle_t rCalcVelocity_Task_Handle = NULL;
/* AnalyseCommand ������*/
static TaskHandle_t AnalyseCommand_Task_Handle = NULL;
/* Run ������*/
static TaskHandle_t Run_Task_Handle = NULL;
/* OLEDShowing ������*/
static TaskHandle_t OLEDShowing_Task_Handle = NULL;
/* MetalDetection ������*/
static TaskHandle_t MetalDetection_Task_Handle = NULL;
/* SendCustomData ������*/
static TaskHandle_t SendCustomData_Task_Handle = NULL;
/********************************** �ں˶����� *********************************/
/*����ٴ����������ź������*/
static SemaphoreHandle_t vSensorLCountHandle = NULL;
/*�Ҳ��ٴ����������ź������*/
static SemaphoreHandle_t vSensorRCountHandle = NULL;
/*���ִ������ٶ���Ϣ���о��*/
static QueueHandle_t ReplaceVHandle = NULL;
/*���߿���������Ϣ���о��*/
static QueueHandle_t wirelessCommandHandle = NULL;
/*������˽��Э����Ϣ���о��*/
static QueueHandle_t customDataHandle = NULL;
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
/* ListeningSensors �����ջ */
static StackType_t ListeningSensors_Task_Stack[128];
/* lCalcVelocity �����ջ */
static StackType_t lCalcVelocity_Task_Stack[128];
/* rCalcVelocity �����ջ */
static StackType_t rCalcVelocity_Task_Stack[128];
/* AnalyseCommand �����ջ */
static StackType_t AnalyseCommand_Task_Stack[128];
/* Run �����ջ */
#define RunStackDeep 512
static StackType_t Run_Task_Stack[RunStackDeep];
/* OLEDShowing �����ջ */
#define OLEDShowingStackDeep 256
static StackType_t OLEDShowing_Task_Stack[OLEDShowingStackDeep];
/* MetalDetection �����ջ */
static StackType_t MetalDetection_Task_Stack[128];
/* SendCustomData �����ջ */
#define SendCustomDataStackDeep 128
static StackType_t SendCustomData_Task_Stack[SendCustomDataStackDeep];

/* AppTaskCreate ������ƿ� */
static StaticTask_t AppTaskCreate_TCB;
/* LED ������ƿ� */
static StaticTask_t LED_Task_TCB;
/* ListeningSensors ������ƿ� */
static StaticTask_t ListeningSensors_Task_TCB;
/* lCalcVelocity ������ƿ� */
static StaticTask_t lCalcVelocity_Task_TCB;
/* rCalcVelocity ������ƿ� */
static StaticTask_t rCalcVelocity_Task_TCB;
/* AnalyseCommand ������ƿ� */
static StaticTask_t AnalyseCommand_Task_TCB;
/* Run ������ƿ� */
static StaticTask_t Run_Task_TCB;
/* OLEDShowing ������ƿ� */
static StaticTask_t OLEDShowing_Task_TCB;
/* MetalDetection ������ƿ� */
static StaticTask_t MetalDetection_Task_TCB;
/* SendCustomData ������ƿ� */
static StaticTask_t SendCustomData_Task_TCB;

/* �ź������ݽṹָ�� */
static StaticSemaphore_t vSensorLCount_Structure; /*����ٴ����������ź���*/
static StaticSemaphore_t vSensorRCount_Structure; /*�Ҳ��ٴ����������ź���*/

/* ��Ϣ�������ݽṹָ�� */
static StaticQueue_t ReplaceV_Structure;
static StaticQueue_t wirelessCommand_Structure;
static StaticQueue_t customData_Structure;

/* ��Ϣ���еĴ洢���򣬴�С������ uxQueueLength * uxItemSize ���ֽ� */
uint8_t ReplaceVStorageArea[REPLACE_V_QUEUE_LENGTH * REPLACE_V_ITEM_SIZE];
uint8_t wirelessCommandStorageArea[WIRELESSCOMMAND_QUEUE_LENGTH * WIRELESSCOMMAND_ITEM_SIZE];
uint8_t customDataStorageArea[CUSTOMDATA_QUEUE_LENGTH * CUSTOMDATA_ITEM_SIZE];

/* ��ͨȫ�ֱ��� */
int lVelocity; // �����ٶ�(m/s)
int rVelocity; // �����ٶ�(m/s)

int lReplaceV; // Ŀ�������ٶ�
int rReplaceV; // Ŀ�������ٶ�

int lTargetV; // ���滻�������ٶ�
int rTargetV; // ���滻�������ٶ�

u16 lTime = 0; // ����1�����ʱ
u16 rTime = 0; // ����1�����ʱ

u16 PWMVal[2] = {0};

u8 carMode = 0; // С������ģʽ 0-��ZigbeeЭ�������� 1-��ѭ���������

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
int lV            = 13; // ������ʻʱ�����Ƽ��ٶ�
int rV            = 40; // ������ʻʱ�����Ƽ��ٶ�

u8 runRoad = 1;// this variable set 1 means car will along the out line, set 2 this car will along the in line
u16 CHANGEROADTIME_MAX = 0x4a*10;
u16 changeRoadTime = 0xffff;//This variable means the car begin change time to end change time  

u8 AckFlag = 0;//This flag will be set 1 when the car recive another device`s answer back
/*
*************************************************************************
*                             ��������
*************************************************************************
*/
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize);

int Filter(float newValue, float oldValue, float alpha);
static void AppTaskCreate(void);                    /* ���ڴ������� */
static void LED_Task(void *parameter);              // LED����
static void ListeningSensors_Task(void *parameter); // �����������ٴ�����
static void lCalcVelocity_Task(void *parameter);    // ���������ٶ�����
static void rCalcVelocity_Task(void *parameter);    // ���������ٶ�����
static void AnalyseCommand_Task(void *parameter);   // ������������
static void Run_Task(void *parameter);              // ��Ѳ������
static void OLEDShowing_Task(void *parameter);      // OLED��ʾ����
static void MetalDetection_Task(void *parameter);   // task of detecting the metal
static void SendCustomData_Task(void *parameter);   // task of send custom data
static void Setup(void);                            /* ���ڳ�ʼ�����������Դ */

int main(void)
{
    Setup(); // ��ʼ��
    /* ���� AppTaskCreate ���� */
    AppTaskCreate_Handle = xTaskCreateStatic((TaskFunction_t)AppTaskCreate,       // ������
                                             (const char *)"AppTaskCreate",       // ��������
                                             (uint32_t)128,                       // �����ջ��С
                                             (void *)NULL,                        // ���ݸ��������Ĳ���
                                             (UBaseType_t)3,                      // �������ȼ�
                                             (StackType_t *)AppTaskCreate_Stack,  // �����ջ
                                             (StaticTask_t *)&AppTaskCreate_TCB); // ������ƿ�

    if (AppTaskCreate_Handle != NULL) {
        // ���񴴽��ɹ�
        printf("=====׼������FreeRTOS!=====\r\n");
        vTaskStartScheduler(); /* ���������� */
    }
    while (1);
}

/**
 * @brief    ���������ʼ��
 * @param    void
 * @retval   void
 */
static void Setup(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 4bit������������ռ���ȼ�
    MTS_Init();
    LED_Init();
    BEEP_Init();
    USART1_Init(115200);
    LED_Init();
    BEEP_Init();
    USART1_Init(115200);
    USART2_Init(115200);
    TIM2_Int_Init(10 - 1, 7200 - 1); // ��ʱ��ʱ��72M����Ƶϵ��7200������72M/7200=10Khz�ļ���Ƶ�ʣ�����10��Ϊ1ms
    vSensors_Init();
    Metal_Detection_Init();
    track_Init();
}

static void AppTaskCreate(void)
{
    taskENTER_CRITICAL(); // �����ٽ���

    // ��¼���ֹ����ٴ�������ȡ������
    vSensorLCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,           // ������ֵ
                                                         0,                         // ��ʼ����ֵ
                                                         &vSensorLCount_Structure); // �ź��������ݽṹ��
    if (vSensorLCountHandle != NULL)                                                /* �����ɹ� */
        printf("vSensorLCount �����ź��������ɹ�!\r\n");
    else
        printf("vSensorLCount �����ź�������ʧ��!\r\n");

    // ��¼�ҳ��ֹ����ٴ�������ȡ������
    vSensorRCountHandle = xSemaphoreCreateCountingStatic(vSensorCountMax,           // ������ֵ
                                                         0,                         // ��ʼ����ֵ
                                                         &vSensorRCount_Structure); // �ź��������ݽṹ��
    if (vSensorRCountHandle != NULL)                                                /* �����ɹ� */
        printf("vSensorRCount �����ź��������ɹ�!\r\n");
    else
        printf("vSensorRCount �����ź�������ʧ��!\r\n");

    // �洢���ִ���������ֵ
    ReplaceVHandle = xQueueCreateStatic(REPLACE_V_QUEUE_LENGTH, // �������
                                        REPLACE_V_ITEM_SIZE,    // �������ݵ�Ԫ�ĵ�λ
                                        ReplaceVStorageArea,    // ���еĴ洢����
                                        &ReplaceV_Structure     // ���е����ݽṹ
    );
    if (ReplaceVHandle != NULL) /* �����ɹ� */
        printf("ReplaceV���д����ɹ�!\r\n");
    else
        printf("ReplaceV���д���ʧ��!\r\n");

    // �洢��������
    wirelessCommandHandle = xQueueCreateStatic(WIRELESSCOMMAND_QUEUE_LENGTH, // �������
                                               WIRELESSCOMMAND_ITEM_SIZE,    // �������ݵ�Ԫ�ĵ�λ
                                               wirelessCommandStorageArea,   // ���еĴ洢����
                                               &wirelessCommand_Structure    // ���е����ݽṹ
    );
    if (wirelessCommandHandle != NULL) /* �����ɹ� */
        printf("wirelessCommand���д����ɹ�!\r\n");
    else
        printf("wirelessCommand���д���ʧ��!\r\n");

    // �洢������˽��Э������
    customDataHandle = xQueueCreateStatic(CUSTOMDATA_QUEUE_LENGTH, // �������
                                               CUSTOMDATA_ITEM_SIZE,    // �������ݵ�Ԫ�ĵ�λ
                                               customDataStorageArea,   // ���еĴ洢����
                                               &customData_Structure    // ���е����ݽṹ
    );
    if (customDataHandle != NULL) /* �����ɹ� */
        printf("customData���д����ɹ�!\r\n");
    else
        printf("customData���д���ʧ��!\r\n");

    /* ����LED_Task���� */
    LED_Task_Handle = xTaskCreateStatic((TaskFunction_t)LED_Task,       // ������
                                        (const char *)"LED_Task",       // ��������
                                        (uint32_t)128,                  // ����ջ��
                                        (void *)NULL,                   // ���ݸ��������Ĳ���
                                        (UBaseType_t)4,                 // �������ȼ�
                                        (StackType_t *)LED_Task_Stack,  // �����ջ
                                        (StaticTask_t *)&LED_Task_TCB); // ������ƿ�
    if (LED_Task_Handle != NULL)                                        /* �����ɹ� */
        printf("LED_Task���񴴽��ɹ�!\r\n");
    else
        printf("LED_Task���񴴽�ʧ��!\r\n");

    /* ���� ListeningSensors_Task ���� */
    ListeningSensors_Task_Handle = xTaskCreateStatic((TaskFunction_t)ListeningSensors_Task,       // ������
                                                     (const char *)"ListeningSensors_Task",       // ��������
                                                     (uint32_t)128,                               // ����ջ��
                                                     (void *)NULL,                                // ���ݸ��������Ĳ���
                                                     (UBaseType_t)4,                              // �������ȼ�
                                                     (StackType_t *)ListeningSensors_Task_Stack,  // �����ջ
                                                     (StaticTask_t *)&ListeningSensors_Task_TCB); // ������ƿ�
    if (ListeningSensors_Task_Handle != NULL)                                                     /* �����ɹ� */
        printf("ListeningSensors_Task���񴴽��ɹ�!\r\n");
    else
        printf("ListeningSensors_Task���񴴽�ʧ��!\r\n");

    /* ���� lCalcVelocity_Task ���� */
    lCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t)lCalcVelocity_Task,       // ������
                                                  (const char *)"lCalcVelocity_Task",       // ��������
                                                  (uint32_t)128,                            // ����ջ��
                                                  (void *)NULL,                             // ���ݸ��������Ĳ���
                                                  (UBaseType_t)4,                           // �������ȼ�
                                                  (StackType_t *)lCalcVelocity_Task_Stack,  // �����ջ
                                                  (StaticTask_t *)&lCalcVelocity_Task_TCB); // ������ƿ�
    if (lCalcVelocity_Task_Handle != NULL)                                                  /* �����ɹ� */
        printf("lCalcVelocity_Task���񴴽��ɹ�!\r\n");
    else
        printf("lCalcVelocity_Task���񴴽�ʧ��!\r\n");

    /* ���� rCalcVelocity_Task ���� */
    rCalcVelocity_Task_Handle = xTaskCreateStatic((TaskFunction_t)rCalcVelocity_Task,       // ������
                                                  (const char *)"rCalcVelocity_Task",       // ��������
                                                  (uint32_t)128,                            // ����ջ��
                                                  (void *)NULL,                             // ���ݸ��������Ĳ���
                                                  (UBaseType_t)4,                           // �������ȼ�
                                                  (StackType_t *)rCalcVelocity_Task_Stack,  // �����ջ
                                                  (StaticTask_t *)&rCalcVelocity_Task_TCB); // ������ƿ�
    if (rCalcVelocity_Task_Handle != NULL)                                                  /* �����ɹ� */
        printf("rCalcVelocity_Task���񴴽��ɹ�!\r\n");
    else
        printf("rCalcVelocity_Task���񴴽�ʧ��!\r\n");

    /* ���� AnalyseCommand_Task ���� */
    AnalyseCommand_Task_Handle = xTaskCreateStatic((TaskFunction_t)AnalyseCommand_Task,       // ������
                                                   (const char *)"AnalyseCommand_Task",       // ��������
                                                   (uint32_t)128,                             // ����ջ��
                                                   (void *)NULL,                              // ���ݸ��������Ĳ���
                                                   (UBaseType_t)4,                            // �������ȼ�
                                                   (StackType_t *)AnalyseCommand_Task_Stack,  // �����ջ
                                                   (StaticTask_t *)&AnalyseCommand_Task_TCB); // ������ƿ�
    if (AnalyseCommand_Task_Handle != NULL)                                                   /* �����ɹ� */
        printf("AnalyseCommand���񴴽��ɹ�!\r\n");
    else
        printf("AnalyseCommand���񴴽�ʧ��!\r\n");

    /* ���� Run_Task ���� */
    Run_Task_Handle = xTaskCreateStatic((TaskFunction_t)Run_Task,       // ������
                                        (const char *)"Run_Task",       // ��������
                                        (uint32_t)RunStackDeep,         // ����ջ��
                                        (void *)NULL,                   // ���ݸ��������Ĳ���
                                        (UBaseType_t)4,                 // �������ȼ�
                                        (StackType_t *)Run_Task_Stack,  // �����ջ
                                        (StaticTask_t *)&Run_Task_TCB); // ������ƿ�
    if (Run_Task_Handle != NULL)                                        /* �����ɹ� */
        printf("Run���񴴽��ɹ�!\r\n");
    else
        printf("Run���񴴽�ʧ��!\r\n");

    /* ���� OLEDShowing_Task ���� */
    OLEDShowing_Task_Handle = xTaskCreateStatic((TaskFunction_t)OLEDShowing_Task,       // ������
                                                (const char *)"OLEDShowing_Task",       // ��������
                                                (uint32_t)OLEDShowingStackDeep,         // ����ջ��
                                                (void *)NULL,                           // ���ݸ��������Ĳ���
                                                (UBaseType_t)4,                         // �������ȼ�
                                                (StackType_t *)OLEDShowing_Task_Stack,  // �����ջ
                                                (StaticTask_t *)&OLEDShowing_Task_TCB); // ������ƿ�
    if (OLEDShowing_Task_Handle != NULL)                                                /* �����ɹ� */
        printf("OLEDShowing���񴴽��ɹ�!\r\n");
    else
        printf("OLEDShowing���񴴽�ʧ��!\r\n");

    /* ���� MetalDetection_Task ���� */
    MetalDetection_Task_Handle = xTaskCreateStatic((TaskFunction_t)MetalDetection_Task,       // ������
                                                   (const char *)"MetalDetection_Task",       // ��������
                                                   (uint32_t)128,                             // ����ջ��
                                                   (void *)NULL,                              // ���ݸ��������Ĳ���
                                                   (UBaseType_t)4,                            // �������ȼ�
                                                   (StackType_t *)MetalDetection_Task_Stack,  // �����ջ
                                                   (StaticTask_t *)&MetalDetection_Task_TCB); // ������ƿ�
    if (MetalDetection_Task_Handle != NULL)                                                   /* �����ɹ� */
        printf("MetalDetection���񴴽��ɹ�!\r\n");
    else
        printf("MetalDetection���񴴽�ʧ��!\r\n");

    /* ���� SendCustomData_Task ���� */
    SendCustomData_Task_Handle = xTaskCreateStatic((TaskFunction_t)SendCustomData_Task,       // ������
                                                   (const char *)"SendCustomData_Task",       // ��������
                                                   (uint32_t)SendCustomDataStackDeep,                             // ����ջ��
                                                   (void *)NULL,                              // ���ݸ��������Ĳ���
                                                   (UBaseType_t)4,                            // �������ȼ�
                                                   (StackType_t *)SendCustomData_Task_Stack,  // �����ջ
                                                   (StaticTask_t *)&SendCustomData_Task_TCB); // ������ƿ�
    if (SendCustomData_Task_Handle != NULL)                                                   /* �����ɹ� */
        printf("SendCustomData���񴴽��ɹ�!\r\n");
    else
        printf("SendCustomData���񴴽�ʧ��!\r\n");

    vTaskDelete(AppTaskCreate_Handle); // ɾ��AppTaskCreate����
    printf("======����FreeRTOS!======\r\n");
    taskEXIT_CRITICAL(); // �˳��ٽ���
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
            printf("������Ϣ����ʧ��\r\n");
        AckFlag = 0;
        breakCounter = 0;
        while(AckFlag != 1 && breakCounter < 6){
            //ѭ���������ݣ�ֱ���յ�Ӧ��1.8����û�յ��Ͳ�����
			USART_SendData(USART2, 0xC1);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			USART_SendData(USART2, 0xC2);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			USART_SendData(USART2, 0xC3);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			USART_SendData(USART2, 0xC4);         //�򴮿�1��������
			while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
            for(i = 0; i < CUSTOMDATA_QUEUE_LENGTH; i++){
                USART_SendData(USART2, Data[i]);         //�򴮿�1��������
                while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
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
            // ��������
            vTaskSuspend(Run_Task_Handle);
            // ͬ��Ӳ������
            beSendData[1] = coinCounter;
			for(i = 0; i < CUSTOMDATA_QUEUE_LENGTH; i++){				
				xReturn = xQueueSend(customDataHandle, &beSendData[i], 1000/portTICK_PERIOD_MS);
				if(xReturn != pdPASS)
					printf("Ӳ��������Ϣ����ʧ��\r\n");
			}
            metalDiscoveryFlag = 1;
            // �����������
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
            // �������
            vTaskResume(Run_Task_Handle);
            while (1 == GPIO_ReadInputDataBit(METAL_DET_GPIO, METAL_DET_Pin))
                ;
            vTaskDelay(300);
            pwmerr = 3;
        }
    }
}

/**

  * @brief OLED��ʾ��������
  * @brief
  * @param
  * @retval
  */
static void OLEDShowing_Task(void *parameter)
{
    char str[128] = {0}; // ��Ҫ��ʾ���ַ�����ʱ��Ŵ�
    // ��ʼ��u8g2
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
            u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t); // ͼ���ȫ
            u8g2_DrawGlyph(&u8g2, 82, 20, 0x0047);           // Ӳ��������ʾͼ��
            u8g2_DrawGlyph(&u8g2, 0, 63, 0x029E);            // С��¿ͼ��

            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf); // 9���ص���ַ���
            u8g2_DrawStr(&u8g2, 21, 59, ":");        // С��¿��ð�ţ���ʾ�ٶ�
            u8g2_DrawStr(&u8g2, 103, 15, ":");       // Ӳ��ͼ���ð�ţ���ʾӲ������
            sprintf(str, "0.%dm/s", v);
            u8g2_DrawStr(&u8g2, 32, 59, str); // С��ʵ���ٶ�
            sprintf(str, "%d", coinCounter);
            u8g2_DrawStr(&u8g2, 114, 15, str); // Ӳ��ʵ������
        } else {
            u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t); // ͼ���ȫ
            u8g2_DrawGlyph(&u8g2, 82, 20, 0x0047);           // Ӳ��������ʾͼ��

            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf); // 9���ص���ַ���
            u8g2_DrawStr(&u8g2, 103, 15, ":");       // Ӳ��ͼ���ð�ţ���ʾӲ������
            sprintf(str, "%d", coinCounter);
            u8g2_DrawStr(&u8g2, 114, 15, str); // Ӳ��ʵ������

            u8g2_SetFont(&u8g2, u8g2_font_emoticons21_tr); // �����ȫ
            u8g2_DrawGlyphX2(&u8g2, 40, 50, 0x0030);       // ��⵽Ӳ����ʾ����
        }
        if (nodeCounter == 3) {
            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf); // 9���ص���ַ���
            u8g2_DrawStr(&u8g2, 100, 59, "speed up!");
        }
        u8g2_SendBuffer(&u8g2); // ͬ����Ļ
    }
}

/**
  * @brief Run ��������
  */

static void Run_Task(void *parameter)
{
    u16 GPIO_Pins[6] = {GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15}; // ����ģ������
    u8 i;

    int ignoreErr = 0; // �����ٶ��ϵ����ֵ
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
					// ���ڲ�����״̬��speed down
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
			printf("�����ٶ�Ϊ:0.%dm/s  �����ٶ�Ϊ:0.%dm/s  ���ִ��滻�ٶ�Ϊ:0.%dm/s  ���ִ��滻�ٶ�Ϊ:0.%dm/s\r\n", lVelocity, rVelocity, lReplaceV, rReplaceV);
        } else {
            PWMVal[1] = 0;
            MTREN(TIM3, PWMVal[1]);
        }
    }
}

/**
 * @brief    ��������������������
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
            printf("������Ϣ����ʧ��\r\n");
        printf("�ɹ����տ������%d\r\n", commands[0]);

        if (commands[0] == 0x00) {
            carMode  = 0; // �ֶ���
            lTargetV = 0;
            rTargetV = 0;
            // �п���ע���������п���ͣ��������ʱ
            MTLEN(TIM3, 0);
            MTREN(TIM3, 0);
            printf("ͣ��~\r\n");
        } else if (commands[0] == 0x01) {
            carMode  = 0; // �ֶ���
            lTargetV = commands[1];
            rTargetV = commands[2];
            printf("���ٳɹ�~���ҳ���Ŀ�공��Ϊ��0.%dm/s  0.%dm/s\r\n", lTargetV, rTargetV);
        } else if (commands[0] == 0x02) {
            carMode = 1;
            printf("��������Ѳ��ģʽ\r\n");
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
            AckFlag = 1;//�յ�Ӧ��
        }
    }
}

float filterNum = 0.5f; // ��ͨ�˲�ϵ��
/**
 * @brief    �����ٶ���������
 * @brief    ����ÿ����ٴ�����
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
        // �������ӵ��ٶ�
        lVelocity = Filter((ONEPULSEDISTANCE * lCounter) * 100 / lTime, lVelocity, filterNum);
        lTime     = 0;
    }
}

/**
 * @brief    �����ٶ���������
 * @brief    ����ÿ����ٴ�����
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
        // �������ӵ��ٶ�
        rVelocity = Filter((ONEPULSEDISTANCE * rCounter) * 100 / rTime, rVelocity, filterNum);
        rTime     = 0;
    }
}

/**
 * @brief    �������ٴ�������������
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
                    printf("��������ź�����ֵʧ�ܣ�������Ϊ��%d\r\n", xReturn);
            }
            L = !L;
        }
        if (R != GPIO_ReadInputDataBit(VSENSORR_GPIO, VSENSORR_Pin)) {
            if (R == 1) {
                xReturn = xSemaphoreGive(vSensorRCountHandle);
                if (xReturn != pdTRUE)
                    printf("�Ҳ������ź�����ֵʧ�ܣ�������Ϊ��%d\r\n", xReturn);
            }
            R = !R;
        }
    }
}

/**
 * @brief    LED_Task��������
 * @param    parameter
 * @retval   void
 */
static void LED_Task(void *parameter)
{
    while (1) {
        TestLED = 0;
        vTaskDelay(500 / portTICK_PERIOD_MS); /*��ʱ500ms*/
        TestLED = 1;
        vTaskDelay(500 / portTICK_PERIOD_MS); /*��ʱ500ms*/
    }
}

/**
 * @brief    һ�׵�ͨ�˲��㷨
 * @param    newValue: �²���������
 * @param    oldValue: ��һ���˲����ֵ
 * @param    alpha   : �˲�ϵ��
 * @retval   �����˲����ֵ
 */
int Filter(float newValue, float oldValue, float alpha)
{
    newValue = (int)(alpha * newValue + (1 - alpha) * oldValue);
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
    *ppxIdleTaskTCBBuffer   = &Idle_Task_TCB;           /* ������ƿ��ڴ� */
    *ppxIdleTaskStackBuffer = Idle_Task_Stack;          /* �����ջ�ڴ� */
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE; /* �����ջ��С */
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
    *ppxTimerTaskTCBBuffer   = &Timer_Task_TCB;              /* ������ƿ��ڴ� */
    *ppxTimerTaskStackBuffer = Timer_Task_Stack;             /* �����ջ�ڴ� */
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH; /* �����ջ��С */
}

/**
 * @brief		����1�жϷ������
 * @param		void
 * @retval		void
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) { // �����ж�
        if (USART1_RX_STA < USART_REC_LEN) {
            usart1RXTime                 = 0;
            USART1_RX_BUF[USART1_RX_STA] = USART_ReceiveData(USART1); // ��ȡ���յ�������
            USART1_RX_STA++;
        } else {
            USART1_RX_STA = 0;
        }
    }
}

/**
 * @brief		����2�жϷ������
 * @param		void
 * @retval		void
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) { // �����ж�
        if (USART2_RX_STA < USART_REC_LEN) {
            usart2RXTime                 = 0;
            USART2_RX_BUF[USART2_RX_STA] = USART_ReceiveData(USART2); // ��ȡ���յ�������
            USART2_RX_STA++;
        } else {
            USART2_RX_STA = 0;
        }
    }
}

// ��ʱ��2�жϷ�����
// ÿ���봥��һ���ж�
void TIM2_IRQHandler(void)
{
    BaseType_t xReturn = pdPASS;
    u8 i;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) { // ����ж�
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
            // ��������һ��������Ϣ
            if (USART1_RX_BUF[0] == 0xC1 && USART1_RX_BUF[1] == 0xC2 && USART1_RX_BUF[2] == 0xC3 && USART1_RX_BUF[3] == 0xC4) {
                for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
                    xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART1_RX_BUF[4 + i], NULL);
                }
                if (xReturn != pdTRUE)
                    printf("����������Ϣ����ʧ��");
            }
            USART1_RX_STA = 0;
            usart1RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
        }
        if (usart2RXTime == 10) {
            // ��������һ��������Ϣ
            if (USART2_RX_BUF[0] == 0xC1 && USART2_RX_BUF[1] == 0xC2 && USART2_RX_BUF[2] == 0xC3 && USART2_RX_BUF[3] == 0xC4) {
                for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
                    xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART2_RX_BUF[4 + i], NULL);
                }
                if (xReturn != pdTRUE)
                    printf("����������Ϣ����ʧ��");
            }
            USART2_RX_STA = 0;
            usart2RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
        }
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ����жϱ�־λ
    }
}
