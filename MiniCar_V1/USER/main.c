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
#include "exti.h"
#include "hc_sr04.h"

/*******************************************/
/*
@version v1.1.2
�����v1.1.1�޸ģ�
- ����
	- ���������
	- �������
	- ����������

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
/********************************** �ں˶����� *********************************/
/*����ٴ����������ź������*/
static SemaphoreHandle_t vSensorLCountHandle = NULL;
/*�Ҳ��ٴ����������ź������*/
static SemaphoreHandle_t vSensorRCountHandle = NULL;
/*���ִ������ٶ���Ϣ���о��*/
static QueueHandle_t ReplaceVHandle = NULL;
/*���߿���������Ϣ���о��*/
static QueueHandle_t wirelessCommandHandle = NULL;
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
#define RunStackDeep 256
static StackType_t Run_Task_Stack[RunStackDeep];
/* OLEDShowing �����ջ */
#define OLEDShowingStackDeep 512
static StackType_t OLEDShowing_Task_Stack[OLEDShowingStackDeep];
/* MetalDetection �����ջ */
static StackType_t MetalDetection_Task_Stack[128];

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

/* �ź������ݽṹָ�� */
static StaticSemaphore_t vSensorLCount_Structure; /*����ٴ����������ź���*/
static StaticSemaphore_t vSensorRCount_Structure; /*�Ҳ��ٴ����������ź���*/

/* ��Ϣ�������ݽṹָ�� */
static StaticQueue_t ReplaceV_Structure;
static StaticQueue_t wirelessCommand_Structure;

/* ��Ϣ���еĴ洢���򣬴�С������ uxQueueLength * uxItemSize ���ֽ� */
uint8_t ReplaceVStorageArea[REPLACE_V_QUEUE_LENGTH * REPLACE_V_ITEM_SIZE];
uint8_t wirelessCommandStorageArea[WIRELESSCOMMAND_QUEUE_LENGTH * WIRELESSCOMMAND_ITEM_SIZE];

/* ��ͨȫ�ֱ��� */
int lVelocity; // �����ٶ�(m/s)
int rVelocity; // �����ٶ�(m/s)

u8 lPWMVal; // ���ֵ�ǰPWMռ�ձ�
u8 rPWMVal; // ���ֵ�ǰPWMռ�ձ�

s8 lTargetV; // Ŀ������PWMռ�ձ�
s8 rTargetV; // Ŀ������PWMռ�ձ�

u16 lTime = 0; // ����1�����ʱ
u16 rTime = 0; // ����1�����ʱ

u8 carMode = 0; // С������ģʽ 0-��ZigbeeЭ�������� 1-��ѭ���������

u8 exti1WaitTime; // used for exti1 debounce

u8 metalDiscoveryFlag = 0; // set 1 means this car found metal

u8 coinCounter; // coin number counter

u8 carState     = 0; // means the car`s move state, 0 is along the line, 1 is turn left slowly, 2 is turn left quickly, 3 is turn right slowly, 4 is turn right quickly
u8 trackingFlag = 0; // Six bit of one byte was used to means the state of car in road. The reflector is 1, the other is 0

u8 v1[2]         = {0x03, 0x64};// С����ת�����
u8 v2[2]         = {0x02, 0x64};// �з���ת�����
u8 v3[2]         = {0x01, 0x64};// �����ת�����
u8 v4[2]         = {0x00, 0x00};// Ѱ���쳣
u8 pwmerr        = 1;//pwm���ٷ���
u8 lV            = 0x15; // ������ʻʱ�����Ƽ�PWMռ�ձ�
u8 rV            = 0x15; // ������ʻʱ�����Ƽ�PWMռ�ձ�

u8 runRoad = 1;// this variable set 1 means car will along the out line, set 2 this car will along the in line
u16 CHANGEROADTIME_MAX = 0x3c*10;
u16 changeRoadTime = 0xffff;//This variable means the car begin change time to end change time  

u8 AckFlag = 0;//This flag will be set 1 when the car recive another device`s answer back

uint64_t runTime = 0;//Count the time from the car start run to stop run 
u8 runTimeEF = 0;//runTime enable flag , set 1 enable runTime to count

uint16_t speedUpTime = 2000;//2s to speed up
u8 speedUpTimeEF = 0;

u8 keyCD = 0;//the exti3 key 

u16 TIM1CH1_CAPTURE_STA = 0; //���벶��״̬ bit15��ʾ�Ƿ����һ�����岶��bit14��ʾ�Ƿ���������һ�α仯�أ�bit13~bit0��ʾ�������ʱ��(��λ��TIM1CH1_CAPTURE_STA++��䴥������)
float distance = 0;
u8 distanceWarningFlag = 0;// ������������־λ, ����ʱ��1

float followDistance = 25;//�����������(25cm)

u8 lightFlag = 0;
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

float Filter(float newValue, float oldValue, float alpha);
static void AppTaskCreate(void);                    /* ���ڴ������� */
static void LED_Task(void *parameter);              // LED����
static void ListeningSensors_Task(void *parameter); // �����������ٴ�����
static void lCalcVelocity_Task(void *parameter);    // ���������ٶ�����
static void rCalcVelocity_Task(void *parameter);    // ���������ٶ�����
static void AnalyseCommand_Task(void *parameter);   // ������������
static void Run_Task(void *parameter);              // ��Ѳ������
static void OLEDShowing_Task(void *parameter);      // OLED��ʾ����
static void MetalDetection_Task(void *parameter);   // task of detecting the metal
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
    USART2_Init(115200);
//    TIM2_Int_Init(10 - 1, 7200 - 1); // ��ʱ��ʱ��72M����Ƶϵ��7200������72M/7200=10Khz�ļ���Ƶ�ʣ�����10��Ϊ1ms
    vSensors_Init();
    dSensors_Init();
    Metal_Detection_Init();
    track_Init();
	HC_SR04_Init();
	TestLED = 0;
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

    vTaskDelete(AppTaskCreate_Handle); // ɾ��AppTaskCreate����
    printf("======����FreeRTOS!======\r\n");
    taskEXIT_CRITICAL(); // �˳��ٽ���
}

/**
 * @brief main body of metal-detection
 */
static void MetalDetection_Task(void *parameter)
{
    // while(1);
    while (1) {
        if (carMode == 1 && 1 == GPIO_ReadInputDataBit(METAL_DET_GPIO, METAL_DET_Pin)) {
            //��Ѳ��ģʽʱʶ��Ӳ��
            MTLEN(TIM3, 0);
            MTREN(TIM3, 0);
            coinCounter++;
            // ��������
            vTaskSuspend(Run_Task_Handle);
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
            BEEP = 0;
            metalDiscoveryFlag = 0;
            // �������
            vTaskResume(Run_Task_Handle);
            while (1 == GPIO_ReadInputDataBit(METAL_DET_GPIO, METAL_DET_Pin));
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
	int secondTime;
	int milliSecondTime;
    while (1) {
        u8g2_ClearBuffer(&u8g2); // clear the u8g2 buffer
        if (metalDiscoveryFlag == 0 && distanceWarningFlag == 0) {
            v = (lVelocity + rVelocity) / 2;
            u8g2_SetFont(&u8g2, u8g2_font_streamline_all_t); // ͼ���ȫ
            u8g2_DrawGlyph(&u8g2, 82, 20, 0x0047);           // Ӳ��������ʾͼ��
            u8g2_DrawGlyph(&u8g2, 0, 63, 0x029E);            // С��¿ͼ��
            u8g2_DrawGlyph(&u8g2, 0, 21, 0x0158);            // ���ͼ��
            u8g2_DrawGlyph(&u8g2, 0, 42, 0x0661);            // ���ͼ��

            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf); // 9���ص���ַ���
            u8g2_DrawStr(&u8g2, 21, 59, ":");        // С��¿��ð�ţ���ʾ�ٶ�
            u8g2_DrawStr(&u8g2, 103, 15, ":");       // Ӳ��ͼ���ð�ţ���ʾӲ������
            u8g2_DrawStr(&u8g2, 21, 38, ":");       // ���ͼ���ð�ţ���ʾ����
			
            sprintf(str, "0.%dm/s", v);
            u8g2_DrawStr(&u8g2, 32, 59, str); // С��ʵ���ٶ�
			
            sprintf(str, "%d", coinCounter);
            u8g2_DrawStr(&u8g2, 114, 15, str); // Ӳ��ʵ������
			
			secondTime = runTime/1000;
			milliSecondTime = (runTime % 1000) /10;
            sprintf(str, "%d:%d", secondTime, milliSecondTime);
            u8g2_DrawStr(&u8g2, 24, 17, str); // С���ܵ�ʱ��
			
            sprintf(str, "%.1f", distance);
            u8g2_DrawStr(&u8g2, 32, 38, str); // ����
        } else {
            u8g2_SetFont(&u8g2, u8g2_font_emoticons21_tr); // �����ȫ
            u8g2_DrawGlyphX2(&u8g2, 40, 50, 0x0030);       // ��⵽Ӳ����ʾ����
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
    s8 turningTrend = 0;//ת�����ƣ����ڸ���ģʽ��
                        //����ж�������������࣬��ֵΪ-1��
                        //�����ǰ������Ϊ0��
                        //������Ҳ࣬��Ϊ1��
	PID* car_pid_position = PID_Position_Create(3,0.001,500,100,50000);
    while (1) {
        // Base on the mode of the car to do thing
        // if mode is 1, means the car was in track finding mode
        if (carMode == 1){
			Go();
            // Detecting state of the car
            trackingFlag = 0;
            for (i = 0; i < 6; i++) {
                trackingFlag |= GPIO_ReadInputDataBit(GPIOB, GPIO_Pins[i]);
                trackingFlag <<= 1;
            }
            trackingFlag >>= 1;
			
			if (runRoad == 2 && changeRoadTime < CHANGEROADTIME_MAX){
				lTargetV = 0;
				rTargetV = 0;
			}
			else {
				runTimeEF = 1;
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
				} 
				else if (trackingFlag == 0x18 || trackingFlag == 0x10) { // 0b011000 || 0b010000
					// the car shifted to the right
					lTargetV = v2[0];
					rTargetV = v2[1];
				} 
				else if (trackingFlag == 0x30 || trackingFlag == 0x20) { // 0b110000 || 0b100000
					// the car shifted a lot to the right
					lTargetV = v3[0];
					rTargetV = v3[1];
				}
				// Turn right
				else if (trackingFlag == 0x04) { // 0b000100
					// the car shifted a bit to the left
					rTargetV = v1[0];
					lTargetV = v1[1];
				} 
				else if (trackingFlag == 0x02 || trackingFlag == 0x06) { // 0b000010 || 0b000110
					// the car shifted to the left
					rTargetV = v2[0];
					lTargetV = v2[1];
				} 
				else if (trackingFlag == 0x03 || trackingFlag == 0x01) { // 0b000011 || 0b000001
					// the car shifted a lot to the left
					rTargetV = v3[0];
					lTargetV = v3[1];
				} 
				else {
					// ���ڲ�����״̬��speed down
					lTargetV = v4[0];
					rTargetV = v4[1];
					runTimeEF = 0;
				}
            }
        }
		else if(carMode == 0){
            // if mode is 0, means the car was in manual control mode
            //���������
            if(distanceWarningFlag == 1){
				BEEP = 1;
                MTLEN(TIM3, 0);
                MTREN(TIM3, 0);
                Back();
                vTaskDelay(150);
                
                MTLEN(TIM3, lV+10);
                MTREN(TIM3, rV+10);
                vTaskDelay(700);
                MTLEN(TIM3, 0);
                MTREN(TIM3, 0);
                Go();
                vTaskDelay(150);
				
                MTLEN(TIM3, v3[0]);
                MTREN(TIM3, v3[1]-20);
                vTaskDelay(500);
                MTLEN(TIM3, 0);
                MTREN(TIM3, 0);
                vTaskDelay(150);
                
                MTLEN(TIM3, v3[1]-20);
                MTREN(TIM3, v3[0]);
                vTaskDelay(350);
                MTLEN(TIM3, 0);
                MTREN(TIM3, 0);
                vTaskDelay(150);
				BEEP = 0;
            }
				
        }
		else if(carMode == 2){
            // if mode is 2, means the car was in following mode
            if(distanceWarningFlag == 1){
				BEEP = 1;
                lTargetV = lV+10;
                rTargetV = rV+10;
                //if too close
                Back();
                
            }
            else if(distanceWarningFlag == 0){
				BEEP = 0;
                if(turningTrend == 0){
                    //����ж�������ǰ��
					if(distance - followDistance < 1 && distance - followDistance > -1)
						lTargetV = 0;
                    else
						lTargetV = PID_Position(car_pid_position,distance - followDistance);
					if(lTargetV < 0){
                        Back();
                        lTargetV = -lTargetV;
                    }
                    else
                        Go();
                    rTargetV = lTargetV;
					vTaskDelay(1);
                }
            }
        }
        
        // Adjust pwm duty ratio to speed control
		MTLEN(TIM3, lTargetV);
		MTREN(TIM3, rTargetV);
        //Output the real time velocity
//        printf("�����ٶ�Ϊ:0.%dm/s  �����ٶ�Ϊ:0.%dm/s  ���ִ��滻PWMValΪ:0.%dm/s  ���ִ��滻PWMValΪ:0.%dm/s\r\n", lVelocity, rVelocity, lTargetV, rTargetV);

    }
}

/**
 * @brief    ��������������������
 */
static void AnalyseCommand_Task(void *parameter)
{
    u8 commands[WIRELESSCOMMAND_QUEUE_LENGTH];
    u8 i;
    while (1) {
        for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
            xQueueReceive(wirelessCommandHandle, &commands[i], portMAX_DELAY);
        }

        if (commands[0] == 0x00) {
			lightFlag = 0;
            runTimeEF = 0;//ֹͣ��ʱ
            lTargetV = 0;
            rTargetV = 0;
            carMode  = 0; // �ֶ���
            // �п���ע���������п���ͣ��������ʱ
            MTLEN(TIM3, 0);
            MTREN(TIM3, 0);
            printf("ͣ��~\r\n");
        }
		else if (commands[0] == 0x01) {
			lightFlag = 0;
            Go();
			runTimeEF = 1;//��ʼ��ʱ
            carMode = 1;//�Զ���
            printf("��������Ѳ��ģʽ\r\n");
        }
		else if (commands[0] == 0x02) {
			lightFlag = 1;
			//��ת
            carMode = 0;//�ֶ���
            lTargetV = v2[0];
            rTargetV = v2[1];
        }
		else if (commands[0] == 0x03) {
			lightFlag = 1;
			//��ת
            carMode = 0;//�ֶ���
            lTargetV = v2[1];
            rTargetV = v2[0];
        } 
		else if (commands[0] == 0x04) {
			lightFlag = 1;
            //����
			carMode = 0;//�ֶ���
			Back();
            lTargetV = lV;
            rTargetV = rV;
        }
		else if (commands[0] == 0x05) {
            lV = commands[1];
            rV = commands[2];
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        }
		else if (commands[0] == 0x06) {
            v1[0] = commands[1];
            v1[1] = commands[2];
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        }
		else if (commands[0] == 0x07) {
            v2[0] = commands[1];
            v2[1] = commands[2];
            v3[0] = commands[1];
            v3[1] = commands[2];
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
        } 
		else if (commands[0] == 0x08) {
			lightFlag = 0;
            //��ͣ�²��л�Ϊֱ��ģʽ
			Go();
            carMode  = 0; // �ֶ���
            lTargetV = 0;
            rTargetV = 0;
            //����
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(1000);
            BEEP = 0;
            //������������ģʽ
            carMode = 2;
        }
		else if (commands[0] == 0x09){
			lightFlag = 0;
			Go();
            carMode  = 0; // �ֶ���
            lTargetV = commands[1];
            rTargetV = commands[2];
            BEEP  = 1;
            vTaskDelay(100);
            BEEP = 0;
            vTaskDelay(300);
            BEEP = 1;
            vTaskDelay(100);
            BEEP = 0;
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
    u8 vL = 0;
    u8 vR = 0;
    u8 dL = 0;
    u8 dR = 0;
	u16 ultrasonicTimes = 0;
	float temp;
    while (1) {
        if (vL != GPIO_ReadInputDataBit(VSENSORL_GPIO, VSENSORL_Pin)) {
            if (vL == 1) {
                xReturn = xSemaphoreGive(vSensorLCountHandle);
                if (xReturn != pdTRUE)
                    printf("��������ź�����ֵʧ�ܣ�������Ϊ��%d\r\n", xReturn);
            }
            vL = !vL;
        }
        if (vR != GPIO_ReadInputDataBit(VSENSORR_GPIO, VSENSORR_Pin)) {
            if (vR == 1) {
                xReturn = xSemaphoreGive(vSensorRCountHandle);
                if (xReturn != pdTRUE)
                    printf("�Ҳ������ź�����ֵʧ�ܣ�������Ϊ��%d\r\n", xReturn);
            }
            vR = !vR;
        }
		dL = GPIO_ReadInputDataBit(DSENSORL_GPIO, DSENSORL_Pin);
		dR = GPIO_ReadInputDataBit(DSENSORR_GPIO, DSENSORR_Pin);
		if(dL == 0 || dR == 0)
			distanceWarningFlag = 1;
		else
			distanceWarningFlag = 0;
		if(distanceWarningFlag == 0 && TIM1CH1_CAPTURE_STA&0x8000){
			//������������һ�����������岶��
			ultrasonicTimes = TIM1CH1_CAPTURE_STA&0x3FFF;
			TIM1CH1_CAPTURE_STA = 0;
			temp = Filter(ultrasonicTimes*0.17, distance, 0.8);
			if(temp < 300)
				distance = temp;
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
		if(lightFlag == 1){
			TestLED = 0;
			vTaskDelay(300 / portTICK_PERIOD_MS); /*��ʱ300ms*/
			TestLED = 1;
			vTaskDelay(300 / portTICK_PERIOD_MS); /*��ʱ300ms*/
		}
    }
}

/**
 * @brief    һ�׵�ͨ�˲��㷨
 * @param    newValue: �²���������
 * @param    oldValue: ��һ���˲����ֵ
 * @param    alpha   : �˲�ϵ��
 * @retval   �����˲����ֵ
 */
float Filter(float newValue, float oldValue, float alpha)
{
    newValue = alpha * newValue + (1 - alpha) * oldValue;
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
            usart1RXTime = 0;
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


void TIM1_CC_IRQHandler(void){
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET){ 
		//CC1 1 ���������¼�

		if(TIM1CH1_CAPTURE_STA & 0x4000) {
			//�����һ�������أ�������½�������
			TIM1CH1_CAPTURE_STA |= 0x8000;//�����ɲ���һ�θߵ�ƽ����
			TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //����Ϊ�����ز��� 
			TIM1CH1_CAPTURE_STA &= ~0x4000;
		}
		else{
			//��һ�β���������
			//��գ���ʼ��ʱ�ȴ��½���
			TIM1CH1_CAPTURE_STA = 0;
			TIM_SetCounter(TIM1, 0); 
			TIM1CH1_CAPTURE_STA |= 0X4000;//��ǲ�����������
			TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling); //����Ϊ�½��ز��� 
		}
	}
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //����жϱ�־λ	
}

//10us���жϣ�ʵ��ÿ�뷢10us�������HC-SR04
void TIM1_UP_IRQHandler(void){
	static u32 counter_10us = 0;
	static u8 counter_ms;
    BaseType_t xReturn = pdPASS;
    u8 i;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET) { // ����ж�
		//HC-SR04
		if((TIM1CH1_CAPTURE_STA & 0X8000)==0 && (TIM1CH1_CAPTURE_STA & 0X4000)){
			//��δ��ɲ��񣬵����Ѿ����񵽸ߵ�ƽ��
			if((TIM1CH1_CAPTURE_STA&0X3FFF) == 0X3FFF){
				//�ߵ�ƽ̫����(����ʱ�����16383*10us)
				TIM1CH1_CAPTURE_STA |= 0xFFFF;
			}
			else TIM1CH1_CAPTURE_STA++;		
		}

		if(counter_10us < 100000){
			counter_10us++;
		}
		else if(counter_10us == 100000){
			HC_SR04_TRIG = 1;
			counter_10us++;
		}
		else{
			HC_SR04_TRIG = 0;
			counter_10us = 0;
		}
		
		//ms��MCU��ʱ
		if(counter_ms == 100){
			//MCU������1ms
			if (keyCD < 20)
				keyCD++;
			
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


			if(changeRoadTime < CHANGEROADTIME_MAX)
				changeRoadTime++;
			
			if(runTimeEF == 1)
				runTime++;


			// else if(nodeCounter == 6)
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
					
					if(USART2_RX_BUF[4] == 0xFF)
						AckFlag = 1;
					else {
						for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
							xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART2_RX_BUF[4 + i], NULL);
						}
					}
					if (xReturn != pdTRUE)
						printf("����������Ϣ����ʧ��");
				}
				USART2_RX_STA = 0;
				usart2RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
			}
		}
		counter_ms++;

	}
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //����жϱ�־λ
}

// // ��ʱ��2�жϷ�����
// // ÿ���봥��һ���ж�
// void TIM2_IRQHandler(void)
// {
//     BaseType_t xReturn = pdPASS;
//     u8 i;
//     if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) { // ����ж�
// 		if (keyCD < 20)
// 			keyCD++;
		
//         if (lTime < 300)
//             lTime++;
//         else
//             lVelocity = 0;

//         if (rTime < 300)
//             rTime++;
//         else
//             rVelocity = 0;

//         if (usart1RXTime < 10)
//             usart1RXTime++;

//         if (usart2RXTime < 10)
//             usart2RXTime++;

// 		if(changeRoadTime < CHANGEROADTIME_MAX)
// 			changeRoadTime++;
        
//         if(runTimeEF == 1)
//             runTime++;


//         // else if(nodeCounter == 6)
//         if (usart1RXTime == 10) {
//             // ��������һ��������Ϣ
//             if (USART1_RX_BUF[0] == 0xC1 && USART1_RX_BUF[1] == 0xC2 && USART1_RX_BUF[2] == 0xC3 && USART1_RX_BUF[3] == 0xC4) {
//                 for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
//                     xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART1_RX_BUF[4 + i], NULL);
//                 }
//                 if (xReturn != pdTRUE)
//                     printf("����������Ϣ����ʧ��");
//             }
//             USART1_RX_STA = 0;
//             usart1RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
//         }
//         if (usart2RXTime == 10) {
//             // ��������һ��������Ϣ
//             if (USART2_RX_BUF[0] == 0xC1 && USART2_RX_BUF[1] == 0xC2 && USART2_RX_BUF[2] == 0xC3 && USART2_RX_BUF[3] == 0xC4) {
                
//                 if(USART2_RX_BUF[4] == 0xFF)
//                     AckFlag = 1;
//                 else {
//                     for (i = 0; i < WIRELESSCOMMAND_QUEUE_LENGTH; i++) {
//                         xReturn = xQueueSendFromISR(wirelessCommandHandle, &USART2_RX_BUF[4 + i], NULL);
//                     }
//                 }
//                 if (xReturn != pdTRUE)
//                     printf("����������Ϣ����ʧ��");
//             }
//             USART2_RX_STA = 0;
//             usart2RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
//         }
//         TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ����жϱ�־λ
//     }
// }

