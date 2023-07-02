#include "sys.h"
#include "usart/usart.h"
#include "freeRTOS.h"
#include "task.h"
#include "LED/led.h"
#include "HC_SR04/hc_sr04.h"
#include "CAP/myCap.h"
#include "Timer/myTimer.h"
#include "Filter/Filters.h"

/**************************** �������� ********************************/
//AppCreate ����ջ��
#define AppCreate_Task_Stack_Deep 128
//AppCreate �����ջ
StackType_t AppCreate_Task_Stack[AppCreate_Task_Stack_Deep];
//AppCreate ������
TaskHandle_t AppCreate_Task_Handle;
//AppCreate ������
void AppCreate_Task(void *pvParameters);

//LED ����ջ��
#define LED_Task_Stack_Deep 128
//LED �����ջ
StackType_t LED_Task_Stack[LED_Task_Stack_Deep];
//LED ������
TaskHandle_t LED_Task_Handle;
//LED ������
void LED_Task(void *pvParameters);

//LED2 ����ջ��
#define LED2_Task_Stack_Deep 128
//LED2 �����ջ
StackType_t LED2_Task_Stack[LED2_Task_Stack_Deep];
//LED2 ������
TaskHandle_t LED2_Task_Handle;
//LED2 ������
void LED2_Task(void *pvParameters);

//Ranging ����ջ��
#define Ranging_Task_Stack_Deep 128
//Ranging �����ջ
StackType_t Ranging_Task_Stack[Ranging_Task_Stack_Deep];
//Ranging ������
TaskHandle_t Ranging_Task_Handle;
//Ranging ������
void Ranging_Task(void *pvParameters);

/**************************** ȫ�ֱ��������� ********************************/
float distance = 0;//С����ǰ����������(��λcm)
u16 T2CCP0_STA = 0; //���벶��״̬ bit15��ʾ�Ƿ����һ�����岶��bit14��ʾ�Ƿ���������һ�α仯�أ�bit13~bit0��ʾ�������ʱ��(T2CCP0_STA++��䴥������)

void setup(void) //����0��ʼ��
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //����ϵͳʱ��Ϊ80MHz
	LED_Init();
	Usart0_Init(115200);
	HC_SR04_Init();
	HC_SR04_Start();
	Time0A_Init(800-1);//ϵͳƵ��Ϊ80Mhz��800/80000000=10us,ʵ��10us���ж�
}

int main(void)
{
	setup(); // ��ʼ��
    printf("\r\nProgram is run-begining\r\nϵͳʱ��Ƶ��Ϊ%u\r\n",SysCtlClockGet());
   /* ���� AppTaskCreate ���� */
	xTaskCreate((TaskFunction_t)	AppCreate_Task,       		// ������
				(const char *)		"AppCreate_Task",       	// ��������
				(uint32_t)			AppCreate_Task_Stack_Deep,  // �����ջ��С
				(void *)			NULL,                       // ���ݸ��������Ĳ���
				(UBaseType_t)		3,                      	// �������ȼ�
				(TaskHandle_t *)	AppCreate_Task_Handle);		// ������

   printf("=====׼������FreeRTOS!=====\r\n");
   vTaskStartScheduler(); /* ���������� */
    while (1);//������ִ�в�����仰
}

void AppCreate_Task(void *pvParameters)
{
   taskENTER_CRITICAL(); //�����ٽ���
   //���� LED_Task ����
   xTaskCreate((TaskFunction_t)	LED_Task,				// ������
               (const char *)	"LED_Task",				// ��������
               (uint16_t)		LED_Task_Stack_Deep,	// �����ջ��С
               (void *)			NULL,					// ���ݸ��������Ĳ���
               (UBaseType_t)	3,						// �������ȼ�
               (TaskHandle_t *)	&LED_Task_Handle);		// ������
			   
   //���� LED2_Task ����
   xTaskCreate((TaskFunction_t)	LED2_Task,				// ������
               (const char *)	"LED2_Task",				// ��������
               (uint16_t)		LED2_Task_Stack_Deep,	// �����ջ��С
               (void *)			NULL,					// ���ݸ��������Ĳ���
               (UBaseType_t)	3,						// �������ȼ�
               (TaskHandle_t *)	&LED2_Task_Handle);		// ������
               
   //���� Ranging_Task ����
   xTaskCreate((TaskFunction_t)	Ranging_Task,				// ������
               (const char *)	"Ranging_Task",				// ��������
               (uint16_t)		Ranging_Task_Stack_Deep,	// �����ջ��С
               (void *)			NULL,					// ���ݸ��������Ĳ���
               (UBaseType_t)	3,						// �������ȼ�
               (TaskHandle_t *)	&Ranging_Task_Handle);		// ������

   vTaskDelete(AppCreate_Task_Handle); //ɾ����ʼ���� (2)
   taskEXIT_CRITICAL();            //�˳��ٽ���
}

/**
  * @brief    Ranging_Task ������
  * @brief    ͨ������ T2CCP0 ��������峤�ȵõ�С����ǰ����������
  */
void Ranging_Task(void *pvParameters){
    u16 ultrasonicTimes = 0;
    while(1){
        if(T2CCP0_STA&0x8000){
            //���һ�����������岶��
			ultrasonicTimes = T2CCP0_STA&0x3FFF;
			T2CCP0_STA = 0;
			distance = Filter(ultrasonicTimes*0.17, distance, 0.8);
			printf("Time=%hu, distance=%.1fcm\r\n",ultrasonicTimes,distance);
        }
    }
}

/**
  * @brief    LED_Task ������
  * @brief    ��˸�ƣ����ֳ�����������
  */
void LED_Task(void *pvParameters){
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

/**
  * @brief    LED2_Task ������
  * @brief    ��˸�ƣ����ֳ�����������
  */

void LED2_Task(void *pvParameters){
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


//T2CCP0���жϷ�����
void TIMER2A_Handler(void){
    //����жϱ�־λ
	TimerIntClear( TIMER2_BASE,  TimerIntStatus( TIMER2_BASE,  true));
    if(T2CCP0_STA & 0x4000){
        //�����һ�������أ�������½�������
        T2CCP0_STA |= 0x8000;//������һ�θߵ�ƽ���岶��
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); //��������Ϊ�����ز��� 
        T2CCP0_STA &= ~0x4000;//����
    }
    else{
        //��һ�β���������
        //��գ���ʼ��ʱ�ȴ��½���
        T2CCP0_STA = 0;
        T2CCP0_STA |= 0x4000;//��ǲ�����������
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE); //����Ϊ�½��ز��� 
    }
}

//10us����ʱ����ʵ��ϵͳ��ʱ
void TIMER0A_Handler(void){
    static u16 usecond = 0;
    static u16 msecond = 0;
    static u8 TrigFlag = 0;//��TrigFlagΪ1ʱ��HC_SR04_TRIG����ߵ�ƽ����TrigFlag��0��
                           //��TrigFlagΪ0ʱ��HC_SR04_TRIG����͵�ƽ����TrigFlag��2��
                           //��TrigFlagΪ2ʱ���ȴ�TrigFlag���±���0
                           //���������ж�ÿ10usֻ���ж���Чһ��
    static u16 TrigFlagCD = 100;//��TrigFlagCD����100ʱ��TrigFlag��1(ʵ�ֵ���ÿ100ms��ȡ�����������Ϣ)
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //ms����ʱ
    if(usecond == 100){
        usecond = 0;
        //������1ms

        if(TrigFlagCD < 100)
            TrigFlagCD++;
        else{
            //����һ�γ��������ģ��
            TrigFlag = 1;
            TrigFlagCD = 0;
        }
        if(msecond == 1000){
            msecond = 0;
            //������1s

        }
        msecond++;
    }

	//HC-SR04
	if((T2CCP0_STA & 0X8000)==0 && (T2CCP0_STA & 0X4000)){
		//��δ��ɲ��񣬵����Ѿ����񵽸ߵ�ƽ��
		if((T2CCP0_STA&0X3FFF) >= 0x3FFF){
			//�ߵ�ƽ̫����(����ʱ�����700*10us)(������̫Զ�ˣ��Ͳ����ˡ�ע�⣬����С��5cmҲ�ᵼ�ºܳ��ĸߵ�ƽʱ�䣬�������ģ�����̲�������������5cn)
			T2CCP0_STA |= 0xFFFF;
		}
		else T2CCP0_STA++;
	}
    if(TrigFlag == 1){
        HC_SR04_TRIG_ENABLE;
		TrigFlag = 0;
	}
    else if(TrigFlag == 0){
        HC_SR04_TRIG_UNABLE;
        TrigFlag = 2;
	}

    usecond++;
}
