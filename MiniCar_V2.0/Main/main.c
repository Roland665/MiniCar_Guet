#include "sys.h"
#include "usart/usart.h"
#include "freeRTOS.h"
#include "task.h"
#include "LED/led.h"
#include "HC_SR04/hc_sr04.h"
#include "CAP/myCap.h"
#include "Timer/myTimer.h"

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


void setup(void) //����0��ʼ��
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //����ϵͳʱ��Ϊ80MHz
	LED_Init();
	Usart0_Init(115200);
	HC_SR04_Init();
	HC_SR04_Start();
    Time0A_Init(80-1);//ϵͳƵ��Ϊ80Mhz��80000000/80=1us,ʵ��us���ж�
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

   vTaskDelete(AppCreate_Task_Handle); //ɾ����ʼ���� (2)
   taskEXIT_CRITICAL();            //�˳��ٽ���
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



u16 T2CCP0_STA = 0; //���벶��״̬ bit15��ʾ�Ƿ����һ�����岶��bit14��ʾ�Ƿ���������һ�α仯�أ�bit13~bit0��ʾ�������ʱ��(T2CCP0_STA++��䴥������)
//T2CCP0���жϷ�����
void TIMER2A_Handler(void){
	TimerIntClear(TIMER2_BASE, TIMER_CAPA_MATCH);
    if(T2CCP0_STA & 0x4000){
        //�����һ�������أ�������½�������
        T2CCP0_STA |= 0x8000;//������һ�θߵ�ƽ���岶��
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_POS_EDGE); //��������Ϊ�����ز��� 
        T2CCP0_STA &= ~0x4000;//����    
        printf("���һ�����岶��~��������Ϊ%d\r\n",T2CCP0_STA & 0x3FFF);
    }
    else{
        //��һ�β���������
        //��գ���ʼ��ʱ�ȴ��½���
        T2CCP0_STA = 0;
        T2CCP0_STA |= 0X4000;//��ǲ�����������
        TimerControlEvent(TIMER2_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE); //����Ϊ�½��ز��� 
    }
}

void TIMER0A_Handler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//HC-SR04
	if((T2CCP0_STA & 0X8000)==0 && (T2CCP0_STA & 0X4000)){
		//��δ��ɲ��񣬵����Ѿ����񵽸ߵ�ƽ��
		if((T2CCP0_STA&0X3FFF) == 0X3FFF){
			//�ߵ�ƽ̫����(����ʱ�����16383*1us)
			T2CCP0_STA |= 0xFFFF;
		}
		else T2CCP0_STA++;
	}
}