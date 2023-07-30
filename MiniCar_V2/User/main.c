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
�����V2.0ϵ��
  - �������������������ⲿӲ�������踴�ã�Ϊ�˷���PCB����

ʵ����
  - 
*/

/**************************** �ں˶������� ********************************/


/**************************** �������� ********************************/
/* AppCreate ���� */
// ����ջ��
#define AppCreate_Task_Stack_Deep 128
// �����ջ
StackType_t AppCreate_Task_Stack[AppCreate_Task_Stack_Deep];
// ������
TaskHandle_t AppCreate_Task_Handle;
// ������
void AppCreate_Task(void *pvParameters);

/* U3RX_Analyzing ���� */
// ����ջ��
#define U3RX_Analyzing_Task_Stack_Deep 128
// �����ջ
StackType_t U3RX_Analyzing_Task_Stack[U3RX_Analyzing_Task_Stack_Deep];
// ������
TaskHandle_t U3RX_Analyzing_Task_Handle;
// ������
void U3RX_Analyzing_Task(void *pvParameters);

/* LED ���� */
// ����ջ��
#define LED_Task_Stack_Deep 128
// �����ջ
StackType_t LED_Task_Stack[LED_Task_Stack_Deep];
// ������
TaskHandle_t LED_Task_Handle;
// ������
void LED_Task(void *pvParameters);

/* LED2 ���� */
// ����ջ��
#define LED2_Task_Stack_Deep 128
// �����ջ
StackType_t LED2_Task_Stack[LED2_Task_Stack_Deep];
// ������
TaskHandle_t LED2_Task_Handle;
// ������
void LED2_Task(void *pvParameters);

/* Ranging ���� */
// ����ջ��
#define Ranging_Task_Stack_Deep 128
// �����ջ
StackType_t Ranging_Task_Stack[Ranging_Task_Stack_Deep];
// ������
TaskHandle_t Ranging_Task_Handle;
// ������
void Ranging_Task(void *pvParameters);

/* Run ���� */
// ����ջ��
#define Run_Task_Stack_Deep 128
// �����ջ
StackType_t Run_Task_Stack[Run_Task_Stack_Deep];
// ������
TaskHandle_t Run_Task_Handle;
// ������
void Run_Task(void *pvParameters);

/* OLEDShowing ���� */
// ����ջ��
#define OLEDShowing_Task_Stack_Deep 256
// �����ջ
StackType_t OLEDShowing_Task_Stack[OLEDShowing_Task_Stack_Deep];
// ������
TaskHandle_t OLEDShowing_Task_Handle;
// ������
void OLEDShowing_Task(void *pvParameters);

/* SpeedDetection ���� */
// ����ջ��
#define SpeedDetection_Task_Stack_Deep 256
// �����ջ
StackType_t SpeedDetection_Task_Stack[SpeedDetection_Task_Stack_Deep];
// ������
TaskHandle_t SpeedDetection_Task_Handle;
// ������
void SpeedDetection_Task(void *pvParameters);

/**************************** ȫ�ֱ��������� ********************************/
float distance = 0;//С����ǰ����������(��λcm)
u16 T0CCP1_STA = 0; //���벶��״̬ bit15��ʾ�Ƿ����һ�����岶��bit14��ʾ�Ƿ���������һ�α仯�أ�bit13~bit0��ʾ�������ʱ��(T0CCP1_STA++��䴥������)

int16_t roll; //��X����ת
int16_t pitch;//��Y����ת
int16_t yaw;  //��Z����ת

u8 trackState;//ѭ��8·������bit0��ӦOUT1~bit7��ӦOUT8

u8 commands[UART3_REC_LEN+1];//Zigbee���������������,commands[0]�����������Ƿ�ѻ���־λ����һ��ʾ���ڶѻ�

int16_t lPwmVal;// PWM ռ�ձ�
int16_t rPwmVal;// PWM ռ�ձ�

u8 carMode = 0;//С������ģʽ��0��ʾ�ⲿ���ƣ�1��ʾѰ��ģʽ��2Ϊ����

u8 ackFlag = 0;// Ӧ���־λ���յ�Ӧ��ʱ��һ�����������

u32 lPulseCounter = 0;//���ֱ��������������
u32 rPulseCounter = 0;//���ֱ��������������
u16 lPulseCounterTime = 0xFF;//���ֱ����������������
u16 rPulseCounterTime = 0xFF;//���ֱ����������������

float lSpeed;
float rSpeed;
float speed;
void setup(void) //����0��ʼ��
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //����ϵͳʱ��Ϊ80MHz
	LED_Init();
    BEEP_Init();
	Uart0_Init(115200);//������
	Uart3_Init(115200);
	Uart5_Init(115200);//���MPU6050ģ�鴮��
	Time0A_Init(800-1);//ϵͳƵ��Ϊ80Mhz��800/80000000=10us,ʵ��10us���ж�
}

int main(void)
{
	setup(); // ��ʼ��
	int Sysclock = SysCtlClockGet();
    printf("\r\nProgram is Run-begining\r\nϵͳʱ��Ƶ��Ϊ%u\r\n",Sysclock);
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
                                               
    //���� U3RX_Analyzing ����
    xTaskCreate((TaskFunction_t)	U3RX_Analyzing_Task,				// ������
                (const char *)	"U3RX_Analyzing_Task",				    // ��������
                (uint16_t)		U3RX_Analyzing_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					// ���ݸ��������Ĳ���
                (UBaseType_t)	3,						    // �������ȼ�
                (TaskHandle_t *)	&U3RX_Analyzing_Task_Handle);		// ������
    
    //���� LED_Task ����
    xTaskCreate((TaskFunction_t)	LED_Task,				// ������
                (const char *)	"LED_Task",				    // ��������
                (uint16_t)		LED_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					// ���ݸ��������Ĳ���
                (UBaseType_t)	3,						    // �������ȼ�
                (TaskHandle_t *)	&LED_Task_Handle);		// ������

    //���� LED2_Task ����
    xTaskCreate((TaskFunction_t)	LED2_Task,				// ������
                (const char *)	"LED2_Task",				// ��������
                (uint16_t)		LED2_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					// ���ݸ��������Ĳ���
                (UBaseType_t)	3,						    // �������ȼ�
                (TaskHandle_t *)	&LED2_Task_Handle);		// ������

    //���� Ranging_Task ����
    xTaskCreate((TaskFunction_t)	Ranging_Task,				// ������
                (const char *)	"Ranging_Task",				    // ��������
                (uint16_t)		Ranging_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					    // ���ݸ��������Ĳ���
                (UBaseType_t)	3,						        // �������ȼ�
                (TaskHandle_t *)	&Ranging_Task_Handle);		// ������

    //���� Run_Task ����
    xTaskCreate((TaskFunction_t)	Run_Task,				// ������
                (const char *)	"Run_Task",				    // ��������
                (uint16_t)		Run_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					// ���ݸ��������Ĳ���
                (UBaseType_t)	3,						    // �������ȼ�
                (TaskHandle_t *)	&Run_Task_Handle);		// ������

    //���� OLEDShowing_Task ����
    xTaskCreate((TaskFunction_t)	OLEDShowing_Task,		    // ������
                (const char *)	"OLEDShowing_Task",				// ��������
                (uint16_t)		OLEDShowing_Task_Stack_Deep,	// �����ջ��С
                (void *)			NULL,					    // ���ݸ��������Ĳ���
                (UBaseType_t)	3,						        // �������ȼ�
                (TaskHandle_t *)	&OLEDShowing_Task_Handle);	// ������ 

    //���� SpeedDetection_Task ����
    xTaskCreate((TaskFunction_t)	SpeedDetection_Task,		    // ������
                (const char *)	"SpeedDetection_Task",				// ��������
                (uint16_t)		SpeedDetection_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					        // ���ݸ��������Ĳ���
                (UBaseType_t)	3,						            // �������ȼ�
                (TaskHandle_t *)	&SpeedDetection_Task_Handle);	// ������ 

    vTaskDelete(AppCreate_Task_Handle);//ɾ�� AppCreate ����
    taskEXIT_CRITICAL();//�˳��ٽ���
}

/**
  * @brief ������������
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
            //��������ÿ50ms����һ�ε�ǰ�ٶ�
            lSpeed = PulseCounter_*ONEPULSE_FOR_DISTANCE/PulseCounterTime_;//������ٶȣ���λ��m/s
        }
        if(rPulseCounterTime >= 50){
            PulseCounter_ = rPulseCounter;
            rPulseCounter = 0;
            PulseCounterTime_ = rPulseCounterTime;
            rPulseCounterTime = 0;
            //��������ÿ50ms����һ�ε�ǰ�ٶ�
            rSpeed = PulseCounter_*ONEPULSE_FOR_DISTANCE/PulseCounterTime_;//������ٶȣ���λ��m/s
        }
        speed = (rSpeed+lSpeed)/2;
		if(speed > 0)   
			printf("��ǰ����Ϊ:%.2lfm/s,����Ϊ:%.2lfm/s,�ҳ���Ϊ:%.2lfm/s\r\n", speed, lSpeed, rSpeed);
        vTaskDelay(30);
    }
}

/**
  * @brief OLED��ʾ��������
  */
static void OLEDShowing_Task(void *parameter)
{
    char str[128] = {0}; // ��Ҫ��ʾ���ַ�����ʱ��Ŵ�
    // ��ʼ��u8g2
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
            //�ֶ�����ģʽ��ʾ���� 
            static short i,j;
            static short xTemp = 0;
            static short yTemp = 8-1;
            static u8 widTemp = 8;
            static u8 highTemp = 8;
            u8g2_SetDrawColor(&u8g2, 1); // ������ɫ
            i = xTemp;
            j = 8-1;
            while(j >= yTemp){
                if(j > yTemp)
                    i = 16-1;
                else
                    i = xTemp;
                while(i >= 0){
                    u8g2_DrawBox(&u8g2, i*widTemp, j*highTemp, widTemp, highTemp); // ���Ʒ���
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
                u8g2_SendBuffer(&u8g2); // ͬ����Ļ
                vTaskDelay(300);
                u8g2_ClearBuffer(&u8g2); // clear the u8g2 buffer
                u8g2_SendBuffer(&u8g2); // ͬ����Ļ
                vTaskDelay(100);
            }

        }
        u8g2_SendBuffer(&u8g2); // ͬ����Ļ
    }
}

/**
  * @brief    Run_Task ������
  * @brief    �Ե缫��һ���ջ������������񣬻��Ŀ���ٶ� -> PID -> ���Ŀ���ٶ�
  */
void Run_Task(void *pvParameters){
    int8_t trackErr;
    int16_t commonPwmVal = 20;
    // int8_t lPwmVal = commonPwmVal,rPwmVal = commonPwmVal;
    rPwmVal = commonPwmVal;
    lPwmVal = commonPwmVal;
    int16_t pidResult;
    PID *trackPID = PID_Position_Create(10,0,0,100,200);
    Ganv_Track_Init();// ��ʼ��Ѱ��ģ��
    Motor_Init();// ��ʼ���������IO��һ��������ת��һ����PWM���٣�����Ϊ3200����ʼռ�ձ�Ϊ0��Ƶ��Ϊ25Khz
    while(1){
        if(carMode == 1){
			if(Ganv_Get_Err_State() == 1){
                //ѭ������
                //����
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
  * @brief    Ranging_Task ������
  * @brief    ͨ������ T2CCP0 ��������峤�ȵõ�С����ǰ����������
  */
void Ranging_Task(void *pvParameters){
    u16 ultrasonicTimes = 0;
	HC_SR04_Init();
    while(1){
        if(T0CCP1_STA&0x8000){
            //���һ�����������岶��
			ultrasonicTimes = T0CCP1_STA&0x3FFF;
			T0CCP1_STA = 0;
			distance = Filter(ultrasonicTimes*0.17, distance, 0.8);
			printf("Time=%hu, distance=%.1fcm\r\n",ultrasonicTimes,distance);
        }
    }
}

/**
  * @brief    LED2_Task ������
  * @brief    ��˸�ƣ����ֳ�����������
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
			case 1://����1�����£��л�С��ģʽ
				if(carMode == 0)
					carMode = 1;
				else if(carMode == 1){
					carMode = 0;
					vTaskDelay(20);
					Motor_Stop();
				}
				break;
			case 2://δ��
				break;
			default:
				break;
		}
    }
}

/**
  * @brief    LED_Task ������
  * @brief    ��˸�ƣ����ֳ�����������
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
  * @brief    U3RX_Analyzing ������
  * @brief    ����������ͨ�Ŵ��������
  */
void U3RX_Analyzing_Task(void *pvParameters){
    while (1){
        while(commands[0] != 1);//�ȴ���־λ����1
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
                    //�յ�Ӧ��
                    ackFlag = 1;
                    break;
                }
            default:
                //ָ�����
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
        commands[0] = 0;//�����־λ
    }
}


//T0CCP1���жϷ�����
void TIMER0B_Handler(void){
    //����жϱ�־λ
	TimerIntClear(TIMER0_BASE,  TimerIntStatus(TIMER0_BASE, true));
    if(T0CCP1_STA & 0x4000){
        //�����һ�������أ�������½�������
        T0CCP1_STA |= 0x8000;//������һ�θߵ�ƽ���岶��
        TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE); //��������Ϊ�����ز��� 
        T0CCP1_STA &= ~0x4000;//����
    }
    else{
        //��һ�β���������
        //��գ���ʼ��ʱ�ȴ��½���
        T0CCP1_STA = 0;
        T0CCP1_STA |= 0x4000;//��ǲ�����������
        TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_NEG_EDGE); //����Ϊ�½��ز��� 
    }
}

//T1CCP0���жϷ�����
//�����ڲ������ֻ�����������
void TIMER1A_Handler(void){
    uint32_t flag = TimerIntStatus(TIMER1_BASE, true);
    if(flag == TIMER_CAPA_EVENT){
        //�ۼ�������
        rPulseCounter++;
    }
    TimerIntClear(TIMER1_BASE,  flag);
}

//T1CCP1���жϷ�����
//�����ڲ������ֻ�����������
void TIMER1B_Handler(void){
    uint32_t flag = TimerIntStatus(TIMER1_BASE, true);
    if(flag == TIMER_CAPB_EVENT){
        //�ۼ�������
        lPulseCounter++;
    }
    TimerIntClear(TIMER1_BASE,  flag);
}


//10us����ʱ����ʵ��ϵͳ��ʱ
void TIMER0A_Handler(void){
    u8 i;
    static u16 usecond = 0;
    static u16 msecond = 0;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    //ms����ʱ
    if(usecond == 100){
        usecond = 0;
        //������1ms

		if(lPulseCounterTime < 0xFFFF)
            lPulseCounterTime++;
		if(rPulseCounterTime < 0xFFFF)
            rPulseCounterTime++;
		
        if(uart0RXTime < 30){
            uart0RXTime++;
        }
        else if(uart0RXTime == 30){
            // ��������һ��������Ϣ
            if (UART0_RX_BUF[0] == 0xA1 && UART0_RX_BUF[1] == 0xA1 && UART0_RX_BUF[UART0_RX_STA-1] == 0xB1 && UART0_RX_BUF[UART0_RX_STA-2] == 0xB1) {                
                if(commands[0] == 0){
                    //�����������Ϊ�գ�����д������ 
                    for (i = 0; i < UART0_RX_STA-4; i++) {
                        commands[1+i] = UART0_RX_BUF[2+i];
                    }
                    commands[0] = 1;//������������ǿ�
                }
            }
            UART0_RX_STA = 0;
            uart0RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
        }

        if(uart3RXTime < 50){
            uart3RXTime++;
        }
        else if(uart3RXTime == 50){
            // ��������һ��������Ϣ
            if (UART3_RX_BUF[0] == 0xA1 && UART3_RX_BUF[1] == 0xA1 && UART3_RX_BUF[UART3_RX_STA-1] == 0xB1 && UART3_RX_BUF[UART3_RX_STA-2] == 0xB1) {
                if(commands[0] == 0){
                    //�����������Ϊ�գ�����д������ 
                    for (i = 0; i < UART3_RX_STA-4; i++) {
                        commands[1+i] = UART3_RX_BUF[2+i];
                    }
                    commands[0] = 1;//������������ǿ�
                }
            }
            UART3_RX_STA = 0;
            uart3RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
        }

        if(uart5RXTime < 20){
            uart5RXTime++;
        }
        else if(uart5RXTime == 20){
            // ��������һ��������Ϣ
            if (UART5_RX_BUF[0] == 0xA2 && UART5_RX_BUF[1] == 0xA2 && UART5_RX_BUF[8] == 0xB2 && UART5_RX_BUF[9] == 0xB2) {
                pitch   = (((int16_t)UART5_RX_BUF[2])<<8)+UART5_RX_BUF[3];
                roll    = (((int16_t)UART5_RX_BUF[4])<<8)+UART5_RX_BUF[5];
                yaw     = (((int16_t)UART5_RX_BUF[6])<<8)+UART5_RX_BUF[7];
            }
            UART5_RX_STA = 0;
            uart5RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
        }

        if(msecond == 1000){
            msecond = 0;
            //������1s
        }
        msecond++;
    }
	
	//HC-SR04
	if((T0CCP1_STA & 0X8000)==0 && (T0CCP1_STA & 0X4000)){
		//��δ��ɲ��񣬵����Ѿ����񵽸ߵ�ƽ��
		if((T0CCP1_STA&0X3FFF) >= 0x3FFF){
			//�ߵ�ƽ̫����(����ʱ�����700*10us)(������̫Զ�ˣ��Ͳ����ˡ�ע�⣬����С��5cmҲ�ᵼ�ºܳ��ĸߵ�ƽʱ�䣬�������ģ�����̲�������������5cn)
			T0CCP1_STA |= 0xFFFF;
		}
		else T0CCP1_STA++;
	}

    usecond++;
}

//����0�ж�
void UART0_Handler(void){
    uint32_t flag = UARTIntStatus(UART0_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART0_BASE)){
		    //if FIFO have data
            if (UART0_RX_STA < UART_REC_LEN) {
                uart0RXTime = 0;
                UART0_RX_BUF[UART0_RX_STA] = UARTCharGet(UART0_BASE); // ��ȡ���յ�������
                UART0_RX_STA++;
            } else {
                UART0_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART0_BASE,flag);
}

//����3�ж�
void UART3_Handler(void){
    uint32_t flag = UARTIntStatus(UART3_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART3_BASE)){
		    //if FIFO have data
            if (UART3_RX_STA < UART_REC_LEN) {
                uart3RXTime = 0;
                UART3_RX_BUF[UART3_RX_STA] = UARTCharGet(UART3_BASE); // ��ȡ���յ�������
                UART3_RX_STA++;
            } else {
                UART3_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART3_BASE,flag);
}

//����5�ж�
void UART5_Handler(void){
    uint32_t flag = UARTIntStatus(UART5_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART5_BASE)){
		    //if FIFO have data
            if (UART5_RX_STA < UART5_REC_LEN) {
                uart5RXTime = 0;
                UART5_RX_BUF[UART5_RX_STA] = UARTCharGet(UART5_BASE); // ��ȡ���յ�������
                UART5_RX_STA++;
            } else {
                UART5_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART5_BASE,flag);
}






