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
//#include "MPU6050/mpu6050.h"
#include "PID/PID.h"
#include "BEEP/beep.h"
#include "MOTOR/motor.h"
#include "u8g2_user.h"
#include "Ganv_Track/Ganv_Track.h"

/*******************************************/
/*
@version V2.0
�����V1ϵ��
  - ʹ����ˢ�����϶������
  - ����mcu��ΪTM4C123GH6
  - 

ʵ����
  - 
*/

/**************************** �ں˶������� ********************************/
/*�����Դ洢 UART1_REC_LEN �� u8���ͱ����Ķ��� */
#define U1RX_QUEUE_LENGTH         UART1_REC_LEN
#define U1RX_QUEUE_ITEM_SIZE      sizeof(u8)
/*����1������Ϣ���о��*/
static QueueHandle_t U1RX_QUEUE_Handle = NULL;
/*����1��Ϣ���ж���*/
static StaticQueue_t U1RX_QUEUE_Structure;
/*����1��Ϣ���еĴ洢���򣬴�С������ uxQueueLength * uxItemSize ���ֽ� */
uint8_t U1RX_QUEUE_Storage_Area[U1RX_QUEUE_LENGTH * U1RX_QUEUE_ITEM_SIZE];


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

/* U1RX_Analyzing ���� */
// ����ջ��
#define U1RX_Analyzing_Task_Stack_Deep 128
// �����ջ
StackType_t U1RX_Analyzing_Task_Stack[U1RX_Analyzing_Task_Stack_Deep];
// ������
TaskHandle_t U1RX_Analyzing_Task_Handle;
// ������
void U1RX_Analyzing_Task(void *pvParameters);

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

#ifdef MPU6050
/* MPU6050_Sensor ���� */
// ����ջ��
#define MPU6050_Sensor_Task_Stack_Deep 128
// �����ջ
StackType_t MPU6050_Sensor_Task_Stack[MPU6050_Sensor_Task_Stack_Deep];
// ������
TaskHandle_t MPU6050_Sensor_Task_Handle;
// ������
void MPU6050_Sensor_Task(void *pvParameters);
#endif

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

/**************************** ȫ�ֱ��������� ********************************/
float distance = 0;//С����ǰ����������(��λcm)
u16 T2CCP0_STA = 0; //���벶��״̬ bit15��ʾ�Ƿ����һ�����岶��bit14��ʾ�Ƿ���������һ�α仯�أ�bit13~bit0��ʾ�������ʱ��(T2CCP0_STA++��䴥������)

float roll; //��X����ת
float pitch;//��Y����ת
float yaw;  //��Z����ת

u8 trackState;//ѭ��8·������bit0��ӦOUT1~bit7��ӦOUT8

u8 commands[UART1_REC_LEN+1];//Zigbee���������������,commands[0]�����������Ƿ�ѻ���־λ����һ��ʾ���ڶѻ�

int16_t lPwmVal;
int16_t rPwmVal;

u8 carMode = 0;//С������ģʽ��0��ʾ�ⲿ���ƣ�1��ʾѰ��ģʽ��2Ϊ����
void setup(void) //����0��ʼ��
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //����ϵͳʱ��Ϊ80MHz
	LED_Init();
    BEEP_Init();
	Uart0_Init(115200);
	Uart1_Init(115200);
	Uart2_Init(115200);
	HC_SR04_Init();
	HC_SR04_Start();
	Time0A_Init(800-1);//ϵͳƵ��Ϊ80Mhz��800/80000000=10us,ʵ��10us���ж�
#ifdef MPU6050
    mpu6050_Init();
#endif
}

int main(void)
{
	setup(); // ��ʼ��
    printf("\r\nProgram is Run-begining\r\nϵͳʱ��Ƶ��Ϊ%u\r\n",SysCtlClockGet());
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
    //���� U1_QUEUE ��Ϣ����
    U1RX_QUEUE_Handle = xQueueCreateStatic(U1RX_QUEUE_LENGTH,               // �������
                                               U1RX_QUEUE_ITEM_SIZE,      // �������ݵ�Ԫ�ĵ�λ
                                               U1RX_QUEUE_Storage_Area,   // ���еĴ洢����
                                               &U1RX_QUEUE_Structure);    // ���е����ݽṹ
                                               
    //���� U1RX_Analyzing ����
    xTaskCreate((TaskFunction_t)	U1RX_Analyzing_Task,				// ������
                (const char *)	"U1RX_Analyzing_Task",				    // ��������
                (uint16_t)		U1RX_Analyzing_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					// ���ݸ��������Ĳ���
                (UBaseType_t)	3,						    // �������ȼ�
                (TaskHandle_t *)	&U1RX_Analyzing_Task_Handle);		// ������
    
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

#ifdef MPU6050
    //���� MPU6050_Sensor_Task ����
    xTaskCreate((TaskFunction_t)	MPU6050_Sensor_Task,			// ������
                (const char *)	"MPU6050_Sensor_Task",			    // ��������
                (uint16_t)		MPU6050_Sensor_Task_Stack_Deep,	    // �����ջ��С
                (void *)			NULL,					        // ���ݸ��������Ĳ���
                (UBaseType_t)	3,						            // �������ȼ�
                (TaskHandle_t *)	&MPU6050_Sensor_Task_Handle);	// ������
#endif

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

    vTaskDelete(AppCreate_Task_Handle);//ɾ�� AppCreate ����
    taskEXIT_CRITICAL();//�˳��ٽ���
}

/**
  * @brief OLED��ʾ��������
  * @brief
  * @param
  * @retval
  */
    u8 openFlag = 1;
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
        if(openFlag == 1){
            u8g2_SetFont(&u8g2, u8g2_font_8x13O_mf);
            sprintf(str, "%d %d %d %d %d %d %d %d",    (trackState>>7)&1,
                                                (trackState>>6)&1,
                                                (trackState>>5)&1, 
                                                (trackState>>4)&1, 
                                                (trackState>>3)&1, 
                                                (trackState>>2)&1, 
                                                (trackState>>1)&1, 
                                                (trackState>>0)&1);
            u8g2_DrawStr(&u8g2, 5, 37, str);
            sprintf(str, "%d   %d", lPwmVal, rPwmVal);
            u8g2_DrawStr(&u8g2, 30, 60, str);
        }
        else{
            //������ʾ���� 
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
    int16_t commonPwmVal = 2000;
    // int8_t lPwmVal = commonPwmVal,rPwmVal = commonPwmVal;
    rPwmVal = commonPwmVal;
    lPwmVal = commonPwmVal;
    int16_t pidResult;
    PID *trackPID = PID_Position_Create(400,0,0,5000,100000);
    Ganv_Track_Init();// ��ʼ��Ѱ��ģ��
    Motor_Init();// ��ʼ���������IO��һ��������ת��һ����PWM���٣�����Ϊ3200����ʼռ�ձ�Ϊ0��Ƶ��Ϊ25Khz
    while(1){
        //Get the track state
        trackState = Ganv_Get_DD();
        trackErr = Ganv_Calc_DD_Err(trackState);
		if(trackErr == 66){
			rPwmVal = 0;
			lPwmVal = 0;
		}
		else{
			pidResult = PID_Position(trackPID, trackErr);
			rPwmVal = commonPwmVal + pidResult;
			lPwmVal = commonPwmVal - pidResult;
		}
        Motor_SetSpeed(lPwmVal,rPwmVal);
        vTaskDelay(10);
    }
}

#ifdef MPU6050
/**
  * @brief    MPU6050_Sensor_Task ������(δ��Ʒ)
  * @brief    ͨ����MPU6050�����ݽ����ںϼ��㣬�ó�С����ת�Ƕ�
  */
void MPU6050_Sensor_Task(void *pvParameters){
    mpu6050_t mympu6050;
    while(1){
        printf("accx=%d  accy=%d  accz=%d\r\n",mympu6050.accX,mympu6050.accY,mympu6050.accZ);
	    printf("gyroX=%d  gyroY=%d  gyroZ=%d\r\n",mympu6050.gyroX,mympu6050.gyroY,mympu6050.gyroZ);
        mpu6050_Get_Data(&mympu6050);
		vTaskDelay(1000);
    }
}
#endif

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
  * @brief    LED2_Task ������
  * @brief    ��˸�ƣ����ֳ�����������
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
  * @brief    LED_Task ������
  * @brief    ��˸�ƣ����ֳ�����������
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
  * @brief    U1RX_Analyzing ������
  * @brief    ����������ͨ�Ŵ��������
  */
void U1RX_Analyzing_Task(void *pvParameters){
    while (1){
        while(commands[0] != 1);//�ȴ���־λ����1
        BEEP_ENABLE;
        if(commands[1] == 0xFF){
            openFlag = commands[2];
        }
        vTaskDelay(100);
        BEEP_DISABLE;
        commands[0] = 0;//�����־λ
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
    u8 i;
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

        if(uart1RXTime < 50){
            uart1RXTime++;
        }
        else if(uart1RXTime == 50){
            // ��������һ��������Ϣ
            if (UART1_RX_BUF[0] == 0xA1 && UART1_RX_BUF[1] == 0xA1 && UART1_RX_BUF[UART1_RX_STA-1] == 0xB1 && UART1_RX_BUF[UART1_RX_STA-2] == 0xB1) {
                if(commands[0] == 0){
                    //�����������Ϊ�գ�����д������ 
                    for (i = 0; i < UART1_RX_STA-4; i++) {
                        commands[1+i] = UART1_RX_BUF[2+i];
                    }
                    commands[0] = 1;//������������ǿ�
                }
            }
            UART1_RX_STA = 0;
            uart1RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
        }

        if(uart2RXTime < 50){
            uart2RXTime++;
        }
        else if(uart2RXTime == 50){
            // ��������һ��������Ϣ
            if (UART2_RX_BUF[0] == 0xA2 && UART2_RX_BUF[1] == 0xA2 && UART2_RX_BUF[5] == 0xB2 && UART2_RX_BUF[6] == 0xB2) {
                roll = UART2_RX_BUF[2];
                pitch = UART2_RX_BUF[3];
                yaw = UART2_RX_BUF[4];
            }
            UART2_RX_STA = 0;
            uart2RXTime  = 0xFF; // ��ʱ����������ʾû���յ��µ���Ϣ
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
        HC_SR04_TRIG_DISABLE;
        TrigFlag = 2;
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

//����1�ж�
void UART1_Handler(void){
    uint32_t flag = UARTIntStatus(UART1_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART1_BASE)){
		    //if FIFO have data
            if (UART1_RX_STA < UART_REC_LEN) {
                uart1RXTime = 0;
                UART1_RX_BUF[UART1_RX_STA] = UARTCharGet(UART1_BASE); // ��ȡ���յ�������
                UART1_RX_STA++;
            } else {
                UART1_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART1_BASE,flag);
}

//����2�ж�
void UART2_Handler(void){
    uint32_t flag = UARTIntStatus(UART2_BASE,true);
    if(flag == UART_INT_RX || flag == UART_INT_RT){
        while(UARTCharsAvail(UART2_BASE)){
		    //if FIFO have data
            if (UART2_RX_STA < UART2_REC_LEN) {
                uart2RXTime = 0;
                UART2_RX_BUF[UART2_RX_STA] = UARTCharGet(UART2_BASE); // ��ȡ���յ�������
                UART2_RX_STA++;
            } else {
                UART2_RX_STA = 0;
            }
        }
    }
    UARTIntClear(UART2_BASE,flag);
}






