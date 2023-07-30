#include "Ganv_Track.h"
#include "IIC_Hard/IIC_Hard.h"

u8 GetDDFlag = 0;   //����Ѱ��ģ���������ԣ�����������0xDD�����ģ��˺�һֱ�ڴ������¹�����
                    //������һֱ��ȡ���������ݣ�����ģ���ϵ�Ĭ���ڴ������¹���
                    //����Flag����1ʱ�������ȡ����ǰ����Ҫ���� 0xDD ����

u8 GetAnalogDataFlag = 0;   //����Ѱ��ģ���������ԣ�����������0xB0�����ģ��˺�һֱ�ڴ������¹�����
                            //������һֱ��ȡģ��������
                            //����Flag����1ʱ�������ȡ����ǰ����Ҫ���� 0xB0 ����
/**
  * @brief  ��ʼ��������Ϊ���ӵ�8·Ѱ��ģ������輰IO
  */
void Ganv_Track_Init(void){
    Ganv_Err_IO_Init();
	IIC1_Init();
}

/**
  * @brief  ��ʼ����Ѱ��ģ��Err��������������IO
  */
void Ganv_Err_IO_Init(void){
    SysCtlPeripheralEnable(GANV_ERR_GPIOPERIPH);
    GPIODirModeSet(GANV_ERR_GPIO, GANV_ERR_PIN, GPIO_DIR_MODE_IN);
    GPIOPadConfigSet(GANV_ERR_GPIO,GANV_ERR_PIN,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);//����
}

/**
  * @brief  ��ȡѰ��ģ��Err IO�ڵ�״̬
  * @retval 0����Error
  * @retval 1������Error
  */
u8 Ganv_Get_Err_State(void){
    if(GPIOPinRead(GANV_ERR_GPIO, GANV_ERR_PIN) == GANV_ERR_PIN)
		return 1;
	return 0;
}

/**
  * @brief  ��ȡѰ������������(Digital Data)
  * @brief  ����ģ�����ԣ�����һ�������ɽ��������μ���Բ��������Բ�����д�õ�IIC��д����
  * @param  void
  * @retval 8λ����������
  */
u8 Ganv_Get_DD(void){
    if(GetDDFlag == 0){
        //����0xDD����
        IIC_Write_One_Byte(I2C1_BASE, GANV_ADDR, 0xDD);
		GetDDFlag = 1;
    }
    return IIC_Read_One_Byte(I2C1_BASE, GANV_ADDR);
}

/**
  * @brief  ��ȡѰ��ģ��������
  * @brief  �����ֲ��ϵķ���һ��ȷ��������8λģ���������Ӧ8��ͨ��
  * @param  analogDatas: ���ģ������������
  * @retval 8λģ��������
  */
void Ganv_Get_Analog_Data(u8 *analogDatas){
    IIC_Register_Read_Len_Byte(I2C1_BASE, GANV_ADDR, 0xB0, 8, analogDatas);
}

/**
  * @brief  У���Ϊ�Ҷȴ������Ƿ���������
  * @brief  �������������������ַ��ȷ��I2C �������󣬸��Ҷȴ��������� 0xAA ����Ҷȴ����������� ���� 0x66 ��֤���乤������������������
  * @param  void
  * @retval void
  */
void Ganv_Ping(void){
    while(IIC_Register_Read_One_Byte(I2C1_BASE, GANV_ADDR, 0xAA) != 0x66);
}

/**
  * @brief  ����Ѱ���������Ĳ�ֵ
  * @param  trackState: 8λ������Ѱ��״̬
  * @retval ���ز�ֵ����PIDʹ��
  */
int8_t Ganv_Calc_DD_Err(u8 trackState){
    switch (trackState)
    {
    // ֱ��
    case 0xE7://11100111
        return 0;
    case 0xFF://11111111
        return 0;
    
    // ƫ��
    case 0xF7://11110111
        return 1;
    case 0xE3://11100011
        return 1;
    case 0xF3://11110011
        return 2;
    case 0xFB://11111011
        return 3;
    case 0xF1://11110001
        return 3;
    case 0xF9://11111001
        return 4;
    case 0xFD://11111101
        return 5;
    case 0xF8://11111000
        return 5;
    case 0xFC://11111100
        return 6;
    case 0xFE://11111110
        return 7;

    // ƫ��
    case 0xEF://11101111
        return -1;
    case 0xC7://11000111
        return -1;
    case 0xCF://11001111
        return -2;
    case 0xDF://11011111
        return -3;
    case 0x8F://10001111
        return -3;
    case 0x9F://10011111
        return -4;
    case 0xBF://10111111
        return -5;
    case 0x1F://00011111
        return -5;
    case 0x3F://00111111
        return -6;
    case 0x7F://01111111
        return -7;
    
	//�������
	case 0x0F://00001111
		return -3;//��תֱ��
	case 0x07://00000111
		return -4;//��תֱ��
	case 0x03://00000011
		return -5;//��תֱ��
	case 0xF0://11110000
		return 3;//��תֱ��
	case 0xE0://11100000
		return 4;//��תֱ��
	case 0xC0://11000000
		return 5;//��תֱ��
			
    default:
        return 66;
    }
}
