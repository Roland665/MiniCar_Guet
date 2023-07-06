#include "IIC_Soft.h"

void IIC_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(IIC_SCL_GPIOCLK|IIC_SDA_GPIOCLK, ENABLE);//ʹ��GPIOʱ��

  //GPIOB8,B9��ʼ������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = IIC_SCL_Pin;
	GPIO_Init(IIC_SCL_GPIO, &GPIO_InitStructure);//��ʼ��
	GPIO_InitStructure.GPIO_Pin = IIC_SDA_Pin;
	GPIO_Init(IIC_SDA_GPIO, &GPIO_InitStructure);//��ʼ��
	IIC_SCL_High;
	IIC_SDA_High;
}
void IIC_Start(void){
    SDA_OUT();//SDA�����
	//����IIC�����ǿ���״̬��û���˶���������SDA/SCL�������ߵ�Ƶ
    //���и�������������ϵڶ���ͼ��SDA���п�����RA���õģ����Ե���֮��SDA�п�����0�п�����1
	//����Ϊ�ˡ�S�����ģ���ܹ�ƴ�ϣ��Ȱ�SDA��1�����ߣ���Ϊʲô�أ���ΪSDA�п�����0�п�����1��
    IIC_SDA_High;
    //��SCL������ƴͼ��������0��������ȷ��SDA��1���ٰ�SCL����
    IIC_SCL_High;
    //�������Ǹտ�ʼ����������û�ã�������ǳ�����е�һ���ˣ���ʹ�õ�����������Ǿ���Ҫ�������������һ������
    //Ȼ��������ϵ�8��ͼ,�Ȱ�SDA���ͣ��ٰ�SCL����
    IIC_SDA_Low;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
    IIC_SCL_Low;//ǯס IIC ���ߣ�׼�����ͻ��������
}//���Ͼ���Start���������򵥡�д��

//�������� Stop����
void IIC_Stop(void){
	SDA_OUT();//sda �����
	IIC_SCL_Low;
	//�ȱ�֤SDA��0,�����Ȱ�SDA����
    IIC_SDA_Low;
	delay_us(4);
    //�ٰ�SCL����
    IIC_SCL_High;
    //Ȼ���ٰ�SDA����
    IIC_SDA_High;//STOP:when CLK is high DATA change form low to high
	delay_us(4);
}
                
//�ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void){
	u8 ucErrTime=0;
    //�Ȱ�SDA���ߣ��ٰ�SCL����
	SDA_IN();
    IIC_SDA_High;
	delay_us(1);
    IIC_SCL_High;
	delay_us(1);
	while(READ_SDA) {
		ucErrTime++; 
		if(ucErrTime>250){
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_Low;//ʱ����� 0          
	return 0;
}

//���� ACK Ӧ�� 
void IIC_Ack(void) { 
	IIC_SCL_Low;
	SDA_OUT(); 
	IIC_SDA_Low;
	delay_us(2); 
	IIC_SCL_High;
	delay_us(2); 
	IIC_SCL_Low; 
} 
//������ ACK Ӧ��           
void IIC_NAck(void) { 
	IIC_SCL_Low; 
	SDA_OUT(); 
	IIC_SDA_High;delay_us(2); 
	IIC_SCL_High;delay_us(2); 
	IIC_SCL_Low; 
}

//������д Sendbyte����
void IIC_Send_Byte(u8 Byte){
    unsigned char i;
	SDA_OUT();
	//�Ȱ�Ҫ�����ֽڵ����λ����SDA���ٰ�SCL�ø�,�ٰ�SCL�õ�
    //IIC_SDA = Byte & 0x80;//��Byte���λȡ��������SDA
    //IIC_SCL_High;
 	//IIC_SCL_Low;
    //���������� ѭ��8�Σ�������ֱ��ע�͵�����ģ���������д��
    for(i = 0; i < 8; i++){
		if((Byte&0x80)>>7 == 0)IIC_SDA_Low;
		else IIC_SDA_High;
        Byte<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
        IIC_SCL_High;
		delay_us(2);
        IIC_SCL_Low;
		delay_us(2);
    }
}

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL_Low; 
        delay_us(2);
		IIC_SCL_High;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}

