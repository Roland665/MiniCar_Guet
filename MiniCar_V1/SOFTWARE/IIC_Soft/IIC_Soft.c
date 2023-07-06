#include "IIC_Soft.h"

void IIC_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(IIC_SCL_GPIOCLK|IIC_SDA_GPIOCLK, ENABLE);//使能GPIO时钟

  //GPIOB8,B9初始化设置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = IIC_SCL_Pin;
	GPIO_Init(IIC_SCL_GPIO, &GPIO_InitStructure);//初始化
	GPIO_InitStructure.GPIO_Pin = IIC_SDA_Pin;
	GPIO_Init(IIC_SDA_GPIO, &GPIO_InitStructure);//初始化
	IIC_SCL_High;
	IIC_SDA_High;
}
void IIC_Start(void){
    SDA_OUT();//SDA线输出
	//开局IIC总线是空闲状态，没有人动他，所以SDA/SCL是两个高电频
    //还有个情况：根据往上第二张图，SDA还有可能是RA调用的，所以调用之后SDA有可能是0有可能是1
	//所以为了“S”这个模块能够拼上，先把SDA置1（拉高）。为什么呢？因为SDA有可能是0有可能是1，
    IIC_SDA_High;
    //而SCL在其他拼图结束后都是0，所以先确保SDA是1，再把SCL拉高
    IIC_SCL_High;
    //如果这就是刚开始，那上两句没用，如果这是程序进行到一半了，才使用到这个函数，那就需要按上面的流程来一遍拉高
    //然后根据往上第8个图,先把SDA拉低，再把SCL拉低
    IIC_SDA_Low;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
    IIC_SCL_Low;//钳住 IIC 总线，准备发送或接收数据
}//以上就是Start函数，“简单”写完

//接下来是 Stop函数
void IIC_Stop(void){
	SDA_OUT();//sda 线输出
	IIC_SCL_Low;
	//先保证SDA是0,所以先把SDA拉低
    IIC_SDA_Low;
	delay_us(4);
    //再把SCL拉高
    IIC_SCL_High;
    //然后再把SDA拉高
    IIC_SDA_High;//STOP:when CLK is high DATA change form low to high
	delay_us(4);
}
                
//等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void){
	u8 ucErrTime=0;
    //先把SDA拉高，再把SCL拉高
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
	IIC_SCL_Low;//时钟输出 0          
	return 0;
}

//产生 ACK 应答 
void IIC_Ack(void) { 
	IIC_SCL_Low;
	SDA_OUT(); 
	IIC_SDA_Low;
	delay_us(2); 
	IIC_SCL_High;
	delay_us(2); 
	IIC_SCL_Low; 
} 
//不产生 ACK 应答           
void IIC_NAck(void) { 
	IIC_SCL_Low; 
	SDA_OUT(); 
	IIC_SDA_High;delay_us(2); 
	IIC_SCL_High;delay_us(2); 
	IIC_SCL_Low; 
}

//接下来写 Sendbyte函数
void IIC_Send_Byte(u8 Byte){
    unsigned char i;
	SDA_OUT();
	//先把要传输字节的最高位赋给SDA，再把SCL置高,再把SCL置低
    //IIC_SDA = Byte & 0x80;//把Byte最高位取出来赋给SDA
    //IIC_SCL_High;
 	//IIC_SCL_Low;
    //把上述三行 循环8次（这里我直接注释掉上面的，在下面重写）
    for(i = 0; i < 8; i++){
		if((Byte&0x80)>>7 == 0)IIC_SDA_Low;
		else IIC_SDA_High;
        Byte<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
        IIC_SCL_High;
		delay_us(2);
        IIC_SCL_Low;
		delay_us(2);
    }
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}

