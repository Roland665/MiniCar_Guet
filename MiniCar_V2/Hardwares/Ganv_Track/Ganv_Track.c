#include "Ganv_Track.h"
#include "IIC_Hard/IIC_Hard.h"

u8 GetDDFlag = 0;   //根据寻迹模块命令特性，当主机发出0xDD命令后，模块此后一直在此命令下工作，
                    //即可以一直读取数字量数据，并且模块上电默认在此命令下工作
                    //当此Flag被置1时，发起读取操作前不需要发送 0xDD 命令

u8 GetAnalogDataFlag = 0;   //根据寻迹模块命令特性，当主机发出0xB0命令后，模块此后一直在此命令下工作，
                            //即可以一直读取模拟量数据
                            //当此Flag被置1时，发起读取操作前不需要发送 0xB0 命令
/**
  * @brief  初始化感为电子的8路寻迹模块使用的IIC模块
  */
void Ganv_Track_Init(void){
	IIC2_Init();
}


/**
  * @brief  获取寻迹数字量数据(Digital Data)
  * @brief  由于模块特性，发送一次命令后可进行无数次间断性操作，所以不适用写好的IIC读写代码
  * @param  void
  * @retval 8位数字量数据
  */
u8 Ganv_Get_DD(void){
    if(GetDDFlag == 0){
        //发送0xDD命令
        IIC_Write_One_Byte(I2C2_BASE, GANV_ADDR, 0xDD);
		GetDDFlag = 1;
    }
    return IIC_Read_One_Byte(I2C2_BASE, GANV_ADDR);
}

/**
  * @brief  获取寻迹模拟量数据
  * @brief  采用手册上的方法一，确保读出的8位模拟量有序对应8个通道
  * @param  analogDatas: 存放模拟量数据数组
  * @retval 8位模拟量数据
  */
void Ganv_Get_Analog_Data(u8 *analogDatas){
    IIC_Register_Read_Len_Byte(I2C2_BASE, GANV_ADDR, 0xB0, 8, analogDatas);
}

/**
  * @brief  校验感为灰度传感器是否正常工作
  * @brief  如果总线连接完整，地址正确，I2C 程序无误，给灰度传感器发送 0xAA 命令，灰度传感器将返回 数据 0x66 以证明其工作正常、总线完整。
  * @param  void
  * @retval void
  */
void Ganv_Ping(void){
    while(IIC_Register_Read_One_Byte(I2C2_BASE, GANV_ADDR, 0xAA) != 0x66);
}

/**
  * @brief  计算寻迹数字量的差值
  * @param  trackState: 8位数字量寻迹状态
  * @retval 返回差值，供PID使用
  */
int8_t Ganv_Calc_DD_Err(u8 trackState){
    switch (trackState)
    {
    // 直行
    case 0xE7://11100111
        return 0;
    case 0xFF://11111111
        return 0;
    
    // 偏右
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

    // 偏左
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
    

    default:
        return 66;
    }
}
