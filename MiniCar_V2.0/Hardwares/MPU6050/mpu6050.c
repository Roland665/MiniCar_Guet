#include "mpu6050.h"
#include "IIC/IIC_Hard.h"

void mpu6050_Init(void){
	IIC3_Init();
	mpu6050_Write_Register(MPU6050_RA_PWR_MGMT_1,0x01);//不复位、解除睡眠、不需要循环、启用温度传感器、选择陀螺仪时钟
	mpu6050_Write_Register(MPU6050_RA_PWR_MGMT_2,0x00);//循环模式唤醒频率为0，每轴的待机位都为0，不待机
	mpu6050_Write_Register(MPU6050_RA_SMPLRT_DIV,0x09);//时钟十分频
	mpu6050_Write_Register(MPU6050_RA_CONFIG,0x06);//不需要外部同步，数字低通滤波器给110(说是最平滑的滤波)
	mpu6050_Write_Register(MPU6050_RA_GYRO_CONFIG,0x18);//陀螺仪：不自测、陀螺仪量程选择±2000°/s
	mpu6050_Write_Register(MPU6050_RA_ACCEL_CONFIG,0x18);//加速度计：不自测、量程选择±16、不用高通滤波器
}

void mpu6050_Get_Data(mpu6050_t* mpu6050_data){
	u8 dataH, dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_ACCEL_XOUT_H);//读取加速度寄存器高8位值
	dataL = mpu6050_Read_Register(MPU6050_RA_ACCEL_XOUT_L);//读取加速度寄存器低8位值
	mpu6050_data->accX = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_ACCEL_YOUT_H);//读取加速度寄存器高8位值
	dataL = mpu6050_Read_Register(MPU6050_RA_ACCEL_YOUT_L);//读取加速度寄存器低8位值
	mpu6050_data->accY = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_ACCEL_ZOUT_H);//读取加速度寄存器高8位值
	dataL = mpu6050_Read_Register(MPU6050_RA_ACCEL_ZOUT_L);//读取加速度寄存器低8位值
	mpu6050_data->accZ = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_TEMP_OUT_H);//读取温度寄存器高8位值
	dataL = mpu6050_Read_Register(MPU6050_RA_TEMP_OUT_L);//读取温度寄存器低8位值
	mpu6050_data->temp = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_GYRO_XOUT_H);//读取角速度寄存器高8位值
	dataL = mpu6050_Read_Register(MPU6050_RA_GYRO_XOUT_L);//读取角速度寄存器低8位值
	mpu6050_data->gyroX = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_GYRO_YOUT_H);//读取角速度寄存器高8位值
	dataL = mpu6050_Read_Register(MPU6050_RA_GYRO_YOUT_L);//读取角速度寄存器低8位值
	mpu6050_data->gyroY = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_GYRO_ZOUT_H);//读取角速度寄存器高8位值
	dataL = mpu6050_Read_Register (MPU6050_RA_GYRO_ZOUT_L);//读取角速度寄存器低8位值
	mpu6050_data->gyroZ = (dataH << 8) | dataL;

}
/**

  * @brief    指定mpu6050的寄存器写数据
  * @param    registerAddress: 寄存器地址
  * @param    data           : 待写数据
  * @retval    
  */
void mpu6050_Write_Register(u8 registerAddress, u8 data){
	IIC3_Write_Byte(MPU6050_ADDRESS, registerAddress, data);
}

u8 mpu6050_Read_Register(u8 registerAddress){
	return IIC3_Read_Byte(MPU6050_ADDRESS, registerAddress);
}
