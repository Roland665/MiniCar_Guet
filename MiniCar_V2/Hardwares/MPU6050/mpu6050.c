#include "mpu6050.h"
#include "IIC/IIC_Hard.h"

void mpu6050_Init(void){
	IIC3_Init();
	mpu6050_Write_Register(MPU6050_RA_PWR_MGMT_1,0x01);//����λ�����˯�ߡ�����Ҫѭ���������¶ȴ�������ѡ��������ʱ��
	mpu6050_Write_Register(MPU6050_RA_PWR_MGMT_2,0x00);//ѭ��ģʽ����Ƶ��Ϊ0��ÿ��Ĵ���λ��Ϊ0��������
	mpu6050_Write_Register(MPU6050_RA_SMPLRT_DIV,0x09);//ʱ��ʮ��Ƶ
	mpu6050_Write_Register(MPU6050_RA_CONFIG,0x06);//����Ҫ�ⲿͬ�������ֵ�ͨ�˲�����110(˵����ƽ�����˲�)
	mpu6050_Write_Register(MPU6050_RA_GYRO_CONFIG,0x18);//�����ǣ����Բ⡢����������ѡ���2000��/s
	mpu6050_Write_Register(MPU6050_RA_ACCEL_CONFIG,0x18);//���ٶȼƣ����Բ⡢����ѡ���16�����ø�ͨ�˲���
}

void mpu6050_Get_Data(mpu6050_t* mpu6050_data){
	u8 dataH, dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_ACCEL_XOUT_H);//��ȡ���ٶȼĴ�����8λֵ
	dataL = mpu6050_Read_Register(MPU6050_RA_ACCEL_XOUT_L);//��ȡ���ٶȼĴ�����8λֵ
	mpu6050_data->accX = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_ACCEL_YOUT_H);//��ȡ���ٶȼĴ�����8λֵ
	dataL = mpu6050_Read_Register(MPU6050_RA_ACCEL_YOUT_L);//��ȡ���ٶȼĴ�����8λֵ
	mpu6050_data->accY = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_ACCEL_ZOUT_H);//��ȡ���ٶȼĴ�����8λֵ
	dataL = mpu6050_Read_Register(MPU6050_RA_ACCEL_ZOUT_L);//��ȡ���ٶȼĴ�����8λֵ
	mpu6050_data->accZ = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_TEMP_OUT_H);//��ȡ�¶ȼĴ�����8λֵ
	dataL = mpu6050_Read_Register(MPU6050_RA_TEMP_OUT_L);//��ȡ�¶ȼĴ�����8λֵ
	mpu6050_data->temp = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_GYRO_XOUT_H);//��ȡ���ٶȼĴ�����8λֵ
	dataL = mpu6050_Read_Register(MPU6050_RA_GYRO_XOUT_L);//��ȡ���ٶȼĴ�����8λֵ
	mpu6050_data->gyroX = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_GYRO_YOUT_H);//��ȡ���ٶȼĴ�����8λֵ
	dataL = mpu6050_Read_Register(MPU6050_RA_GYRO_YOUT_L);//��ȡ���ٶȼĴ�����8λֵ
	mpu6050_data->gyroY = (dataH << 8) | dataL;

	dataH = mpu6050_Read_Register(MPU6050_RA_GYRO_ZOUT_H);//��ȡ���ٶȼĴ�����8λֵ
	dataL = mpu6050_Read_Register (MPU6050_RA_GYRO_ZOUT_L);//��ȡ���ٶȼĴ�����8λֵ
	mpu6050_data->gyroZ = (dataH << 8) | dataL;

}
/**

  * @brief    ָ��mpu6050�ļĴ���д����
  * @param    registerAddress: �Ĵ�����ַ
  * @param    data           : ��д����
  * @retval    
  */
void mpu6050_Write_Register(u8 registerAddress, u8 data){
	IIC3_Write_Byte(MPU6050_ADDRESS, registerAddress, data);
}

u8 mpu6050_Read_Register(u8 registerAddress){
	return IIC3_Read_Byte(MPU6050_ADDRESS, registerAddress);
}
