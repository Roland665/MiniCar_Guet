#include "IIC_Hard.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"

/*
�����������7λ��ַ���Ǹ�7λ�����λ���ǵ�ַ
*/

//��ʼ��Ӳ��I2C1
void IIC1_Init(void){
	//enable I2C module
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    //reset I2C module
	MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

	//enable GPIO peripheral that contains I2C
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	MAP_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	MAP_GPIOPinConfigure(GPIO_PA7_I2C1SDA);

	// Select the I2C function for these pins.
	MAP_GPIOPinTypeI2CSCL(GPIOA_BASE, GPIO_PIN_6);
	MAP_GPIOPinTypeI2C(GPIOA_BASE, GPIO_PIN_7);

	// Enable and initialize the I2C master module.  
    // Use the system clock for the I2C module.  
    // The data rate is set to 100kbps.
	MAP_I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);

	//clear I2C FIFOs
	HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
}

//��ʼ��Ӳ��I2C2
void IIC2_Init(void){
	//enable I2C module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    //reset I2C module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);

	//enable GPIO peripheral that contains I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	GPIOPinConfigure(GPIO_PE4_I2C2SCL);
	GPIOPinConfigure(GPIO_PE5_I2C2SDA);

	// Select the I2C function for these pins.
	GPIOPinTypeI2CSCL(GPIOE_BASE, GPIO_PIN_4);
	GPIOPinTypeI2C(GPIOE_BASE, GPIO_PIN_5);

	// Enable and initialize the I2C3 master module.  
    // Use the system clock for the I2C3 module.  
    // The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will be set to 400kbps.
	I2CMasterInitExpClk(I2C2_BASE, SysCtlClockGet(), false);

	//clear I2C FIFOs
	HWREG(I2C2_BASE + I2C_O_FIFOCTL) = 80008000;
}

//��ʼ��Ӳ��I2C3
void IIC3_Init(void){
	//enable I2C module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

    //reset I2C module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

	//enable GPIO peripheral that contains I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure the pin muxing for I2C3 functions on port D0 and D1.
	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);

	// Select the I2C function for these pins.
	GPIOPinTypeI2CSCL(GPIOD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIOD_BASE, GPIO_PIN_1);

	// Enable and initialize the I2C3 master module.  
    // Use the system clock for the I2C3 module.  
    // The last parameter sets the I2C data transfer rate.
	// If false the data rate is set to 100kbps and if true the data rate will be set to 400kbps.
	I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);

	//clear I2C FIFOs
	HWREG(I2C3_BASE + I2C_O_FIFOCTL) = 80008000;
}

/** 
  * @brief    I2Cָ����ַдһ�ֽ�����
  * @param    ui32Base          ��IIC����ַ
  * @param    target_address    ���ӻ���ַ
  * @param    data_address      ������Ŀ�ĵ�ַ
  * @param    data              ����д����
  * @retval    
  */
void IIC_Write_One_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address, uint8_t data)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(ui32Base, target_address, false);

	//put data to be sent into FIFO
	I2CMasterDataPut(ui32Base, data_address);

	//Play a start and send device address and register address
	I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_START);

	//wait for MCU to finish transaction
	while(I2CMasterBusy(ui32Base));

	//put data to be sent into FIFO
	I2CMasterDataPut(ui32Base, data);

    //send final byte and play a stops 
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_FINISH);

	//wait for MCU & device to complete transaction
	while(I2CMasterBusy(ui32Base));
}


/** 
  * @brief    I2Cָ����ַд�������ֽ�����
  * @param    ui32Base          ��IIC����ַ
  * @param    target_address    ���ӻ���ַ
  * @param    data_address      ������Ŀ�ĵ�ַ
  * @param    len      �����ݳ���(�ֽ�)
  * @param    data              ����д����
  * @retval    
  */
void IIC_Write_len_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address, u8 len, uint8_t *data)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(ui32Base, target_address, false); 

	//put data to be sent into FIFO
	I2CMasterDataPut(ui32Base, data_address);

	//Play a start and send device address and register address
	I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_START);
    
	//wait for MCU to finish transaction
	while(I2CMasterBusy(ui32Base));

    //Ϊ���һ��finish�ź���һ������
    len--;
    while(len--){
        //put data to be sent into FIFO
        I2CMasterDataPut(ui32Base, *data++);

        //Play a start and send device address and register address
        I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_CONT);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(ui32Base));
    }

	//put data to be sent into FIFO
	I2CMasterDataPut(ui32Base, *data);

    //send final byte and play a stops 
    I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_FINISH);

	//wait for MCU & device to complete transaction
	while(I2CMasterBusy(ui32Base));
}

/**
  * @brief    I2Cָ����ַ��һ�ֽ�����
  * @param    ui32Base          ��IIC����ַ
  * @param    target_address    ���ӻ���ַ
  * @param    data_address      ������Դ��ַ
  * @retval   ��ȡ��������
  */
uint8_t IIC_Read_One_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(ui32Base, target_address, false);

	//put data to be sent into FIFO
	I2CMasterDataPut(ui32Base, data_address);
	
	//send control byte and register address byte to slave device
	I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_START);// ��ʼ�ź�+�ӻ���ַ+Ӧ��+�ӻ��Ĵ���+Ӧ��

	//wait for MCU to complete send transaction
	while(I2CMasterBusy(ui32Base));

	//read from the specified slave device
	I2CMasterSlaveAddrSet(ui32Base, target_address, true);

	//send control byte and read from the register from the MCU
	I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_RECEIVE);// ��ʼ�ź�+�ӻ���ַ+Ӧ��+��������+ֹͣ�ź�

	//wait while checking for MCU to complete the transaction
	while(I2CMasterBusy(ui32Base));

	//Get the data from the MCU register and return to caller
	return( I2CMasterDataGet(ui32Base));
}


