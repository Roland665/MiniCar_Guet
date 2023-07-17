#include "IIC_Hard.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"

/*
声明：这里的7位地址都是高7位，最低位不是地址
*/

//初始化硬件I2C1
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

//初始化硬件I2C2
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

//初始化硬件I2C3
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
  * @brief    I2C指定地址写一字节数据
  * @param    ui32Base          ：IIC基地址
  * @param    target_address    ：从机地址
  * @param    data_address      ：数据目的地址
  * @param    data              ：待写数据
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
  * @brief    I2C指定地址写不定长字节数据
  * @param    ui32Base          ：IIC基地址
  * @param    target_address    ：从机地址
  * @param    data_address      ：数据目的地址
  * @param    len      ：数据长度(字节)
  * @param    data              ：待写数据
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

    //为最后一个finish信号留一个数据
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
  * @brief    I2C指定地址读一字节数据
  * @param    ui32Base          ：IIC基地址
  * @param    target_address    ：从机地址
  * @param    data_address      ：数据源地址
  * @retval   读取到的数据
  */
uint8_t IIC_Read_One_Byte(u32 ui32Base, uint16_t target_address, uint16_t data_address)
{
	//specify that we want to communicate to device address with an intended write to bus
	I2CMasterSlaveAddrSet(ui32Base, target_address, false);

	//put data to be sent into FIFO
	I2CMasterDataPut(ui32Base, data_address);
	
	//send control byte and register address byte to slave device
	I2CMasterControl(ui32Base, I2C_MASTER_CMD_BURST_SEND_START);// 起始信号+从机地址+应答+从机寄存器+应答

	//wait for MCU to complete send transaction
	while(I2CMasterBusy(ui32Base));

	//read from the specified slave device
	I2CMasterSlaveAddrSet(ui32Base, target_address, true);

	//send control byte and read from the register from the MCU
	I2CMasterControl(ui32Base, I2C_MASTER_CMD_SINGLE_RECEIVE);// 起始信号+从机地址+应答+接收数据+停止信号

	//wait while checking for MCU to complete the transaction
	while(I2CMasterBusy(ui32Base));

	//Get the data from the MCU register and return to caller
	return( I2CMasterDataGet(ui32Base));
}


