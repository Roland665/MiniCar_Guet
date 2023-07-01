#include "usart.h"

//÷ÿ”≥…‰printf∫Ø ˝
int fputc(int ch, FILE *f)
{
    UARTCharPut(UART0_BASE, ch);
    return (ch);
} 
int fgetc(FILE *f)
{
    int ch = UARTCharGet(UART0_BASE);
    return (ch);
}



void Usart0_Init(uint32_t bound){
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIOA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, bound, 16000000);

}
