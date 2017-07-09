
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "GPSData.h"

int completeNMEA;
char GPSNMEA[128];
int index;
int temp_int;
char ch;
uint32_t led_state;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// Send a string to the UART.
void UARTSend(const char *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
        while(!ROM_UARTSpaceAvail(UART0_BASE));
    }
}

void UARTIntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

    // Clear the asserted interrupts.
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(ROM_UARTCharsAvail(UART1_BASE))
    {
        uint32_t ch = ROM_UARTCharGetNonBlocking(UART1_BASE);

        // Read the next character from the UART and write it back to the UART.
        //ROM_UARTCharPutNonBlocking(UART0_BASE, ch);

        GPSBuffer[GPSBufferIndex++] = ch;

        if (ch == '\n') {
            completeNMEA = 1;
            GPSBuffer[GPSBufferIndex++] = '\0';
            GPSBufferIndex = 0;
            strcpy(GPSNMEA, GPSBuffer);
        }

        if (GPSBufferIndex >= 128) {
            GPSBuffer[0] = '\0';
            GPSBufferIndex = 0;
        }
    }
}

void Timer1AHandler(void){
    //Required to launch next interrupt
    ROM_TimerIntClear(TIMER1_BASE, TIMER_A);

    led_state ^= GPIO_PIN_2;
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, led_state);
    //state ^= 0x02; // Toggle led state
    //digitalWrite(LED, state); // Blink
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART0.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));

    // Enable the GPIO Peripheral used by the UART0.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
    ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));

    // Enable interrupts for UART1 receiving
    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
}

void ConfigureTimer1A(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock
    ROM_IntMasterEnable(); // Enable Interrupts
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic

    // Configure Timer Frequency
    // Frequency is given by MasterClock / CustomValue (40MHz/10MHz)
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 10000000);

    ROM_IntEnable(INT_TIMER1A);  // Enable Timer 1A Interrupt
    ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // Timer 1A Interrupt when Timeout
    ROM_TimerEnable(TIMER1_BASE, TIMER_A); // Start Timer 1A
}

void itoa(int num, char *buffer, int digits)
{
    for (index = digits-1; index >= 0; --index) {
        ch = num % 10;
        buffer[index] = ch + 48;
        num /= 10;
    }
    buffer[digits] = '\0';
}

void itof(double f, char *buffer)
{
    temp_int = 100;
    if (f < 0) {
        buffer[0] = '-';
        f *= -1;
    } else {
        buffer[0] = '+';
    }
    for (index = 1; index < 4; ++index) {
        buffer[index] = ((uint32_t)(f / temp_int) % 10) + 48;
        temp_int /= 10;
    }
    buffer[index++] = '.';
    temp_int = 10;
    for (; index < 11; ++index) {
        buffer[index] = ((uint32_t)(f * temp_int) % 10) + 48;
        temp_int *= 10;
    }
    buffer[index] = '\0';
}

int main(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Enable processor interrupts
    ROM_IntMasterEnable();

    // Enable the GPIO port that is used for the on-board LED.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LED (PF2 & PF3).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    // Initialize the UART.
    ConfigureUART();
    ConfigureTimer1A();

    struct GPSData gpsdata;
    char convert[16];

    while(1)
    {
        //if (GPSBuffer[GPSBufferIndex] == '\n') {
        if (completeNMEA == 1) {
            if (parseGPSData(&gpsdata, GPSNMEA) == 1) {
                itoa(gpsdata.time, convert, 6);
                UARTSend("\r\nTime: ", 8);
                UARTSend(convert, strlen(convert));

                itof(gpsdata.latitude, convert);
                UARTSend("\r\nLatitude: ", 12);
                UARTSend(convert, strlen(convert));

                itof(gpsdata.longitude, convert);
                UARTSend("\r\nLongitude: ", 13);
                UARTSend(convert, strlen(convert));
            }
            completeNMEA = 0;
        }
        ROM_SysCtlDelay(10);
    }
}
