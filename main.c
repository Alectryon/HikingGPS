
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
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "GPSData.h"
#include "ST7735.h"
#include "fatfs/ff.h"
#include "fatfs/diskio.h"

int completeNMEA;
char GPSNMEA[128];
int index;
int temp_int;
char ch;
uint32_t led_state;

//*****************************************************************************
//
// Defines the size of the buffers that hold the path, or temporary data from
// the SD card.  There are two buffers allocated of this size.  The buffer size
// must be large enough to hold the longest expected full path name, including
// the file name, and a trailing null character.
//
//*****************************************************************************
#define PATH_BUF_SIZE   80

//*****************************************************************************
//
// Defines the size of the buffer that holds the command line.
//
//*****************************************************************************
#define CMD_BUF_SIZE    64

//*****************************************************************************
//
// The following are data structures used by FatFs.
//
//*****************************************************************************
static FATFS g_sFatFs;
static DIR g_sDirObject;
static FILINFO g_sFileInfo;
static FIL g_sFileObject;

static uint32_t ui32BytesRead;
static char g_pcTmpBuf[256];
static char g_pcCwdBuf[PATH_BUF_SIZE] = "/";

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

    disk_timerproc();
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

void ConfigureTimer1(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Enable Timer 1 Clock

    ROM_SysCtlDelay(10);
    ROM_IntMasterEnable(); // Enable Interrupts
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // Configure Timer Operation as Periodic

    // Configure Timer Frequency for LED
    // Frequency is given by MasterClock / CustomValue (40MHz/10MHz)
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() / 100);

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

//
// Drawing functions
//

FRESULT drawBMP(const int drawX0, const int drawY0, const int drawX1, const int drawY1, const int x, const int y, const char *filename)
{
    FRESULT iFResult;
    uint16_t row, col, tempi;

    int drawWidth = drawX1 - drawX0;
    int drawHeight = drawY1 - drawY0;

    iFResult = f_open(&g_sFileObject, filename, FA_READ);
    if(iFResult != FR_OK)
        return iFResult;

    int fileIndex = 70;
    iFResult = f_read(&g_sFileObject, g_pcTmpBuf, fileIndex,
                                          (UINT *)&ui32BytesRead);

    char convert[16];
    int width = g_pcTmpBuf[18] | (((uint16_t)g_pcTmpBuf[19]) << 8);
    //int height = g_pcTmpBuf[22] | (((uint16_t)g_pcTmpBuf[23]) << 8);

    itoa(width, convert, 6);
    UARTSend("\r\nWidth: ", 9);
    UARTSend(convert, strlen(convert));

    int32_t offset=(int32_t)(width)*y*2+x*2 + fileIndex;

    iFResult = f_lseek(&g_sFileObject, offset);
    fileIndex = offset;



    setOrientation(ORIENTATION_MY);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    setAddrWindow(drawX0, drawY0, drawX1, drawY1+drawHeight);
    writeCmd(RAMWR);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    for (row = 0; row <= drawHeight; ++row) {
        ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        SELECT_SD();
        offset = (width-drawWidth)*2;    // Offset to the next row

        iFResult = f_read(&g_sFileObject, g_pcTmpBuf, sizeof(g_pcTmpBuf),
                                          (UINT *)&ui32BytesRead);

        fileIndex += offset + drawWidth * 2;
        iFResult = f_lseek(&g_sFileObject, fileIndex);

        DESELECT_SD();

        ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
        for (col = 0; col <= drawWidth; ++col) {
            tempi = col << 1;    // Multiply by 2

            ROM_SSIDataPutNonBlocking(SSI0_BASE, g_pcTmpBuf[tempi+1]);
            ROM_SSIDataGetNonBlocking(SSI0_BASE, 0);
            ROM_SSIDataPutNonBlocking(SSI0_BASE, g_pcTmpBuf[tempi]);
            ROM_SSIDataGetNonBlocking(SSI0_BASE, 0);
        }
    }

    return iFResult;
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
    ConfigureTimer1();
    ConfigureLCD();

    struct GPSData gpsdata;
    char convert[16];

    // SD Init
    // Mount the file system, using logical disk 0.
    FRESULT iFResult;
    iFResult = f_mount(0, &g_sFatFs);
    if(iFResult != FR_OK)
    {
        //UARTprintf("f_mount error: %s\n", StringFromFResult(iFResult));
        UARTSend("\r\nError Mounting\r\n", 18);
        return(1);
    } else {
        UARTSend("\r\nSuccess Mounting\r\n", 20);
    }

    ROM_SysCtlDelay(1000);


    uint32_t ui32TotalSize, ui32ItemCount, ui32FileCount, ui32DirCount;
    ui32TotalSize = 0;
    ui32FileCount = 0;
    ui32DirCount = 0;
    ui32ItemCount = 0;
    char *pcFileName;

#if _USE_LFN
    char pucLfn[_MAX_LFN + 1];
    g_sFileInfo.lfname = pucLfn;
    g_sFileInfo.lfsize = sizeof(pucLfn);
#endif

    // Open the current directory for access.
    iFResult = f_opendir(&g_sDirObject, g_pcCwdBuf);

    // Check for error and return if there is a problem.
    if(iFResult != FR_OK)
    {
        // Ensure that the error is reported.
        UARTSend("\r\nError from SD Card:", 21);
        char temp[2];
        itoa(iFResult, temp, 2);
        UARTSend(temp, 2);
        //return(iFResult);
    } else {
        UARTSend("\r\nSuccess Opening Dir:\r\n", 24);
    }

    // Enter loop to enumerate through all directory entries.
    //
    for(;;)
    {
        // Read an entry from the directory.
        iFResult = f_readdir(&g_sDirObject, &g_sFileInfo);

        // Check for error and return if there is a problem.
        if(iFResult != FR_OK) return((int)iFResult);

        // If the file name is blank, then this is the end of the listing.
        if(!g_sFileInfo.fname[0]) break;

#if _USE_LFN
        pcFileName = ((*g_sFileInfo.lfname)?g_sFileInfo.lfname:g_sFileInfo.fname);
#else
        pcFileName = g_sFileInfo.fname;
#endif

        // Print the entry information on a single line with formatting to show
        // the attributes, date, time, size, and name.
        UARTSend(pcFileName, strlen(pcFileName));
        UARTSend("\r\n", 2);

        // If the attribute is directory, then increment the directory count.
        if(g_sFileInfo.fattrib & AM_DIR) ui32DirCount++;
        // Otherwise, it is a file.  Increment the file count, and
        // add in the file size to the total.
        else
        {
            ui32FileCount++;
            ui32TotalSize += g_sFileInfo.fsize;
        }

        // Move to the next entry in the item array we use to populate the list box.
        ui32ItemCount++;
    }   // endfor

    // Deselect the SD Card
    DESELECT_SD();

    initDisplay();

    fillRect(0, 0, LCDMAXWIDTH, LCDMAXHEIGHT, YELLOW);
    drawString("Hello", 50, 50, RED);

    SELECT_SD();


    // Open the file for reading.
    iFResult = f_open(&g_sFileObject, "Sandia.trail", FA_READ);

    // If there was some problem opening the file, then return an error.
    if(iFResult != FR_OK)
    {
        // Ensure that the error is reported.
        UARTSend("\r\nError from SD Card:", 21);
        char temp[2];
        itoa(iFResult, temp, 2);
        UARTSend(temp, 2);
    }

    iFResult = f_read(&g_sFileObject, g_pcTmpBuf, sizeof(g_pcTmpBuf) - 1,
                              (UINT *)&ui32BytesRead);

    float x0, y0, scaleX, scaleY;
    uint32_t temp = g_pcTmpBuf[0] | g_pcTmpBuf[1] << 8
        | g_pcTmpBuf[2] << 16 | g_pcTmpBuf[3] << 24;
    memcpy(&x0, &temp, sizeof(x0));
    temp = g_pcTmpBuf[4] | g_pcTmpBuf[5] << 8
        | g_pcTmpBuf[6] << 16 | g_pcTmpBuf[7] << 24;
    memcpy(&y0, &temp, sizeof(y0));
    temp = g_pcTmpBuf[8] | g_pcTmpBuf[9] << 8
        | g_pcTmpBuf[10] << 16 | g_pcTmpBuf[11] << 24;
    memcpy(&scaleX, &temp, sizeof(scaleX));
    temp = g_pcTmpBuf[12] | g_pcTmpBuf[13] << 8
        | g_pcTmpBuf[14] << 16 | g_pcTmpBuf[15] << 24;
    memcpy(&scaleY, &temp, sizeof(scaleY));

    itof(x0, convert);
    UARTSend("\r\nX0: ", 6);
    UARTSend(convert, strlen(convert));

    // Now lets get the pixel data
    // Organized bottom to top, left to right
    drawBMP(100, 50, LCDMAXWIDTH, 130, 200, 200, "ABQ.bmp");

    while(1)
    {
        if (completeNMEA == 1) {
            if (parseGPSData(&gpsdata, GPSNMEA) == 1) {
                /*itoa(gpsdata.time, convert, 6);
                UARTSend("\r\nTime: ", 8);
                UARTSend(convert, strlen(convert));

                itof(gpsdata.latitude, convert);
                UARTSend("\r\nLatitude: ", 12);
                UARTSend(convert, strlen(convert));

                itof(gpsdata.longitude, convert);
                UARTSend("\r\nLongitude: ", 13);
                UARTSend(convert, strlen(convert));*/
            }
            completeNMEA = 0;
        }
        ROM_SysCtlDelay(10);
    }
}
