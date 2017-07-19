
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
// The following are data structures used by FatFs.
//
//*****************************************************************************
static FATFS g_sFatFs;
static DIR g_sDirObject;
static FILINFO g_sFileInfo;
static FIL g_sFileObject;
static FRESULT iFResult;

static uint32_t ui32BytesRead;
static char g_pcTmpBuf[256];
static char g_pcCwdBuf[PATH_BUF_SIZE] = "/";

//***************************
#define BMPHEADERSIZE   70
// Max 4 images on LCD at once
static uint8_t centerImg[128][128];
static uint16_t color, red, green, blue;
static int32_t offset;
static int16_t row, col, tempi;

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

void shiftImg(int xShift, int yShift)
{
    // Down shift
    if (yShift <= 0) {
        for (row = 0; row < LCDWIDTH; ++row) {
            // Left Shift
            if (xShift <= 0) {
                for (col = 0; col < LCDWIDTH; ++col) {
                    if ((row - yShift > LCDMAXWIDTH) | (col - xShift > LCDMAXWIDTH))
                        centerImg[row][col] = 0;
                    else
                        centerImg[row][col] = centerImg[row - yShift][col - xShift];
                }
            } else {    // Right Shift
                for (col = LCDMAXWIDTH; col >= 0; --col) {
                    if ((col - xShift < 0) | (row - yShift > LCDMAXWIDTH))
                        centerImg[row][col] = 0;
                    else
                        centerImg[row][col] = centerImg[row - yShift][col - xShift];
                }
            }
        }
    }

    // Up shift
    else {
        for (row = LCDMAXWIDTH; row >= 0; --row) {
            // Left Shift
            if (xShift <= 0) {
                for (col = 0; col < LCDWIDTH; ++col) {
                    if ((row - yShift < 0) | (col - xShift > LCDMAXWIDTH))
                        centerImg[row][col] = 0;
                    else
                        centerImg[row][col] = centerImg[row - yShift][col - xShift];
                }
            } else {    // Right Shift
                for (col = LCDMAXWIDTH; col >= 0; --col) {
                    if ((col - xShift < 0) | (row - yShift < 0))
                        centerImg[row][col] = 0;
                    else
                        centerImg[row][col] = centerImg[row - yShift][col - xShift];
                }
            }
        }
    }

    // Left shift
    /*if (xShift < 0) {
        for (row = 0; row < LCDWIDTH; ++row) {
            for (col = 0; col < LCDWIDTH; ++col) {
                if (col - xShift > LCDMAXWIDTH)
                    centerImg[row][col] = 0;
                else
                    centerImg[row][col] = centerImg[row][col - xShift];
            }
        }
    }

    // Right shift
    else if (xShift > 0) {
        for (row = LCDMAXWIDTH; row >= 0; --row) {
            for (col = LCDMAXWIDTH; col >= 0; --col) {
                if (col - xShift < 0)
                    centerImg[row][col] = 0;
                else
                    centerImg[row][col] = centerImg[row][col - xShift];
            }
        }
    }

    // Down shift
    if (yShift < 0) {
        for (row = 0; row < LCDWIDTH; ++row) {
            for (col = 0; col < LCDWIDTH; ++col) {
                if (row - yShift > LCDMAXWIDTH)
                    centerImg[row][col] = 0;
                else
                    centerImg[row][col] = centerImg[row - yShift][col];
            }
        }
    }

    // Up shift
    else if (yShift > 0) {
        for (row = LCDMAXWIDTH; row >= 0; --row) {
            for (col = LCDMAXWIDTH; col >= 0; --col) {
                if (row - yShift < 0)
                    centerImg[row][col] = 0;
                else
                    centerImg[row][col] = centerImg[row - yShift][col];
            }
        }
    }*/
}

FRESULT loadBMP(const int drawX0, const int drawY0, const int drawX1, const int drawY1, const int x, const int y, const char *filename)
{
    int drawWidth = drawX1 - drawX0;
    //int drawHeight = drawY1 - drawY0;

    iFResult = f_open(&g_sFileObject, filename, FA_READ);
    if(iFResult != FR_OK)
        return iFResult;

    uint32_t fileIndex = BMPHEADERSIZE; // Header size is 70
    iFResult = f_read(&g_sFileObject, g_pcTmpBuf, fileIndex,
                                          (UINT *)&ui32BytesRead);

    uint16_t width = g_pcTmpBuf[18] | (((uint16_t)g_pcTmpBuf[19]) << 8);
    //int height = g_pcTmpBuf[22] | (((uint16_t)g_pcTmpBuf[23]) << 8);

    offset=(int32_t)(width)*y*2+x*2 + fileIndex;

    iFResult = f_lseek(&g_sFileObject, offset);
    fileIndex = offset;

    for (row = drawY0; row <= drawY1; ++row) {
        SELECT_SD();
        offset = (width-drawWidth-1)*2;    // Offset to the next row

        iFResult = f_read(&g_sFileObject, g_pcTmpBuf, (drawWidth+1) * 2,
                                          (UINT *)&ui32BytesRead);

        fileIndex += offset + ui32BytesRead;
        iFResult = f_lseek(&g_sFileObject, fileIndex);

        DESELECT_SD();

        for (col = 0; col <= drawWidth; ++col) {
            tempi = col << 1;    // Multiply by 2
            color = g_pcTmpBuf[tempi] | (g_pcTmpBuf[tempi+1] << 8);

            red = (color & 0xF800) >> 9;
            green = (color & 0x07E0) >> 6;
            blue = (color & 0x001F) >> 3;

            centerImg[row][col+drawX0] = (red & 0xE0) | (green & 0x1C) | blue;
        }
    }

    iFResult = f_close(&g_sFileObject);

    return iFResult;
}

void drawBMP(const int drawX0, const int drawY0, const int drawX1, const int drawY1)
{
    setOrientation(ORIENTATION_MY);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    setAddrWindow(drawX0, drawY0, drawX1, drawY1);
    writeCmd(RAMWR);

    for (row = drawY0; row <= drawY1; ++row) {
        for (col = drawX0; col <= drawX1; ++col) {
            color = ((centerImg[row][col] & 0xE0) << 9) |
                    ((centerImg[row][col] & 0x1C) << 6) |
                    ((centerImg[row][col] & 0x03) << 3);

            ROM_SSIDataPutNonBlocking(SSI0_BASE, color >> 8);
            ROM_SSIDataGetNonBlocking(SSI0_BASE, 0);
            ROM_SSIDataPutNonBlocking(SSI0_BASE, color);
            ROM_SSIDataGetNonBlocking(SSI0_BASE, 0);
        }
    }
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

FRESULT drawBMPFromFile2(const int drawX0, const int drawY0, const int drawX1, const int drawY1, const int x, const int y, const char *filename)
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
    setAddrWindow(drawX0, drawY0, drawX1, drawY1);
    writeCmd(RAMWR);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    for (row = 0; row <= drawHeight; ++row) {
        ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        SELECT_SD();
        offset = (width-drawWidth)*2;    // Offset to the next row

        iFResult = f_read(&g_sFileObject, g_pcTmpBuf, (drawWidth+1) * 2,
                                          (UINT *)&ui32BytesRead);

        fileIndex += offset + (drawWidth+1) * 2;
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
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

    iFResult = f_close(&g_sFileObject);

    return iFResult;
}

void drawAlectryonLogo(const uint8_t x, const uint8_t y)
{
    // Alectryon Logo drawn below the main menu
    // Width=40, Height=40
    fillRect(x, y, x+40, y+40, WHITE);
    fillRect(x+10, y+7, x+11, y+13, RED);
    fillRect(x+11, y+5, x+13, y+16, RED);
    fillRect(x+13, y+6, x+14, y+16, RED);
    fillRect(x+14, y+3, x+16, y+17, RED);
    fillRect(x+16, y+4, x+17, y+17, RED);
    fillRect(x+17, y+6, x+18, y+17, RED);
    fillRect(x+18, y+10, x+28, y+39, RED);
    fillRect(x+19, y+8, x+23, y+9, RED);
    fillRect(x+20, y+7, x+22, y+7, RED);
    fillRect(x+21, y+6, x+22, y+6, RED);
    fillRect(x+25, y+7, x+27, y+9, RED);
    fillRect(x+26, y+4, x+27, y+7, RED);
    fillRect(x+27, y+2, x+28, y+4, RED);
    fillRect(x+28, y+9, x+29, y+37, RED);
    fillRect(x+29, y+7, x+30, y+9, RED);
    fillRect(x+30, y+4, x+31, y+6, RED);
    fillRect(x+30, y+10, x+32, y+33, RED);
    fillRect(x+32, y+8, x+33, y+10, RED);
    fillRect(x+33, y+12, x+34, y+24, RED);
    fillRect(x+34, y+12, x+35, y+19, RED);
    fillRect(x+36, y+16, x+36, y+17, RED);
    fillRect(x+35, y+22, x+35, y+25, RED);
    fillRect(x+35, y+22, x+35, y+25, RED);
    fillRect(x+36, y+24, x+36, y+25, RED);
    fillRect(x+33, y+26, x+33, y+31, RED);
    fillRect(x+30, y+34, x+30, y+37, RED);
    fillRect(x+31, y+34, x+31, y+35, RED);
    fillRect(x+0, y+36, x+17, y+39, RED);
    fillRect(x+1, y+34, x+17, y+35, RED);
    fillRect(x+2, y+32, x+17, y+33, RED);
    fillRect(x+2, y+32, x+17, y+33, RED);
    fillRect(x+3, y+31, x+17, y+31, RED);
    fillRect(x+4, y+30, x+17, y+30, RED);
    fillRect(x+4, y+28, x+17, y+29, RED);
    fillRect(x+6, y+27, x+17, y+27, RED);
    fillRect(x+7, y+26, x+17, y+26, RED);
    fillRect(x+8, y+25, x+17, y+25, RED);
    fillRect(x+9, y+24, x+17, y+24, RED);
    fillRect(x+10, y+23, x+17, y+23, RED);
    fillRect(x+11, y+22, x+17, y+22, RED);
    fillRect(x+12, y+21, x+17, y+21, RED);
    fillRect(x+14, y+20, x+17, y+20, RED);
    fillRect(x+15, y+19, x+17, y+19, RED);
    fillRect(x+17, y+18, x+17, y+18, RED);
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
    //char convert[16];

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

    //ROM_SysCtlDelay(1000);

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
    fillRect(0, 0, LCDMAXWIDTH, LCDMAXHEIGHT, WHITE);
    drawAlectryonLogo(44, 0);

    SELECT_SD();


    // Open the file for reading.
    /*iFResult = f_open(&g_sFileObject, "Sandia.trail", FA_READ);

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
    UARTSend(convert, strlen(convert));*/

    // Check if the same image or

    loadBMP(0, 0, LCDMAXWIDTH, LCDMAXWIDTH, 0, 0, "ABQ.bmp");
    /*loadBMP(64, 64, LCDMAXWIDTH, LCDMAXWIDTH, 64, 64, "ABQ.bmp");
    loadBMP(0, 64, LCDWIDTH/2-1, LCDMAXWIDTH, 0, 64, "james.bmp");
    loadBMP(64, 0, LCDMAXWIDTH, LCDWIDTH/2-1, 64, 0, "james.bmp");*/

    //shiftImg(25, 40);

    drawBMP(0, 0, LCDMAXWIDTH, LCDMAXWIDTH);

    /*int deltaX = 0;
    int deltaY = 0;
    int x = 0;
    int y = 0;*/

    while(1)
    {
        if (completeNMEA == 1) {
            if (parseGPSData(&gpsdata, GPSNMEA) == 1) {

                setOrientation(0);
                if (gpsdata.status == GPS_NOFIX)
                    drawString("No Fix  ", 5, 2, RED);
                else if (gpsdata.status == GPS_GPSFIX)
                    drawString("GPS Fix ", 5, 2, BLUE);
                else if (gpsdata.status == GPS_DIFF)
                    drawString("Diff GPS", 5, 2, BLUE);
                drawString("# Sats:", 70, 2, BLACK);
                drawInt(gpsdata.numsats, 2, 100, 2, BLUE);
                drawString("Time:", 5, 12, BLACK);
                // Draw time in format
                uint8_t hour = gpsdata.time/10000;
                uint8_t min = (gpsdata.time-10000*hour)/100;
                uint8_t sec = (gpsdata.time-10000*hour-100*min);
                drawString("  :  :", 38, 12, BLACK);
                drawInt(hour, 2, 26, 12, BLUE);
                drawInt(min, 2, 44, 12, BLUE);
                drawInt(sec, 2, 62, 12, BLUE);
                drawString("Altitude:", 5, 22, BLACK);
                drawInt(gpsdata.altitude, 5, 65, 22, BLUE);
            }
            completeNMEA = 0;
        }

        /*shiftImg(deltaX, deltaY);

        // load the missing parts
        x -= deltaX;
        y -= deltaY;
        loadBMP(0, LCDMAXWIDTH, LCDMAXWIDTH, LCDMAXWIDTH, x, y + LCDMAXWIDTH, "ABQ.bmp");
        loadBMP(LCDMAXWIDTH, 0, LCDMAXWIDTH, LCDMAXWIDTH, x+LCDMAXWIDTH, y, "ABQ.bmp");

        drawBMP(0, 0, LCDMAXWIDTH, LCDMAXWIDTH);*/


        ROM_SysCtlDelay(10);
    }
}
