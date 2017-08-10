#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "math.h"
#include "stdio.h"
#include "inc/hw_ssi.h"
#include "driverlib/ssi.h"

#include "ST7735.h"

void ConfigureLCD()
{
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //ROM_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    //ROM_GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    //ROM_GPIOPinConfigure(GPIO_PA4_SSI0RX);
    //ROM_GPIOPinConfigure(GPIO_PA5_SSI0TX);
    //ROM_GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_2);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); // Manual FSS for LCD

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4); //output, LCD D/C

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6); //output, LCD Reset

    //ROM_SSIConfigSetExpClk(SSI0_BASE, ROM_SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,20000000, 8);
    //ROM_SSIEnable(SSI0_BASE);
}

void writeCmd(uint8_t cmd)
{
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0);
    ROM_SSIDataPut(SSI0_BASE, cmd);
    ROM_SSIDataGet(SSI0_BASE, 0);
    while(ROM_SSIBusy(SSI0_BASE));
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_PIN_4);
}

void hardwareReset()
{
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
    ROM_SysCtlDelay(500);
    ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_PIN_6);
    ROM_SysCtlDelay(5000);
}

void initDisplay()
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    hardwareReset();
    writeCmd(SLPOUT);   // Display out of sleep mode
    ROM_SysCtlDelay(150);
    writeCmd(COLMOD);   // Set the color mode
    ROM_SysCtlDelay(150);
    ROM_SSIDataPut(SSI0_BASE, 0x05);      // to mode 5 RGB565 2 bytes
    while(ROM_SSIBusy(SSI0_BASE));
    ROM_SysCtlDelay(150);
    writeCmd(DISPON);   // turn on display*/
}

void write565(uint16_t data, uint32_t count)
{
    writeCmd(RAMWR);
    for (; count > 0; --count)
    {
        ROM_SSIDataPut(SSI0_BASE, (data >> 8));
        SSIDataGet(SSI0_BASE, 0);
        //while(ROM_SSIBusy(SSI0_BASE));
        ROM_SSIDataPut(SSI0_BASE, (data & 0xFF));
        while(ROM_SSIBusy(SSI0_BASE));
        SSIDataGet(SSI0_BASE, 0);
        while(ROM_SSIBusy(SSI0_BASE));
    }
}

void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    writeCmd(CASET);
    ROM_SSIDataPut(SSI0_BASE, 0);
    SSIDataGet(SSI0_BASE, 0);
    ROM_SSIDataPut(SSI0_BASE, x0);
    SSIDataGet(SSI0_BASE, 0);
    ROM_SSIDataPut(SSI0_BASE, 0);
    SSIDataGet(SSI0_BASE, 0);
    ROM_SSIDataPut(SSI0_BASE, x1);
    SSIDataGet(SSI0_BASE, 0);
    writeCmd(RASET);
    ROM_SSIDataPut(SSI0_BASE, 0);
    SSIDataGet(SSI0_BASE, 0);
    ROM_SSIDataPut(SSI0_BASE, y0);
    SSIDataGet(SSI0_BASE, 0);
    ROM_SSIDataPut(SSI0_BASE, 0);
    SSIDataGet(SSI0_BASE, 0);
    ROM_SSIDataPut(SSI0_BASE, y1);
    SSIDataGet(SSI0_BASE, 0);
}

void drawPixel(uint8_t x, uint8_t y, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    setAddrWindow(x, y, x, y);
    write565(color, 1);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void fillRect(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t width = x1-x0+1;
    uint8_t height = y1-y0+1;
    setAddrWindow(x0, y0, x1, y1);
    write565(color, width*height);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void HLine(uint8_t x0, uint8_t x1, uint8_t y, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t width = x1-x0+1;
    setAddrWindow(x0, y, x1, y);
    write565(color, width);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void VLine(uint8_t x, uint8_t y0, uint8_t y1, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t height = y1-y0+1;
    setAddrWindow(x, y0, x, y1);
    write565(color, height);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = (dx>dy ? dx : -dy)/2, e2;
    for(;;) {
        drawPixel(x0, y0, color);
        if (x0==x1 && y0==y1) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 < dy) { err += dx; y0 += sy; }
    }
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void LCD_putCh(char ch, uint8_t x, uint8_t y, uint16_t color, uint16_t bgcolor)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    // Write a character is 6x7
    int pixel;
    uint8_t row, col, bit, data, mask = 0x01;
    setAddrWindow(x, y, x+4, y+6);
    writeCmd(RAMWR);

    for (row = 0; row < 7; ++row)
    {
        for (col = 0; col < 5; ++col)
        {
            data = (FONT_CHARS[ch-32][col]);
            bit = data & mask;
            if (bit == 0) pixel=bgcolor;
            else pixel = color;
            ROM_SSIDataPut(SSI0_BASE, pixel >> 8);
            ROM_SSIDataGet(SSI0_BASE, 0);
            ROM_SSIDataPut(SSI0_BASE, pixel);
            ROM_SSIDataGet(SSI0_BASE, 0);
        }
        mask <<= 1;
    }
    while(ROM_SSIBusy(SSI0_BASE));
}

void drawString(const char *str, uint8_t x, uint8_t y, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t xt = x;
    uint8_t temp = 0;
    while(str[temp] != '\0') {
        LCD_putCh(str[temp], xt, y, color, WHITE);
        xt += 6;
        ++temp;
    }
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void drawStringB(const uint8_t *str, uint8_t bufferSize, uint8_t x, uint8_t y, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t xt = x;
    uint8_t i;
    for (i = 0; i < bufferSize-1; ++i) {
        LCD_putCh((char)str[i], xt, y, color, WHITE);
        xt += 6;
    }
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void drawInt(uint16_t num, uint8_t numdigits, uint8_t x, uint8_t y, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t xt = x + 18;
    char ch;
    while(numdigits-- > 0) {
        ch = num % 10;
        LCD_putCh(ch + 48, xt, y, color, WHITE);
        xt -= 6;
        num /= 10;
    }
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void drawIntEdit(uint8_t num, uint8_t highlight, uint8_t x, uint8_t y, uint16_t color, uint16_t bgcolor)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    int8_t index;
    uint8_t xt = x + 18;
    for (index = 2; index >= 0; --index) {
        if (index == highlight)
            LCD_putCh((num%10) + 48, xt, y, color, bgcolor);
        else
            LCD_putCh((num%10) + 48, xt, y, color, WHITE);
        xt -= 6;
        num /= 10;
    }
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void drawFloat(double f, uint8_t x, uint8_t y, uint16_t color)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t i;
    uint16_t temp = 100;
    uint8_t xt = x;
    if (f < 0) {
        LCD_putCh('-', xt, y, color, WHITE);
        xt += 6;
        f *= -1;
    } else {
        LCD_putCh(' ', xt, y, color, WHITE);
        xt += 6;
    }
    for (i = 0; i < 3; ++i) {
        LCD_putCh(((int)(f / temp) % 10) + 48, xt, y, color, WHITE);
        xt += 6;
        temp /= 10;
    }
    LCD_putCh('.', xt, y, color, WHITE);
    xt += 6;
    temp = 10;
    for (i = 0; i < 4; ++i) {
        LCD_putCh(((uint32_t)(f * temp) % 10) + 48, xt, y, color, WHITE);
        xt += 6;
        temp *= 10;
    }
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

void clearDisp()
{
    fillRect(0, 0, LCDMAXWIDTH, LCDMAXHEIGHT, WHITE);
}

void setOrientation(uint8_t degrees)
{
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    uint8_t arg;
    switch (degrees)
    {
        case ORIENTATION_0: arg = 0x00; break;
        case ORIENTATION_90: arg = 0x60; break;
        case ORIENTATION_180: arg = 0xC0; break;
        case ORIENTATION_270: arg = 0xA0; break;
        case ORIENTATION_MY: arg = 0x80; break;	// BMP mirror Y-axis
        case ORIENTATION_MX: arg = 0x20; break;	// BMP mirror Y-axis
        default: arg = 0x00; break;
    }
    writeCmd(MADCTL);
    ROM_SSIDataPut(SSI0_BASE, arg);
    while(ROM_SSIBusy(SSI0_BASE));
    ROM_SSIDataGet(SSI0_BASE, 0);
    ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}
