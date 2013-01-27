/******************************************************************************
 *
 * grl_pcd8544_84x48.c - TI Stellaris graphics library display driver for
 *                       PCD8544 controller, used in Nokia 5110/3310 displays
 *
 * Copyright (c) 2013, Joseph Kroesche (kroesche.org)
 * All rights reserved.
 *
 * This software is released under the FreeBSD license, found in the
 * accompanying file LICENSE.txt and at the following URL:
 *      http://www.freebsd.org/copyright/freebsd-license.html
 *
 * This software is provided as-is and without warranty.
 *
 *****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "utils/cmdline.h"
#include "utils/ustdlib.h"
#include "grlib/grlib.h"

#include "grl_pcd8544_84x48.h"

/******************************************************************************
 *
 * This is a display driver for the Philips PCD8544 84 x 48 pixel LCD
 * controller.  This is the controller used in Nokia 5110/3310 displays that
 * can be found at adafruit or sparkfun.  This display driver is meant to be
 * used with the TI StellarisWare graphics library (grlib).
 *
 * Please refer to the StellarisWare graphics library user manual in your
 * StellarisWare distribution (SW-GRL-nnnn.pdf, found in the docs directory).
 *
 * This driver uses a frame buffer so you must call GrFlush() after all
 * primitive operations in order to update the screen.
 *
 * This is a monochrome display so it is 1-bit per pixel and the only grlib
 * color choices are ClrWhite and ClrBlack.  ClrBlack will be interpreted as
 * pixel off, while ClrWhite and any other non-black color will be
 * interpreted as pixel on.
 *
 * TODO:
 *  - add reset line control
 *    right now I assume display reset is tied to Stellaris board reset
 *  - add LED backlight control
 *    right now you can tie LED control signal high or low
 *  - make driver interrupt driven
 *  - add auto flush and dirty rectangle tracking
 *  - bounds checking
 *
 * EXAMPLE:
 *  Here is code showing how you might use this driver in an example:
 *
 *    // init the driver
 *    PCD8544_84x48_Init(SysCtlClockGet());
 *
 *    // init the grlib graphics context
 *    GrContextInit(&context, &g_sNokia);
 *
 *    // define rectangle for screen border
 *    rect.sXMin = 0;
 *    rect.sYMin = 0;
 *    rect.sXMax = GrContextDpyWidthGet(&context) - 1;
 *    rect.sYMax = 23;
 *
 *    // set foreground color (note, white means pixel on)
 *    // draw the rectangle
 *    GrContextForegroundSet(&context, ClrWhite);
 *    GrRectDraw(&context, &rect);
 *
 *    // set the font for text
 *    GrContextFontSet(&context, g_pFontFixed6x8);
 *
 *    // write a string to the screen
 *    GrStringDrawCentered(&context, "Hello", -1, 
 *                         GrContextDpyWidthGet(&context) / 2, 10, 0);
 *
 *    // flush the frame buffer - this will update all prior operations
 *    // to screen
 *    GrFlush(&context);
 *
 *****************************************************************************/

/******************************************************************************
 *
 * Macros
 *
 *****************************************************************************/

/* Define the GPIO ports and pins used for the SSI/SPI peripheral
 */
#define DISPLAY_GPIO_PERIPH     SYSCTL_PERIPH_GPIOB
#define DISPLAY_PINCFG_SSICLK   GPIO_PB4_SSI2CLK
#define DISPLAY_PINCFG_SSIFSS   GPIO_PB5_SSI2FSS
#define DISPLAY_PINCFG_SSITX    GPIO_PB7_SSI2TX
#define DISPLAY_GPIO_PORT       GPIO_PORTB_BASE
#define DISPLAY_GPIO_PINS       (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7)
#define DISPLAY_DC_PIN          GPIO_PIN_6

/* Define the SSI/SPI peripheral to use with the display, data rate
 */
#define DISPLAY_SSI_PERIPH      SYSCTL_PERIPH_SSI2
#define DISPLAY_SSI_BASE        SSI2_BASE
#define DISPLAY_SSI_RATE        1000000

/******************************************************************************
 *
 * Locals
 *
 *****************************************************************************/

/* Create the frame buffer for the display.  We must use a frame buffer
 * because it is necessary to modify individual bits within bytes.  Therefore
 * the existing value of each byte for the display must be stored somewhere
 * and it is not possible to read it from the display.
 */
static uint8_t frameBuffer[6][84];

/* This is the set of commands that should be sent to the display controller
 * when it is initialized.  They are more or less determined experimentally
 * and from reading the data sheet.  The bias is mentioned in the data sheet
 * as having a value of 4 so that should probably be left as is.  However the
 * best contrast value to use seems to be variable and a matter of taste so
 * you may need to adjust this value for your particular display.
 */
static const uint8_t initCommands[] =
{
    0x21,   /* select extended instruction set */
    0x14,   /* set bias */
    0xa8,   /* contrast (Vop) */
    0x20,   /* normal instruction set */
    0x0c,   /* normal display mode */
    0x80,   /* X origin 0 */
    0x40,   /* Y origin 0 */
};

/* This is the set of commands to reset the display origin to 0, 0
 */
static const uint8_t originCommands[] = { 0x40, 0x80 };

/******************************************************************************
 *
 * Write a set of bytes, command or data, to the display controller.
 *
 * *bytes is a pointer to an array of bytes
 * count is the count of bytes to write
 * isCommand is true if it is a command, false if data
 *
 *****************************************************************************/
static void
DisplayWriteBytes(const uint8_t *bytes, unsigned int count, bool isCommand)
{
    unsigned long dcPinValue = isCommand ? 0 : DISPLAY_DC_PIN;
    
    /* Wait for any prior transaction to complete
     */
    while(SSIBusy(DISPLAY_SSI_BASE))
    {}
    
    /* Set the DC pin for command or data
     */
    GPIOPinWrite(DISPLAY_GPIO_PORT, DISPLAY_DC_PIN, dcPinValue);
    
    /* Pause a few cycles for pin setup time
     */
    SysCtlDelay(4);
    
    /* loop until all bytes are written
     */
    while(count--)
    {
        SSIDataPut(DISPLAY_SSI_BASE, *bytes++);
    }
}

/******************************************************************************
 *
 * Translate 24-bit RGB to dispay color value
 *
 * *pvDisplay data is not used
 * ulValue is the 24-bit RGB value
 *
 * The function returns the translated color value.
 *
 * Since this display is monochrome there is really no color other than
 * "off" and "on".  Therefore this function translates any input color other
 * than 0 (black) to a 1 (on) and black as 0 (off).
 *
 * This function is a StellarisWare Graphics Library required driver function.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
static unsigned long
ColorTranslate(void *pvDisplayData, unsigned long ulValue)
{
    return(ulValue ? 1 : 0);
}

/******************************************************************************
 *
 * Flush the frame buffer to the display
 *
 * *pvDisplay data is not used
 *
 * All primitive drawing operations occur in the frame buffer.  This function
 * must be called to flush the frame buffer to the display.
 *
 * This function is a StellarisWare Graphics Library required driver function.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
static void
Flush(void *pvDisplayData)
{
    DisplayWriteBytes(originCommands, sizeof(originCommands), true);
    DisplayWriteBytes(&frameBuffer[0][0], sizeof(frameBuffer), false);
}

/******************************************************************************
 *
 * Draw a horizontal line on the display.
 *
 * *pvDisplay data is not used
 * lX1 is the starting X position
 * lX2 is the ending X position (inclusive)
 * lY is the Y position for the line
 * ulValue is the pre-translated display color to use
 *
 * This function draws a horizontal line into the frame buffer.  GrFlush()
 * must be called in order to update the physical display.
 *
 * This function is a StellarisWare Graphics Library required driver function.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
static void
LineDrawH(void *pvDisplayData, long lX1, long lX2, long lY, unsigned long ulValue)
{
    long tempX;
    
    /* since Y is unchanging, the row and bit value can be computed once
     */
    long row = lY / 8;
    long bit = lY % 8;
    
    /* make sure that the X values are going from smaller to larger
     */
    if(lX1 > lX2)
    {
        tempX = lX2;
        lX2 = lX1;
        lX1 = tempX;
    }
    
    /* iterate over the x values of the line, writing a bit at each position
     */
    for(long col = lX1; col <= lX2; col++)
    {
        HWREGBITB(&frameBuffer[row][col], bit) = ulValue;
    }
}

/******************************************************************************
 *
 * Draw a vertical line on the display.
 *
 * *pvDisplay data is not used
 * lX is the X position to use for the vertical line
 * lY1 is the starting Y position
 * lY2 is the ending Y position (inlusive)
 * ulValue is the pre-translated display color to use
 *
 * This function draws a vertical line into the frame buffer.  GrFlush()
 * must be called in order to update the physical display.
 *
 * This function is a StellarisWare Graphics Library required driver function.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
static void
LineDrawV(void *pvDisplayData, long lX, long lY1, long lY2, unsigned long ulValue)
{
    long tempY;
    long col = lX;
    long bit;
    long row;
    
    /* make sure that the Y values are going from smaller to larger
     */
    if(lY1 > lY2)
    {
        tempY = lY2;
        lY2 = lY1;
        lY1 = tempY;
    }
    
    /* iterate over the Y values of the vertical line
     */
    for(long y = lY1; y <= lY2; y++)
    {
        /* compute the row and bit position for eac pixel
         */
        bit = y % 8;
        row = y / 8;
        
        /* if this bit position is 0 (start of a byte) and there at least
         * 8 more pixels, the this entire byte can be filled with the specified
         * color and skip ahead to the next byte
         */
        if((bit == 0) && ((y + 8) <= lY2))
        {
            frameBuffer[row][col] = ulValue ? 0xff : 0;
            y += 7;
        }
        
        /* otherwise set the individual bit in the byte
         */
        else
        {
            HWREGBITB(&frameBuffer[row][col], bit) = ulValue;
        }
    }
}

/******************************************************************************
 *
 * Set a pixel on the display
 *
 * *pvDisplay data is not used
 * lX is the X position of the pixel
 * lY is the Y position of the pixel
 * ulValue is the pre-translated display color to use
 *
 * This function draws a specific pixel into the frame buffer.  GrFlush()
 * must be called in order to update the physical display.
 *
 * This function is a StellarisWare Graphics Library required driver function.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
static void
PixelDraw(void *pvDisplayData, long lX, long lY, unsigned long ulValue)
{
    /* compute the row and bit number, then set the specific bit in the
     * frame buffer according to the color
     */
    long col = lX;
    long row = lY / 8;
    long bit = lY % 8;
    HWREGBITB(&frameBuffer[row][col], bit) = ulValue;
}

/******************************************************************************
 *
 * Draw a horizontal set of pixels using a palette
 *
 * *pvDisplay data is not used
 * lX is the X position to use to start the draw
 * lY1 is the starting Y position
 * lX0 is the sub-pixel (if needed)
 * ulCount is the count of pixels to draw
 * lBPP is the bits-per-pixel format of the display data (1, 4, or 8)
 * pucData is the a pointer to the display data
 * pucPalette is a color palette to use for looking up color values
 *
 * This function draws a horizontal set of pixel data to the display.  It
 * is not necessarily a line because the pixels may not all have the same
 * color value.  This function is complicated to understand so consult the
 * GRL user manual for more details.
 * This function draws the pixel data into the frame buffer.  GrFlush()
 * must be called in order to update the physical display.
 *
 * This function is a StellarisWare Graphics Library required driver function.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
static void
PixelDrawMultiple(void *pvDisplayData, long lX, long lY, long lX0, long ulCount,
                  long lBPP, const unsigned char *pucData, const unsigned char *pucPalette)
{
    uint8_t color;
    uint32_t paletteIndex;
    
    /* Y is fixed so the row and bit position can be precomputed
     */
    long row = lY / 8;
    long bit = lY % 8;
    
    /* iterate over the pixel data
     */
    for(long col = lX; col < (ulCount + lX); col++)
    {
        /* if the data is one bit per pixel then each bit in the data
         * represents one pixel.  This should almost translate directly to
         * each bit of the display except that the color must be interpreted
         * to be on or off according to the supplied palette
         */
        if(lBPP == 1)
        {
            /* get the index into the palette.  It is going to be 0 or 1.
             * Take into account the subpixel offset lX0 (we might not be
             * at the beginning of the byte
             */
            paletteIndex = *pucData >> (7 - lX0);
            paletteIndex &= 1;
            
            /* now look up the color according to the palette index
             * this will be the pre-translated color needed for this display
             * so no further translation is needed
             */
            color = pucPalette[paletteIndex];
            
            /* reached the end of this byte, so advance to the next
             */
            if(++lX0 == 8)
            {
                pucData++;
                lX0 = 0;
            }
        }
        
        /* if the data is 4 bits per pixel, then each byte represents two
         * pixels (upper and lower nybble)
         */
        else if(lBPP == 4)
        {
            /* look up the palette index for this pixel.  It is a 4-bt value
             * and is either the upper or lower nybble
             */
            paletteIndex = *pucData >> (lX0 ? 0 : 4);
            paletteIndex &= 0x0f;
            
            /* adjust the palette index because each entry in the palette is
             * really 3 bytes, for the 24-bit RGB value
             */
            paletteIndex *= 3;
            
            /* read the 24-bit color value from the palette
             */
            uint32_t paletteEntry = *((uint32_t *)&pucPalette[paletteIndex]);
            paletteEntry &= 0x00ffffff;
            
            /* now the 24-bit RGB color must be translated to something this
             * display understands
             */
            color = ColorTranslate(pvDisplayData, paletteEntry);
            
            /* finally, advance to the next byte if this was the last nybble
             */
            if(++lX0 == 2)
            {
                pucData++;
                lX0 = 0;
            }
        }
        
        /* if the data is 8 bits per pixel, then each byte of the data
         * represents one pixel
         */
        else if(lBPP == 8)
        {
            /* look up the palette index provided by this pixel byte.
             * adjust it by three to get the index into the 24-bit color
             * palette
             */
            paletteIndex = *pucData++;
            paletteIndex *= 3;
            
            /* read the 24-bit color value from the palette
             */
            uint32_t paletteEntry = *((uint32_t *)&pucPalette[paletteIndex]);
            
            /* now the 24-bit RGB color must be translated to something this
             * display understands
             */
            paletteEntry &= 0x00ffffff;
            color = ColorTranslate(pvDisplayData, paletteEntry);
        }
        
        /* unrecognized BPP value so ignore
         */
        else
        {
            return;
        }
        
        /* after all that calculation, now set the value of the bit in the
         * frame buffer
         */
        HWREGBITB(&frameBuffer[row][col], bit) = color;
    }
}

/******************************************************************************
 *
 * Draw a filled rectangle
 *
 * *pvDisplay data is not used
 * pRect is a pointer to a rectangle type defining the rectangle
 * ulValue is the pre-translated display color to use
 *
 * This function draws a rectangle filled with the specified color value.
 * The rectangle is drawn to the frame buffer so GrFlush()
 * must be called in order to update the physical display.
 *
 * This function is a StellarisWare Graphics Library required driver function.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
static void
RectFill(void *pvDisplayData, const tRectangle *pRect, unsigned long ulValue)
{
    for(long col = pRect->sXMin; col <= pRect->sXMax; col++)
    {
        LineDrawV(pvDisplayData, col, pRect->sYMin, pRect->sYMax, ulValue);
    }
    
}

/******************************************************************************
 *
 * Graphics library driver descriptor
 *
 * This structure defines this graphics display and driver to the graphics
 * library.
 *
 * This structure is a StellarisWare Graphics Library required driver structure.
 * See the graphics library documentation for more details.
 *
 *****************************************************************************/
const tDisplay g_sPCD8544_84x48 =
{
    sizeof(tDisplay), 0,
    84, 48,
    PixelDraw, PixelDrawMultiple, LineDrawH, LineDrawV,
    RectFill, ColorTranslate, Flush
};

/******************************************************************************
 *
 * Initialize this graphics driver
 *
 * ulSysClock is the value of the system clock, needed for SPI conifguration
 *
 * This function initializes the hardware needed to control the display
 * driver.  The ulSysClock parameter is the value of the processor system
 * clock which is set by the main application.
 *
 * The application needs to call this function before using any of the
 * display driver.
 *
 *****************************************************************************/
void
GRL_PCD8544_84x48_Init(unsigned long ulSysClock)
{
    /* Enable the GPIO port and the SSI peripheral
     */
    SysCtlPeripheralEnable(DISPLAY_GPIO_PERIPH);
    SysCtlPeripheralEnable(DISPLAY_SSI_PERIPH);
    
    /* set up the GPIO pins used for the SPI peripheral
     */
    GPIOPinConfigure(DISPLAY_PINCFG_SSICLK);
    GPIOPinConfigure(DISPLAY_PINCFG_SSIFSS);
    GPIOPinConfigure(DISPLAY_PINCFG_SSITX);
    GPIOPinTypeSSI(DISPLAY_GPIO_PORT, DISPLAY_GPIO_PINS);
    
    /* Set the DC (data/command) pin to be GPIO
     */
    GPIOPinTypeGPIOOutput(DISPLAY_GPIO_PORT, DISPLAY_DC_PIN);
    
    /* Configure the SSI(SPI) port used to control the display
     */
    SSIDisable(DISPLAY_SSI_BASE);
    SSIConfigSetExpClk(DISPLAY_SSI_BASE, ulSysClock, SSI_FRF_MOTO_MODE_3,
                       SSI_MODE_MASTER, DISPLAY_SSI_RATE, 8);
    SSIEnable(DISPLAY_SSI_BASE);
    
    /* send initialization commands to the display
     */
    DisplayWriteBytes(initCommands, sizeof(initCommands), true);
    
    /* This will write the frame buffer, which should be all 0, to the
     * display.
     */
    Flush(NULL);
}

#ifdef GRL_PCD8544_ENABLE_TEST
/******************************************************************************
 *
 * TEST CODE
 *
 * The following is test code.  The test function can be called from main
 * to exercise the driver.  The test just calls each function in the driver
 * and display something on the screen to verify the driver is working.
 *
 *****************************************************************************/

/* test palette for mapping 1bpp images
 * reverse the bits just to make it interesting
 */
static const uint8_t testPalette1bpp[2] = { 1, 0 };

/* test image using 1bpp
 */
static const uint8_t testFig1[8][2] =
{
    { 0x01, 0x04 },
    { 0x00, 0x88 },
    { 0x01, 0xfc },
    { 0x03, 0x76 },
    { 0x07, 0xff },
    { 0x05, 0xfd },
    { 0x05, 0x05 },
    { 0x00, 0xd8 }
};

/* rgb palette for the remaining test modes. We only map two colors
 * since it is a monochrome display.  For a more thorough test more colors
 * could be mapped to prove that they are all converted to 1bpp correctly
 */
static const uint8_t testPaletteRGB[] =
{
    0x00, 0x00, 0x00, /* black */
    0x01, 0x01, 0x01, /* not-black */
};

/* test image using 4bpp
 */
static const uint8_t testFig2[8][6] =
{
    { 0x00, 0x00, 0x11, 0x11, 0x00, 0x00 },
    { 0x01, 0x11, 0x11, 0x11, 0x11, 0x10 },
    { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 },
    { 0x11, 0x10, 0x01, 0x10, 0x01, 0x11 },
    { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 },
    { 0x00, 0x01, 0x10, 0x01, 0x10, 0x00 },
    { 0x00, 0x11, 0x01, 0x10, 0x11, 0x00 },
    { 0x11, 0x00, 0x00, 0x00, 0x00, 0x11 }
};

/* test image using 8bpp
 */
static const uint8_t testFig3[8][8] =
{
    { 0, 0, 0, 1, 1, 0, 0, 0 },
    { 0, 0, 1, 1, 1, 1, 0, 0 },
    { 0, 1, 1, 1, 1, 1, 1, 0 },
    { 1, 1, 0, 1, 1, 0, 1, 1 },
    { 1, 1, 1, 1, 1, 1, 1, 1 },
    { 0, 0, 1, 0, 0, 1, 0, 0 },
    { 0, 1, 0, 1, 1, 0, 1, 0 },
    { 1, 0, 1, 0, 0, 1, 0, 1 }
};

/* This test function will draw some stuff on the display in order to
 * exercise the driver functions.  It should be called from the application.
 * This function does not call the init function - it must be called from
 * the app before this function is called.
 */
void
GRL_PCD8544_84x48_Test(void)
{
    tRectangle rect;
    const tDisplay *pDisplay = &g_sPCD8544_84x48;
    short width = pDisplay->usWidth;
    short height = pDisplay->usHeight;
    
    /* draw a filled rectangle of the entire screen
     */
    rect.sXMin = 0;
    rect.sYMin = 0;
    rect.sXMax = width - 1;
    rect.sYMax = height - 1;
    RectFill(NULL, &rect, 1);
    
    /* draw a filled rectangle of "off" color just inside screen bounds
     * to create a border
     */
    rect.sXMin += 2;
    rect.sYMin += 2;
    rect.sXMax -= 2;
    rect.sYMax -= 2;
    RectFill(NULL, &rect, 0);
    
    /* draw diagonal upper left to lower right
     */
    for(int y = 0; y < height; y++)
    {
        PixelDraw(NULL, y, y, 1);
    }
    
    /* draw diagonal lower left to upper right
     */
    for(int y = 0; y < height; y++)
    {
        PixelDraw(NULL, (height - 1) - y, y, 1);
    }
    
    /* draw a vertical line
     */
    LineDrawV(NULL, width / 2, (height / 2) - 9, (height / 2) + 9, 1);
    
    /* draw a horizontal line
     */
    LineDrawH(NULL, (width / 2) - 13, (width / 2) + 13, height / 2, 1);
    
    /* draw a rectangle as a background for the next figure since it has
     * inverted palette
     */
    rect.sXMin = 58;
    rect.sYMin = 7;
    rect.sXMax = 72;
    rect.sYMax = 18;
    RectFill(NULL, &rect, 1);
    
    /* draw a figure using multiple with 1bpp
     */
    for(int row = 0; row < 8; row++)
    {
        PixelDrawMultiple(NULL, 60, 9 + row, 5, 11, 1,
                          testFig1[row], testPalette1bpp);
    }
    
    /* draw a figure using multiple with 4bpp
     */
    for(int row = 0; row < 8; row++)
    {
        PixelDrawMultiple(NULL, 60, 20 + row, 0, 12, 4,
                          testFig2[row], testPaletteRGB);
    }

    /* draw a figure using multiple with 8bpp
     */
    for(int row = 0; row < 8; row++)
    {
        PixelDrawMultiple(NULL, 60, 35 + row, 0, 8, 8,
                          testFig3[row], testPaletteRGB);
    }
    
    Flush(NULL);
}
#endif
