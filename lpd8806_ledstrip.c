/******************************************************************************
 *
 * lpd8806_ledstrip.c - Stellaris/Tiva driver for RGB LED strip using LPD8806
 *                      controller
 *
 * Copyright (c) 2014, Joseph Kroesche (kroesche.org)
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
#include <string.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/udma.h"
#include "inc/hw_ssi.h"

#include "lpd8806_ledstrip.h"

/**
 * @addtogroup lpd8806_ledstrip_driver Driver for LPD8806 RGB LED Strips
 *
 * This is a Stellaris/Tiva driver intended to be used with RGB LED lighting
 * strip based on the LPD8806 controller.  These lighting strips can be found on
 * adafruit and other places.  They are typically 32 LEDs in a strip but can
 * be longer and can be joined in series.
 *
 * This has been tested using an Stellaris EK-LM3S6965, but should work
 * with any Stellaris or Tiva part with little or no modifications.
 *
 * The LED strip must be powered appropriately and connected to the data and
 * clock lines of one of the Stellaris SSI/SPI ports.  This driver is attached
 * to a SPI peripheral when it is initialized.
 *
 * This driver maintains a pixel "frame buffer".  The provided functions allow
 * to set and get the RGB value of any pixel in the buffer and to update the
 * entire string at once.
 *
 * Each pixel is 7-bit * 3 channels (total of 21 bits) RGB.  You can manipulate
 * the pixel values by individual color channel or as a single value.
 *
 * @todo make driver interrupt driven (presently is polled or DMA)
 *
 * EXAMPLE:
 *  Here is code showing how you might use this driver in an example:
 *
 * ~~~~~~~~.c
 *    // Allocate a frame buffer for a 32 pixel strip
 *    #define NUM_PIXELS 32
 *    uint8_t frameBuffer[LPD8806_LEDSTRIP_FRAMEBUF_SIZE(NUM_PIXELS)];
 *
 *    // init the driver for SPI 0, and 500 kHz clock rate
 *    LPD8806_LedStrip_Handle_t hLed = LPD8806_LedStrip_Init(SysCtlClockGet(), 0, 500000);
 *    if(!hLed)
 *    {
 *        // handle error
 *        ...
 *    }
 *
 *    // Assign the frame buffer
 *    LPD8806_LedStrip_SetFrameBuffer(hLed, frameBuffer, NUM_PIXELS);
 *
 *    // Clear out the pixel data
 *    LPD8806_LedStrip_Clear(hLed);
 *
 *    // Update the strip so everything will be turned off
 *    LPD8806_LedStrip_Update(hLed);
 *
 *    // set pixel 0 to full red, half green, quarter blue
 *    LPD8806_LedStrip_SetPixelRgb(hLed, 0, 0x7f, 0x40, 0x20);
 *
 *    // update the light string
 *    LPD8806_LedStrip_Update(hLed);
 * ~~~~~~~~
 *
 * @{
 */

/******************************************************************************
 *
 * Macros
 *
 *****************************************************************************/

// set internal macro to indicate if DMA support is compiled in
#ifdef LPD8806_LEDSTRIP_ALLOW_DMA
#define LPD8806_DMA_SUPPORT 1
#else
#define LPD8806_DMA_SUPPORT 0
#endif

/******************************************************************************
 *
 * Locals
 *
 *****************************************************************************/

// data type to represent a single rgb led (pixel) in the string
typedef union __attribute__((__packed__))
{
    uint8_t bytes[3];
    struct __attribute__((__packed__))
    {
        uint8_t g;
        uint8_t r;
        uint8_t b;
    } rgb;
} pixel_t;

// Structure that holds a single instance of an "LedStrip".  It contains
// all variable information about the LedStrip configuration so that
// the driver can support multiple instances.
typedef struct
{
    uint32_t spiPeriph;     // SPI peripheral for PeriphEnable
    uint32_t spiBase;       // SPI peripheral base address
    uint32_t spiRate;       // SPI data rate
    pixel_t *pFrameBuffer;  // Pointer to buffer for pixel data
    uint32_t numPixels;     // number of pixels in the frame buffer
    uint32_t latchBytes;    // number of latch bytes needed for strip
    uint32_t dmaChannel;    // DMA channel to use, if any
    bool useDma;            // flag to indicate if DMA is used
} Instance_t;

// Storage for multiple LED instances.  When _Init() is called, an
// instance is allocated from here.  It uses a simple scheme to mark
// allocations - non-zero if spiPeriph
static Instance_t instances[LPD8806_LEDSTRIP_NUM_INSTANCES];

// A structure holding hardware information for a SPI peripheral
typedef struct
{
    uint32_t periph;
    uint32_t base;
} Spi_t;

// A table to map SPI index numbers to underlying peripherals
static const Spi_t spiPeriphTable[] =
{
    { SYSCTL_PERIPH_SSI0, SSI0_BASE },
    { SYSCTL_PERIPH_SSI1, SSI1_BASE },
};
#define NUM_SPI_PERIPHS (sizeof(spiPeriphTable) / sizeof(Spi_t))

/******************************************************************************
 *
 * Private API
 *
 *****************************************************************************/

/*
 * Validate an instance pointer.
 *
 * @param pInst is the instance pointer to check
 *
 * This function will perform basic validity checks on the instance
 * pointer.
 *
 * @returns Returns **true** if the instance data is valid, or **false**
 *          if not.
 */
static bool
InstanceIsValid(Instance_t *pInst)
{
    if(!pInst)
    {
        return false;
    }
    if(!pInst->pFrameBuffer || (pInst->numPixels == 0))
    {
        return false;
    }
    if(pInst->spiPeriph == 0)
    {
        return false;
    }
    
    return true;
}

/******************************************************************************
 *
 * Public API
 *
 *****************************************************************************/

/**
 *
 * Set the RGB value of a pixel directly
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param pixelNum is the pixel number to modify
 * @param rgb is the 24-bit RGB value ([23:16]red, [15:8]green, [7:0]blue)
 *
 * This function is used to set the color value of a pixel using a
 * combined RGB value in a uint32_t.  The RGB color channel values are
 * 7 bits each and arranged as follows:
 *
 *  Bits | Color Channel
 * ------|--------------
 * 31:24 | Not used
 *    23 | Reserved (set to 0)
 * 22:16 | Red
 *    15 | Reserved (set to 0)
 * 14:8  | Green
 *    7  | Reserved (set to 0)
 *  6:0  | Blue
 *
 */
void
LPD8806_LedStrip_SetPixelValue(LPD8806_LedStrip_Handle_t h,
                               uint32_t pixelNum, uint32_t rgb)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return;
    }

    // check that pixel is within bounds
    if(pixelNum >= pInst->numPixels)
    {
        return;
    }
    
    // Get frame buffer from instance
    pixel_t *pfb = pInst->pFrameBuffer;
    
    // set the individual channels based on the input value
    // the upper bit of each color channel must be set for the controller
    pfb[pixelNum].bytes[0] = ((rgb >> 8) & 0xff) | 0x80;
    pfb[pixelNum].bytes[1] = ((rgb >> 16) & 0xff) | 0x80;
    pfb[pixelNum].bytes[2] = ((rgb >> 0) & 0xff) | 0x80;
}

/**
 *
 * Get the pixel color as a single RGB value
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param pixelNum is the pixel number to read
 *
 * The value is returned as a 24-bit RGB value with color channels
 * arranged as: ([23:16]red, [15:8]green, [7:0]blue)
 *
 * @returns Returns the RGB color value of the pixel.  If there is any
 *          error then 0 is returned.
 */
uint32_t
LPD8806_LedStrip_GetPixelValue(LPD8806_LedStrip_Handle_t h, uint32_t pixelNum)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return 0x80000000;
    }

    // check that pixel is within bounds
    if(pixelNum >= pInst->numPixels)
    {
        return 0x80000000;
    }

    // Get a pointer to the specific pixel
    pixel_t *px = &pInst->pFrameBuffer[pixelNum];
    
    // read each color channel and combine into a 24-bit value.  Mask off
    // the upper bit of each byte since it is used by the controller and
    // is not part of the color value
    uint32_t val = ((uint32_t)px->rgb.r << 16)
                 | ((uint32_t)px->rgb.g << 8)
                 | (uint32_t)px->rgb.b;
    val &= 0x7f7f7f;

    return val;
}

/**
 *
 * Set the pixel color using color channels
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param pixelNum is the pixel number to modify
 * @param r is the 7-bit value of the *red* color channel
 * @param g is the 7-bit value of the *green* color channel
 * @param b is the 7-bit value of the *blue* color channel
 *
 * This function is used to set the color of a specific pixel by specifying
 * the individual color channel values.  Each color channel is 7 bits.
 */
void
LPD8806_LedStrip_SetPixelRgb(LPD8806_LedStrip_Handle_t h, uint32_t pixelNum,
                             uint8_t r, uint8_t g, uint8_t b)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return;
    }
    
    // Get frame buffer
    pixel_t *pfb = pInst->pFrameBuffer;

    // check that pixel is within bounds
    if(pixelNum >= pInst->numPixels)
    {
        return;
    }
    
    // set each color in the pixel
    // the upper bit of each color channel must be set for the controller
    pfb[pixelNum].rgb.r = r | 0x80;
    pfb[pixelNum].rgb.g = g | 0x80;
    pfb[pixelNum].rgb.b = b | 0x80;
}

/**
 *
 * Get the pixel color as separate color channels
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param pixelNum is the pixel number to read
 * @param[out] pr is a pointer for the returned value of the *red* color channel
 * @param[out] pg is a pointer for the returned value of the *green* color channel
 * @param[out] pb is a pointer for the returned value of the *blue* color channel
 *
 * The color channel values are returned via 3 caller supplied pointers,
 * each as a 7-bit value.  The values will all be set to 0 if there is
 * any error.
 */
void
LPD8806_LedStrip_GetPixelRgb(LPD8806_LedStrip_Handle_t h, uint32_t pixelNum,
                             uint8_t *pr, uint8_t *pg, uint8_t *pb)
{
    // Validate the return value pointers
    if(!pr || !pg || !pb)
    {
        return;
    }
    
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        *pr = 0x80;
        *pg = 0x80;
        *pb = 0x80;
        return;
    }
    
    // check that pixel is within bounds
    if(pixelNum >= pInst->numPixels)
    {
        *pr = 0x80;
        *pg = 0x80;
        *pb = 0x80;
        return;
    }

    // Get the frame buffer
    pixel_t *pfb = pInst->pFrameBuffer;
    
    // read each color channel and store in the caller suppied storage
    // upper bit of each byte is masked off since it is not part
    // of the color value
    *pr = pfb[pixelNum].rgb.r & 0x7f;
    *pg = pfb[pixelNum].rgb.g & 0x7f;
    *pb = pfb[pixelNum].rgb.b & 0x7f;
}

/**
 *
 * Set an individual color channel for a pixel
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param pixelNum is the pixel number to modify
 * @param channel is the color channel to modify: CHAN_RED, CHAN_GREEN, CHAN_BLUE
 * @param channelValue is the 7-bit value for the color channel
 *
 * This function can be used to set just a single color channel of a pixel.
 */
void
LPD8806_LedStrip_SetPixelChannel(LPD8806_LedStrip_Handle_t h, uint32_t pixelNum,
                                 LPD8806_LedStrip_Channel_t channel,
                                 uint8_t channelValue)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return;
    }
    
    // Get the frame buffer
    pixel_t *pfb = pInst->pFrameBuffer;
    
    // check that pixel is within bounds
    if(pixelNum >= pInst->numPixels)
    {
        return;
    }
    
    // process the specified color channel
    // set the appropriate part (r, g, or b) of the LED
    // upper bit must be set to satisfy the controller
    switch(channel)
    {
        case CHAN_RED:
        {
            pfb[pixelNum].rgb.r = channelValue | 0x80;
            break;
        }
        case CHAN_GREEN:
        {
            pfb[pixelNum].rgb.g = channelValue | 0x80;
            break;
        }
        case CHAN_BLUE:
        {
            pfb[pixelNum].rgb.b = channelValue | 0x80;
            break;
        }
        default:
        {
            break;
        }
    }
}

/**
 *
 * Get a pixel individual color channel
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param pixelNum is the pixel number to read
 * @param channel is the color channel to read: CHAN_RED, CHAN_GREEN, CHAN_BLUE
 * 
 * @returns Returns the value of the specified color channel as 7-bits, or
 *          0 if there is an error.
 */
uint8_t
LPD8806_LedStrip_GetPixelChannel(LPD8806_LedStrip_Handle_t h, uint32_t pixelNum,
                                 LPD8806_LedStrip_Channel_t channel)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return 0x80;
    }
    
    // Get the frame buffer
    pixel_t *pfb = pInst->pFrameBuffer;
    
    // check that pixel is within bounds
    if(pixelNum >= pInst->numPixels)
    {
        return 0x80;
    }
    
    // read the specified color channel value
    uint8_t ret;
    switch(channel)
    {
        case CHAN_RED:
        {
            ret = pfb[pixelNum].rgb.r;
            break;
        }
        case CHAN_GREEN:
        {
            ret = pfb[pixelNum].rgb.g;
            break;
        }
        case CHAN_BLUE:
        {
            ret = pfb[pixelNum].rgb.b;
            break;
        }
        default:
        {
            return 0x80;
            break;
        }
    }
    
    // mask off the upper bit because that is not part of the color value
    return ret & 0x7f;
}

/**
 * Update the LED string with the buffered values
 *
 * @param h is the LedStrip instance handle from _Init()
 *
 * The pixel values that were written to the LED "frame buffer" with the
 * above functions are copied to the physical LED string via the SSI/SPI
 * port.
 *
 * If DMA support is enabled, then this function will return right away while
 * the DMA runs in the background.  If DMA is not enabled, then this function
 * uses polled IO to copy the data to the SPI port until the entire buffer
 * has been copied to the output FIFO.
 */
void
LPD8806_LedStrip_Update(LPD8806_LedStrip_Handle_t h)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return;
    }
    
    // Get the frame buffer
    pixel_t *pfb = pInst->pFrameBuffer;
    
    // Get count and pointer to start of data
    int count = pInst->numPixels * 3;
    uint8_t *pbytes = &pfb[0].bytes[0];

    // determine if DMA is enabled.  If LPD8806_DMA_SUPPORT is 0, then the
    // compiler will recognize that this code can not be executed and compile
    // it out.
    if(LPD8806_DMA_SUPPORT && pInst->useDma)
    {
        // wait until any prior DMA transfer is complete
        while(uDMAChannelModeGet(pInst->dmaChannel | UDMA_PRI_SELECT) != UDMA_MODE_STOP)
        {}
        
        // set the required trailing 0
        pfb[pInst->numPixels].bytes[0] = 0;
        
        // set up the DMA channel transfer and enable it.  Once this happens
        // the DMA should begin running
        uDMAChannelTransferSet(pInst->dmaChannel | UDMA_PRI_SELECT, UDMA_MODE_BASIC,
                               pbytes, (void *)(pInst->spiBase + SSI_O_DR),
                               count + 1);
        uDMAChannelEnable(pInst->dmaChannel);
    }
    
    // otherwise, handle by IO polling
    else
    {
        // Wait for any prior transaction to complete
        while(SSIBusy(pInst->spiBase))
        {}
        
        // loop until all bytes are written
        while(count--)
        {
            SSIDataPut(pInst->spiBase, *pbytes++);
        }
        
        // write the restart byte(s)
        for (uint32_t extra = 0; extra < pInst->latchBytes; extra++)
        {
            SSIDataPut(pInst->spiBase, 0);
        }
    }
}

/**
 * Get the number of pixels in the instance.
 *
 * @param h is the LedStrip instance handle from _Init()
 *
 * @returns Returns the number of pixels supported for the specified
 *          LedStrip instance.
 */
uint32_t
LPD8806_LedStrip_NumPixels(LPD8806_LedStrip_Handle_t h)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return 0;
    }
    
    return pInst->numPixels;
}

/**
 * Initialize the LED strip driver hardware
 *
 * @param sysClock is the value of the system clock
 * @param spiIdx is the index of the SPI peripheral to use (0-origin)
 * @param spiRate is the data rate to use for SPI
 *
 * The application must call this function before using any other functions
 * in this driver.  It is used to set up a hardware instance for use by
 * the driver.  The parameter *sysClock* is the value of the MCU system
 * clock frequency.  It is needed for initializing the SPI peripheral.
 * The parameter *spiIdx* is the SPI peripheral number to use (0, 1, etc).
 * It is limited to the number of SPI peripherals in the system.
 * The *spiRate* is the desired clock frequency of the SPI peripheral.  There
 * is no validation of this parameter.  The driver will attempt to set
 * the specified value of the SPI clock rate but if it is not valid then
 * the init will probably fail silently.
 *
 * @note The application must also call LPD8806_LedStrip_SetFrameBuffer()
 *       before this driver can be used.
 *
 * @returns Returns a handle that is used for the other LedStrip function.
 *          The returned value will be NULL if there is any error with
 *          the initialization.
 */
LPD8806_LedStrip_Handle_t
LPD8806_LedStrip_Init(uint32_t sysClock, uint32_t spiIdx, uint32_t spiRate)
{
    // Check for valid parameters
    if (spiIdx >= NUM_SPI_PERIPHS)
    {
        return NULL;
    }
    
    // Attempt to allocate an instance for this new LedStrip
    Instance_t *pInst = NULL;
    for(uint32_t idx = 0; idx < LPD8806_LEDSTRIP_NUM_INSTANCES; idx++)
    {
        if(instances[idx].spiPeriph == 0)
        {
            pInst = &instances[idx];
            break;
        }
    }
    
    // Check to see if one was allocated
    if(!pInst)
    {
        return NULL;
    }
    
    // Save the SPI periph and enable it
    pInst->spiPeriph = spiPeriphTable[spiIdx].periph;
    SysCtlPeripheralEnable(pInst->spiPeriph);

    // Save the reset of the SPI config
    // Things are done in this order so there is some time gap from
    // PeripheralEnable() until peripheral is used
    pInst->spiBase = spiPeriphTable[spiIdx].base;
    pInst->spiRate = spiRate;
    
    // Configure the SSI(SPI) port used to control the display
    SSIDisable(pInst->spiBase);
    SSIConfigSetExpClk(pInst->spiBase, sysClock, SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, pInst->spiRate, 8);
    SSIEnable(pInst->spiBase);

    // Return the new handle to the caller
    return (void *)pInst;
}

/**
 * Set the frame buffer for an LedStrip instance
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param pFrameBuffer is a pointer to storage for the frame buffer
 * @param numPixels is the number of pixels for the frame buffer
 *
 * This function assigns a frame buffer to the LedStrip instance.  This
 * must be called at least once following LPD8806_LedStrip_Init().  It can
 * be called other times if needed to change the frame buffer for an LedStrip.
 * For example if there is a pre-rendered frame buffer it could be used to
 * switch between computed and pre-rendered frame buffer.
 *
 * The caller must allocate the space for the frame buffer, *pFrameBuffer*.
 * The macro LPD8806_FRAMEBUF_SIZE can be used to get the number of bytes
 * that are needed for a frame buffer given the number of pixels to store.
 */
void
LPD8806_LedStrip_SetFrameBuffer(LPD8806_LedStrip_Handle_t h, void *pFrameBuffer,
                                uint32_t numPixels)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;

    // Check for basic instance validity
    if(!pInst || (pInst->spiPeriph == 0))
    {
        return;
    }
    
    // Check for valid frame buffer
    if(!pFrameBuffer || (numPixels == 0))
    {
        return;
    }
    
    // Assign the frame buffer to the instance
    pInst->numPixels = numPixels;
    pInst->pFrameBuffer = pFrameBuffer;
    
    // Compute the number of "latch" bytes that are needed.
    // These are extra bytes inserting into the data stream and are
    // required by the lpd8806
    pInst->latchBytes = (numPixels + 31) / 32;
}

/**
 * Clear the frame buffer
 *
 * @param h is the LedStrip instance handle from _Init()
 *
 * This is a convenience function the clear the contents of the frame
 * buffer back to "zero", meaning all pixels are off.  This function does
 * not update the hardware.
 */
void
LPD8806_LedStrip_Clear(LPD8806_LedStrip_Handle_t h)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return;
    }
    
    // Get the frame buffer
    pixel_t *pfb = pInst->pFrameBuffer;

    // clear out all the pixel values and set the upper bits, which is
    // needed by the controler
    for(int i = 0; i < pInst->numPixels; i++)
    {
        pfb[i].bytes[0] = 0x80;
        pfb[i].bytes[1] = 0x80;
        pfb[i].bytes[2] = 0x80;
    }
}

/**
 * Enable the use of DMA by the driver
 *
 * @param h is the LedStrip instance handle from _Init()
 * @param dmaChannel is the DMA channel number that should be used for the SSI/SPI
 *
 * This function assigns the DMA channel to the LedStrip instance and enables
 * use of DMA when writing pixel data using the SPI peripheral.  The caller
 * must provide a valid DMA channel and configuration - nothing is done
 * to validate the DMA channel.
 *
 * The application must first set up the DMA controller and configure the
 * correct DMA channel for this peripheral before calling this function.
 */
#ifdef LPD8806_LEDSTRIP_ALLOW_DMA
void
LPD8806_LedStrip_EnableDma(LPD8806_LedStrip_Handle_t h, uint32_t dmaChannel)
{
    // Get instance pointer
    Instance_t *pInst = (Instance_t *)h;
    
    // Validate the instance
    if(!InstanceIsValid(pInst))
    {
        return;
    }
    
    // save DMA flag and channel number
    pInst->useDma = true;
    pInst->dmaChannel = dmaChannel;
    
    // set up the DMA channel
    uDMAChannelControlSet(dmaChannel | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                          UDMA_DST_INC_NONE | UDMA_ARB_4);
    
    // configure the SSI/SPI peripheral to generate DMA request
    SSIDMAEnable(pInst->spiBase, SSI_DMA_TX);
}
#endif

/**
 * @}
 */

#ifdef LPD8806_LEDSTRIP_COMPILER_TEST
/******************************************************************************
 *
 * TEST CODE
 *
 * The following is test code just for the purpose of a compiler test.
 * It is not functional.
 *
 *****************************************************************************/

#define NUM_TEST_PIXELS 40
static uint8_t frameBuffer[LPD8806_FRAMEBUF_SIZE(NUM_TEST_PIXELS)];

void
LPD8806_LedStrip_Test(void)
{
    LPD8806_LedStrip_Handle_t h;
    
    h = LPD8806_LedStrip_Init(SysCtlClockGet(), 0, 500000);
    LPD8806_LedStrip_SetFrameBuffer(h, frameBuffer, NUM_TEST_PIXELS);
    LPD8806_LedStrip_Clear(h);
    LPD8806_LedStrip_Update(h);
    LPD8806_LedStrip_SetPixelChannel(h, 0, CHAN_BLUE, 0x40);
    uint8_t blue = LPD8806_LedStrip_GetPixelChannel(h, 0, CHAN_BLUE);
    LPD8806_LedStrip_SetPixelValue(h, 0, 0x123456);
    uint32_t value = LPD8806_LedStrip_GetPixelValue(h, 0);
    LPD8806_LedStrip_SetPixelRgb(h, 0, 1, 2, 3);
    uint8_t r, g, b;
    LPD8806_LedStrip_GetPixelRgb(h, 0, &r, &g, &b);
    uint32_t numPixels = LPD8806_LedStrip_NumPixels(h);
    numPixels = numPixels;
    value = value;
    blue = blue;
}
#endif
