/******************************************************************************
 *
 * lpd8806_ledstrip.h - Driver for RGB LED strip using LPD8806 controller
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

#ifndef lpd8806_ledstrip_h
#define lpd8806_ledstrip_h

/**
 * @addtogroup lpd8806_ledstrip_driver
 * @{
 */

/******************************************************************************
 *
 * Macros
 *
 *****************************************************************************/

/**
 * Existence of this macro allows DMA support to be built in.  This leaves
 * use of DMA as a run-time option.  If this macro is not defined or undef'd
 * then all DMA support is compiled out and it will not be a run-time option.
 * This value can be changed by editing the header file, or by defining or
 * undefining the macro on the command line.  This macro is defined by default.
 */
#ifndef LPD8806_LEDSTRIP_ALLOW_DMA
#define LPD8806_LEDSTRIP_ALLOW_DMA
#endif

/**
 * This macro defines the number of LedStrip instances that are supported
 * by the driver.  The larger the number of instances, the more memory used
 * by the driver.  The default value is 2.  If you have a system with more
 * RGB LedStrips than that, then you can increase this number.
 */
#ifndef LPD8806_LEDSTRIP_NUM_INSTANCES
#define LPD8806_LEDSTRIP_NUM_INSTANCES 2
#endif

/**
 * This convenience macro computes the number of bytes that are required
 * for a frame buffer given the number of pixels.  It can be used by the
 * application to allocate memory for the frame buffer before calling
 * LPD8806_LedStrip_SetFrameBuffer().
 * The frame buffer must be one place longer than the number of pixels.
 */
#define LPD8806_LEDSTRIP_FRAMEBUF_SIZE(numPixels) (((numPixels) + 1) * 3)

/******************************************************************************
 *
 * Types
 *
 *****************************************************************************/

/**
 * Define channel values that can be used for LPD8806_LedStrip_SetPixelChannel()
 * and LPD8806_LedStrip_GetPixelChannel()
 */
typedef enum
{
    CHAN_RED,   /**< Red color channel */
    CHAN_GREEN, /**< Green color channel */
    CHAN_BLUE   /**< Blue color channel */
} LPD8806_LedStrip_Channel_t;

/**
 * Define the instance handle to be used for all LedStrip API functions.
 * This value is returned from LPD8806_LedStrip_Init() and must be passed to
 * all other functions.
 */
typedef void * LPD8806_LedStrip_Handle_t;

/**
 * @}
 */

/******************************************************************************
 *
 * Function Prototypes
 *
 *****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

extern void LPD8806_LedStrip_SetPixelValue(LPD8806_LedStrip_Handle_t h,
                                           uint32_t pixelNum, uint32_t rgb);
extern uint32_t LPD8806_LedStrip_GetPixelValue(LPD8806_LedStrip_Handle_t h,
                                               uint32_t pixelNum);
extern void LPD8806_LedStrip_SetPixelRgb(LPD8806_LedStrip_Handle_t h,
                                         uint32_t pixelNum, uint8_t r,
                                         uint8_t g, uint8_t b);
extern void LPD8806_LedStrip_GetPixelRgb(LPD8806_LedStrip_Handle_t h,
                                         uint32_t pixelNum, uint8_t *pr,
                                         uint8_t *pg, uint8_t *pb);
extern void LPD8806_LedStrip_SetPixelChannel(LPD8806_LedStrip_Handle_t h,
                                             uint32_t pixelNum,
                                             LPD8806_LedStrip_Channel_t channel,
                                             uint8_t channelValue);
extern uint8_t LPD8806_LedStrip_GetPixelChannel(LPD8806_LedStrip_Handle_t h,
                                                uint32_t pixelNum,
                                                LPD8806_LedStrip_Channel_t channel);
extern uint32_t LPD8806_LedStrip_NumPixels(LPD8806_LedStrip_Handle_t h);
extern void LPD8806_LedStrip_Update(LPD8806_LedStrip_Handle_t h);
extern LPD8806_LedStrip_Handle_t LPD8806_LedStrip_Init(unsigned long ulSysClock,
                                                       uint32_t spiIdx,
                                                       uint32_t spiRate);
extern void LPD8806_LedStrip_SetFrameBuffer(LPD8806_LedStrip_Handle_t h,
                                            void *pFrameBuffer,
                                            uint32_t numPixels);
void LPD8806_LedStrip_Clear(LPD8806_LedStrip_Handle_t h);
extern void LPD8806_LedStrip_EnableDma(LPD8806_LedStrip_Handle_t h,
                                       uint32_t dmaChannel);

#ifdef __cplusplus
}
#endif

#endif
