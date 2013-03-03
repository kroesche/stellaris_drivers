/******************************************************************************
 *
 * servo-wt.c - Servo driver (wide timer) for TI Stellaris microcontroller
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

#ifndef __SERVO_H__
#define __SERVO_H__

/**
 * @addtogroup servo_wt_driver
 * @{
 */

/******************************************************************************
 *
 * Macros
 *
 *****************************************************************************/

/**
 * Specifies the A "half" of the timer to use for the servo.
 */
#define SERVO_TIMER_A   0
/**
 * Specifies the B "half" of the timer to use for the servo.
 */
#define SERVO_TIMER_B   1

/**
 * @}
 */

/******************************************************************************
 *
 * Prototypes
 *
 *****************************************************************************/
typedef void *ServoHandle_t;

extern int Servo_Init(uint32_t sysClock, uint32_t timerPeriodUsecs);
extern ServoHandle_t Servo_Config(uint32_t timerIdx, uint32_t timerHalf);
extern int Servo_Calibrate(ServoHandle_t hServo, int32_t usecsPer90,
                           int32_t usecsAtZero, bool reverseAngle);
extern int Servo_SetLimits(ServoHandle_t hServo, int32_t usecsMin,
                           int32_t usecsMax);
extern int Servo_SetPositionUsecs(ServoHandle_t hServo, int32_t usecs);
extern int Servo_SetPosition(ServoHandle_t hServo, int32_t centiDegrees);
extern int32_t Servo_GetPositionUsecs(ServoHandle_t hServo);
extern int32_t Servo_GetPosition(ServoHandle_t hServo);
extern int Servo_Enable(ServoHandle_t hServo);
extern int Servo_Disable(ServoHandle_t hServo);
extern int Servo_SetTickObserver(ServoHandle_t hServo,
                                 void (*pfnServoTickObserver)(void *pObserverData),
                                 void *pObserverData);
extern int Servo_MoveUsecs(ServoHandle_t hServo, int32_t usecs,
                           void (*pfnMotionObserver)(void *pObserverData),
                           void *pObserverData);
extern int Servo_Move(ServoHandle_t hServo, int32_t centiDegrees,
                      void (*pfnMotionObserver)(void *pObserverData),
                      void *pObserverData);
extern int Servo_SetMotionParameters(ServoHandle_t hServo, int32_t rateDegPerSec);

extern void Servo_BatteryInit(uint32_t gain, uint32_t offset);
extern uint32_t Servo_ReadBatteryMv(void);
extern uint32_t Servo_ReadBatteryRaw(void);

#endif
