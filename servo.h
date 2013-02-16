
#ifndef servo_h
#define servo_h

/**
 * @addtogroup servo_driver
 * @{
 */

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

typedef void *ServoHandle_t;

extern int ServoInit(uint32_t sysClock, uint32_t timerPeriodUsecs);
extern ServoHandle_t ServoConfig(uint32_t timerIdx, uint32_t timerHalf);
extern int ServoCalibrate(ServoHandle_t hServo, int32_t usecsPer90,
                          int32_t usecsAtZero, bool reverseAngle);
extern int ServoSetLimits(ServoHandle_t hServo, int32_t usecsMin,
                          int32_t usecsMax);
extern int ServoSetPositionUsecs(ServoHandle_t hServo, int32_t usecs);
extern int ServoSetPosition(ServoHandle_t hServo, int32_t centiDegrees);
extern int32_t ServoGetPositionUsecs(ServoHandle_t hServo);
extern int32_t ServoGetPosition(ServoHandle_t hServo);
extern int ServoEnable(ServoHandle_t hServo);
extern int ServoDisable(ServoHandle_t hServo);
extern int ServoSetTickObserver(ServoHandle_t hServo,
                                void (*pfnServoTickObserver)(void *pObserverData),
                                void *pObserverData);
extern int ServoMoveUsecs(ServoHandle_t hServo, int32_t usecs,
                          void (*pfnMotionObserver)(void *pObserverData),
                          void *pObserverData);
extern int ServoMove(ServoHandle_t hServo, int32_t centiDegrees,
                     void (*pfnMotionObserver)(void *pObserverData),
                     void *pObserverData);

extern int ServoSetMotionParameters(ServoHandle_t hServo, int32_t rateDegPerSec);

#endif
