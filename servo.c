/******************************************************************************
 *
 * servo.c - Servo driver for TI Stellaris microcontroller
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

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"

#include "driverlib/debug.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"

#include "servo.h"

/**
 * @defgroup servo_driver Servo Driver
 *
 * This is a servo driver for TI Stellaris microcontrollers.  Specifically
 * it is for the LM4F parts (such as LM4F120 or LM4F232) because this driver
 * depends on the wide timers.  The wide timers are not available on older
 * LM3S devices.
 *
 * This driver provides a set of functions for configuring and using timers
 * for servo motion control.  It uses the PWM mode of the timers to generate
 * the appropriate pulse-width control for typical hobby servos.
 *
 * Two unit systems are provided for dealing with servo position: microseconds
 * and centi-degrees.  Because the servo position is controlled by the pulse
 * width of the timer output signal, the position can be represented as a pulse
 * width.  Therefore microseconds is used as the units to represent the pulse
 * width and thus the servo position.  The other option for units is
 * centi-degrees, which is degrees * 10.  This allows resolution of 0.1
 * degrees which should be sufficient for hobby applications.  Using
 * centi-degrees allows fractional degree resolution without requiring
 * floating point.
 *
 * Servos can be moved two ways: direct position and motion control.  When
 * direct position is used, the servo is just immediately commanded to the
 * new position and the rate of motion is whatever the servo provides.  When
 * motion control is used, the rate of motion is controlled.  This allows for
 * smoother and less jerky control.
 *
 * The servo enable/disable can be used to apply/remove the pulse signal to
 * the servos.  It may be useful to disable a servo once it has reached the
 * desired position so that the servo stops hunting around or jittering at
 * the target position.
 *
 * Two forms of callback are provided: tick and motion.  The tick callback
 * allows a client function to be called at each pulse period (timer tick).
 * This allows for varying the pulse width at the pulse frequency.  This could
 * be used for some advanced control method where the position is varied over
 * time according to some algorithm.  The motion control makes use of this,
 * so if you install a tick callback and then use a Move() function, your
 * tick callback will be deleted.  The motion callback is used with the
 * Move() functions.  Since Move() is asynchronous and returns before the
 * motion is done, this allows a client to be notified when the motion is
 * finished.
 *
 * @todo add acceleration feature to motion control
 * @todo extend driver to work without wide timer
 *
 * EXAMPLE:
 *  Here is code showing how you might use this driver in an example:
 *
 *      // init the driver
 *      err = ServoInit(SysControlClockGet(), 20000); // 50 Hz
 *
 *      // get a servo instance using wide timer 4A
 *      handle = ServoConfig(4, SERVO_TIMER_A)
 *
 *      // enable the servo (this establishes a known position)
 *      err = ServoEnable(handle);
 *
 *      // set motion rate for 90 deg per sec
 *      err = ServoSetMotionParameters(handle, 90);
 *
 *      // command the servo to move to +45 deg
 *      err = ServoMove(handle, 450, NULL, NULL);
 *
 * @{
 */

/******************************************************************************
 *
 * Macros
 *
 *****************************************************************************/

/* Define the following if you want debug messages to serial port.
 * Undefine or comment out to disable debug messages.
 */
//#define SERVO_DEBUG_MSGS

/* Define the number of servo instances allowed.  It does not make sense for
 * this to be any larger than the number of wide timer halves.
 */
#define NUM_SERVOS 12

/* These are the default calibration and other values for the servos. These
 * will be used until the user overrides them at run-time by function call.
 */
#define DEFAULT_USECS_PER_90    1000    /* pulse width 1ms */
#define DEFAULT_USECS_AT_ZERO   1500    /* pulse width 1.5 ms - middle */
#define DEFAULT_REVERSE_ANGLE   false
#define DEFAULT_USECS_MIN       500     /* min pulse 0.5 ms */
#define DEFAULT_USECS_MAX       2500    /* max pulse 2.5 ms */
#define DEFAULT_INIT_USECS      1500    /* assumed initial position is 1.5 */
#define DEFAULT_RATE            90      /* default rate in deg per sec */

/* Defines function for printing debug messages.  Control with
 * SERVO_DEBUG_MSGS (above)
 */
#ifdef SERVO_DEBUG_MSGS
#include "utils/uartstdio.h"
#define DbgPrintf(fmt, ...) UARTprintf(fmt, __VA_ARGS__)
#else
#define DbgPrintf(fmt, ...) (void)0
#endif

/******************************************************************************
 *
 * Locals
 *
 *****************************************************************************/

/* Store the number of timer cycles per microsecond, and the timer PWM
 * period in microseconds.
 */
static int32_t gCyclesPerUsec;
static int32_t gTimerPeriodUsecs;

/******************************************************************************
 *
 * The following structure defines an instance of a timer that is used for
 * the servo PWM generation.
 *
 *****************************************************************************/
typedef struct
{
    uint32_t timerPeriph;       /* timer peripheral def for sysctl */
    uint32_t timerBase;         /* timer base address */
    uint32_t timerHalf;         /* which timer half, TIMER_A or TIMER_B */
    uint32_t timerInt;          /* interrupt number for this timer */
    uint32_t timerGpioPeriph;   /* GPIO peripheral for the timer pin */
    uint32_t timerGpioBase;     /* GPIO base address for the timer pin */
    uint32_t timerPin;          /* GPIO pin for the timer, GPIO_PIN_n */
    uint32_t timerPinConfig;    /* pin configuraton used to configure pin mux */
} Timer_t;

/* The following table defines all the timers (A and B halves) that are used
 * for servo control.  This table is stored in flash.
 */
static const Timer_t timerTable[] =
{
    { SYSCTL_PERIPH_WTIMER0, WTIMER0_BASE, TIMER_A, INT_WTIMER0A,
        SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PC4_WT0CCP0 },
    { SYSCTL_PERIPH_WTIMER0, WTIMER0_BASE, TIMER_B, INT_WTIMER0B,
        SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PC5_WT0CCP1 },
    { SYSCTL_PERIPH_WTIMER1, WTIMER1_BASE, TIMER_A, INT_WTIMER1A,
        SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_PC6_WT1CCP0 },
    { SYSCTL_PERIPH_WTIMER1, WTIMER1_BASE, TIMER_B, INT_WTIMER1B,
        SYSCTL_PERIPH_GPIOC, GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PC7_WT1CCP1 },
    { SYSCTL_PERIPH_WTIMER2, WTIMER2_BASE, TIMER_A, INT_WTIMER2A,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PD0_WT2CCP0 },
    { SYSCTL_PERIPH_WTIMER2, WTIMER2_BASE, TIMER_B, INT_WTIMER2B,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PD1_WT2CCP1 },
    { SYSCTL_PERIPH_WTIMER3, WTIMER3_BASE, TIMER_A, INT_WTIMER3A,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PD2_WT3CCP0 },
    { SYSCTL_PERIPH_WTIMER3, WTIMER3_BASE, TIMER_B, INT_WTIMER3B,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PD3_WT3CCP1 },
    { SYSCTL_PERIPH_WTIMER4, WTIMER4_BASE, TIMER_A, INT_WTIMER4A,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_4, GPIO_PD4_WT4CCP0 },
    { SYSCTL_PERIPH_WTIMER4, WTIMER4_BASE, TIMER_B, INT_WTIMER4B,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_5, GPIO_PD5_WT4CCP1 },
    { SYSCTL_PERIPH_WTIMER5, WTIMER5_BASE, TIMER_A, INT_WTIMER5A,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PD6_WT5CCP0 },
    { SYSCTL_PERIPH_WTIMER5, WTIMER5_BASE, TIMER_B, INT_WTIMER5B,
        SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PD7_WT5CCP1 },
};
#define NUM_TIMERS (sizeof(timerTable) / sizeof(Timer_t))

/******************************************************************************
 *
 * The following structure defines a servo with all its state information.
 *
 *****************************************************************************/
typedef struct
{
    /* Instance of timer that is controlling this servo.
     */
    const Timer_t *pTimer;
    
    /* Calibration data:
     * - number of microseconds pulse width change for 90 degrees
     * - pulse width, in microseconds, when the servo is at zero position
     * - minimum pulse width in microseconds
     * - maximum pulse width in microseconds
     * - reverse angle flag (move in opposite direction)
     */
    int32_t usecsPer90;
    int32_t usecsAtZero;
    int32_t usecsMin;
    int32_t usecsMax;
    bool reverseAngle;
    
    /* The amount of pulse width change in microseconds that should occur
     * per pulse-width period when the servo is moving, representing the
     * rate of motion.
     */
    int32_t rateUsecsPerTick;
    
    /* The current position and target position in microseconds, and a flag
     * to indicate when motion is complete.
     */
    int32_t currentUsecs;
    int32_t targetUsecs;
    bool motionComplete;
    
    /* Holds the tick callback function for this server, if any.  This is
     * called for every PWM period.
     */
    void (*pfnTickObserver)(void *pTickObserverData);
    
    /* A data pointer that is to be passed to the tick observer.
     */
    void *pTickObserverData;
    
    /* Holds the callback function for the motion observer, if any.  This is
     * called when motion is complete.
     */
    void (*pfnMotionObserver)(void* pMotionObserverData);
    
    /* A data pointer that is to be passed to the motion observer.
     */
    void *pMotionObserverData;
} Servo_t;

/* This array is used to map timers to servos.  The servo structure above
 * maps servos to timers, but we need one to go the other direction.  The
 * timer structure above is not used because we want to keep it as const so
 * it can remain in flash, and just use a separate array in RAM for this
 * data item.
 */
static Servo_t *timerToServoMap[NUM_TIMERS];

/* This array holds the servo instances
 */
static Servo_t servoTable[NUM_SERVOS];

/******************************************************************************
 *
 * Private Functions
 *
 *****************************************************************************/

/* @internal
 * Calculate microseconds of absolute angle from centi-degrees
 *
 * @param[in] pServo is the servo instance
 * @param[in] centiDegrees is the absolute angle in centi-degrees
 *
 * This function converts an absolute position angle specified in centi-degrees
 * to the pulse width representing that angle, in microseconds.  Internal to
 * this driver, all angles are represented in microseconds of pulse width.
 *
 * @returns The absolute angle in microseconds of pulse width.
 */
static int32_t
CalcUsecsFromCentiDegreesAbs(Servo_t *pServo, int32_t centiDegrees)
{
    int32_t usecs;
    
    /* initial multiply
     */
    usecs = centiDegrees * pServo->usecsPer90;
    
    /* add in (or subtract if input is negative) the value for
     * 1/2 of usec for rounding
     */
    usecs += ((centiDegrees < 0) ? -1 : 1) * 450;
    
    /* divide by 900 centideg per 90 deg to get usecs delta (signed)
     */
    usecs /= 900;
    
    /* change the sign if the servo is specified to reverse angle
     */
    usecs = pServo->reverseAngle ? -usecs : usecs;
    
    /* finally, add the delta to the zero point to get the actual
     * position in usecs
     */
    usecs = pServo->usecsAtZero + usecs;
    
    /* return final value to caller
     */
    return(usecs);
}

/* @internal
 * Calculate microseconds of relative angle from centi-degrees
 *
 * @param[in] pServo is the servo instance
 * @param[in] centiDegrees is the relative angle in centi-degrees
 *
 * This function converts a relative position angle specified in centi-degrees
 * to the pulse width representing that angle, in microseconds.  Internal to
 * this driver, all angles are represented in microseconds of pulse width.
 *
 * @returns The relative angle in microseconds of pulse width.
 */
static int32_t
CalcUsecsFromCentiDegreesRel(Servo_t *pServo, int32_t centiDegrees)
{
    int32_t usecs;
    
    /* initial multiply
     */
    usecs = centiDegrees * pServo->usecsPer90;
    
    /* add in (or subtract if input is negative) the value for
     * 1/2 of usec for rounding
     */
    usecs += ((centiDegrees < 0) ? -1 : 1) * 450;
    
    /* divide by 900 centideg per 90 deg to get usecs delta (signed)
     */
    usecs /= 900;
    
    /* return final value to caller
     */
    return(usecs);
}

/** @internal
 * Calculate centi-degrees of absolute angle from microseconds of pulse width
 *
 * @param[in] pServo is the servo instance
 * @param[in] usecs is the absolute angle in microseconds of pulse width
 *
 * This function converts a servo absolute angle given in microseconds of pulse
 * width, to centi-degrees.
 *
 * @returns The absolute angle in centi-degrees.
 */
static int32_t
CalcCentiDegreesFromUsecsAbs(Servo_t *pServo, int32_t usecs)
{
    int32_t cdeg;
    
    /* compute relative pulse width, from zero
     */
    usecs -= pServo->usecsAtZero;
    
    /* change the sign if the servo is specified to reverse angle
     */
    usecs = pServo->reverseAngle ? -usecs : usecs;
    
    /* multiply by centideg per 90deg
     */
    cdeg = usecs * 900;
    
    /* adjust for rounding on the next divide
     */
    cdeg += ((cdeg < 0) ? -1 : 1) * (pServo->usecsPer90 / 2);
    
    /* divide by the usecs per 90deg angle
     */
    cdeg /= pServo->usecsPer90;
    
    return(cdeg);
}

#if 0 // leave out until needed
/** @internal
 * Calculate centi-degrees of relative angle from microseconds of pulse width
 *
 * @param[in] pServo is the servo instance
 * @param[in] usecs is the relative angle in microseconds of pulse width
 *
 * This function converts a relative angle given in microseconds of pulse
 * width, to centi-degrees.
 *
 * @returns The relative angle in centi-degrees.
 */
static int32_t
CalcCentiDegreesFromUsecsRel(Servo_t *pServo, int32_t usecs)
{
    int32_t cdeg;

    /* multiply by centideg per 90deg
     */
    cdeg = usecs * 900;
    
    /* adjust for rounding on the next divide
     */
    cdeg += pServo->usecsPer90 / 2;
    
    /* divide by the usecs per 90deg angle
     */
    cdeg /= pServo->usecsPer90;
    
    return(cdeg);
}
#endif

/* @internal
 * Handle servo motion each PWM period
 *
 * @param[in] hServo is the servo handle
 *
 * When a servo move has been commanded, this function is called once
 * per PWM period to move the servo by the amount specified by the motion
 * rate (previously configured).
 */
static void
ServoMotionHandler(void *pvServo)
{
    Servo_t *pServo = (Servo_t *)pvServo;
    const Timer_t *pTimer = pServo->pTimer;
    
    /* because this is being called from an interrupt handler that has
     * already been set up with validated servo and timer instances, we
     * choose to assume that the servo and timer instances are valid and
     * not re-check them here.  This function will be called every PWM
     * period so it is a good idea to keep it as short as possible.
     */
    
    /* check to see if at the target position
     */
    if(pServo->targetUsecs == pServo->currentUsecs)
    {
        /* stop the timer interrupt so we are not called any more
         */
        ROM_IntDisable(pTimer->timerInt);
        
        pServo->motionComplete = true;
        
        /* if there is a motion observer, then call it to notify
         * motion is complete
         */
        if(pServo->pfnMotionObserver)
        {
            pServo->pfnMotionObserver(pServo->pMotionObserverData);
        }
        
        return;
    }
    
    /* if the target value is greater than the current, the add the
     * motion rate to get the next position
     */
    else if(pServo->targetUsecs > pServo->currentUsecs)
    {
        if((pServo->targetUsecs - pServo->currentUsecs) < pServo->rateUsecsPerTick)
        {
            pServo->currentUsecs = pServo->targetUsecs;
        }
        else
        {
            pServo->currentUsecs += pServo->rateUsecsPerTick;
        }
    }
    
    /* otherwise the target value is less, so subtract the motion rate
     */
    else
    {
        if((pServo->currentUsecs - pServo->targetUsecs) < pServo->rateUsecsPerTick)
        {
            pServo->currentUsecs = pServo->targetUsecs;
        }
        else
        {
            pServo->currentUsecs -= pServo->rateUsecsPerTick;
        }
    }
    
    /* constrain the position to the limits
     */
    pServo->currentUsecs = (pServo->currentUsecs > pServo->usecsMax) ?
                           pServo->usecsMax : pServo->currentUsecs;
    pServo->currentUsecs = (pServo->currentUsecs < pServo->usecsMin) ?
                           pServo->usecsMin : pServo->currentUsecs;

    /* set the output pulse width to the new value
     */
    ROM_TimerMatchSet(pTimer->timerBase, pTimer->timerHalf,
                      (gTimerPeriodUsecs - pServo->currentUsecs) * gCyclesPerUsec);
}

/* @internal
 * Interrupt handler for wide timer interrupts
 *
 * @param[in] timerIdx is an index into the timer instance table
 *
 * This function handles wide timer interrupts.  It clears the timer
 * interrupt and then calls the tick callback function, if any.
 */
static void
WTimerIntHandler(uint32_t timerIdx)
{
    /* get the servo instance for this timer, and the timer instance
     */
    Servo_t *pServo = timerToServoMap[timerIdx];
    const Timer_t *pTimer = pServo->pTimer;
    
    /* clear the PWM interrupt (CAPn is used for PWM)
     */
    ROM_TimerIntClear(pTimer->timerBase,
                      (pTimer->timerHalf == TIMER_A) ? TIMER_CAPA_EVENT :
                      TIMER_CAPB_EVENT);
    
    /* see if there is a valid handler and if so call it
     */
    if(pServo->pfnTickObserver)
    {
        pServo->pfnTickObserver(pServo->pTickObserverData);
    }
}

/******************************************************************************
 *
 * API Functions
 *
 *****************************************************************************/

/**
 * Initialize the servo driver.
 *
 * @param[in] sysClock is the system clock rate used to run the timers.
 * @param[in] timerPeriodUsecs is the timer period in microseconds, to used for
 *            servo pulse width generation.
 *
 * This function must be called to initialize the servo driver and before
 * any of the other servo functions are called.  The parameter timerPeriodUsecs
 * is used to set the timer that is used for teh servo pulse period.  The same
 * value will be used for all the servos managed by this driver.  The parameter
 * sysClock is the value of the system clock or time base used for the timers.
 * Internally this value is used to compute the number of timer ticks per
 * microsecond so it shoud always be a whole MHz value.  If fractional MHz
 * is used for the system clock, then pulse width calculations will not be
 * correct.
 *
 * @returns Zero is returned for success, non-zero for the following reasons:
 *          driver is already initialized (-1)
 */
int
ServoInit(uint32_t sysClock, uint32_t timerPeriodUsecs)
{
    DbgPrintf("+ServoInit(sysClock=%u, timerPeriodUsecs=%u)\n",
              sysClock, timerPeriodUsecs);
    
    ASSERT(sysClock != 0);
    ASSERT((sysClock % 1000000) != 0);
    ASSERT(timerPeriodUsecs != 0);
    
    /* Check to see if the values are already set
     */
    if(gCyclesPerUsec || gTimerPeriodUsecs)
    {
        DbgPrintf(" ServoInit() already initialized, ret (-1)\n", 0);
        return(-1);
    }
    
    /* Compute the number of system clock cycles per microsecond and
     * save along with the timer period.
     */
    gCyclesPerUsec = sysClock / 1000000;
    gTimerPeriodUsecs = timerPeriodUsecs;
    
    DbgPrintf("-ServoInit(), ret(0)\n", 0);
    return(0);
}

/**
 * Configure a servo instance.
 *
 * @param[in] timerIdx is the wide timer number to use for the servo
 * @param[in] timerHalf is the half of the timer for this servo, A (0) or B(1).
 *            Macros SERVO_TIMER_A and SERVO_TIMER_B can be used.
 *
 * This function allocates and configures a servo instance.  The caller
 * specifies which wide timer is to be used for the servo.  That timer
 * will then be initialized to be used for servo PWM generation and the
 * servo instance data will be populated.  Default values will be used
 * for all the calibration data.
 *
 * @returns A servo handle is returned if the function is successful.  The
 * servo handle should be used for all other servo driver function calls.
 * If there is any error, then a NULL handle is returned.  Possible errors
 * are: the timer has already been allocated for the servo; there are no
 * available servo instances.
 */
ServoHandle_t
ServoConfig(uint32_t timerIdx, uint32_t timerHalf)
{
    Servo_t *pServo;
    const Timer_t *pTimer;
    uint32_t servoIdx;
    
    DbgPrintf("+ServoConfig(timerIdx=%u, timerHalf=%u)\n", timerIdx, timerHalf);
    
    /* Make sure input parameters are not obviously bad
     */
    if((timerIdx >= (NUM_TIMERS / 2)) || (timerHalf > 1))
    {
        DbgPrintf(" error: bad input value\n", 0);
        return(NULL);
    }
    
    /* check to see if module has been initialized
     */
    if((gCyclesPerUsec == 0) || (gTimerPeriodUsecs == 0))
    {
        DbgPrintf(" error: module not initialized\n", 0);
        return(NULL);
    }
    
    /* Find a free servo instance
     */
    for(servoIdx = 0; servoIdx < NUM_SERVOS; servoIdx++)
    {
        if(servoTable[servoIdx].pTimer == NULL)
        {
            break;
        }
    }
    
    /* check to see if we ran off end of servo table
     */
    if(servoIdx >= NUM_SERVOS)
    {
        DbgPrintf(" error: no available servo instances\n", 0);
        return(NULL);
    }
    
    /* adjust the caller supplied timer number to be an index into our
     * timer table
     */
    timerIdx *= 2;
    timerIdx += timerHalf;
    
    /* get pointers to the servo and timer instances
     */
    pServo = &servoTable[servoIdx];
    pTimer = &timerTable[timerIdx];
    
    /* check to see if this timer is already in use
     */
    if(timerToServoMap[timerIdx])
    {
        DbgPrintf(" error: timer already in use\n", 0);
        return(NULL);
    }
    
    /* check to make sure this timer exists
     */
    if(!SysCtlPeripheralPresent(pTimer->timerPeriph))
    {
        DbgPrintf(" error: physical timer not present\n", 0);
        return(NULL);
    }
    
    /* connect the servo to the timer, and the timer to the servo
     */
    timerToServoMap[timerIdx] = pServo;
    pServo->pTimer = pTimer;
    
    /* set some servo default calibration values, and limits
     */
    ServoCalibrate((ServoHandle_t)pServo, DEFAULT_USECS_PER_90,
                   DEFAULT_USECS_AT_ZERO,
                   DEFAULT_REVERSE_ANGLE);
    ServoSetLimits((ServoHandle_t)pServo, DEFAULT_USECS_MIN, DEFAULT_USECS_MAX);
    ServoSetMotionParameters((ServoHandle_t)pServo, DEFAULT_RATE);
    
    /* enable the timer peripheral
     */
    DbgPrintf(" configuring timer for servo\n", 0);
    ROM_SysCtlPeripheralEnable(pTimer->timerPeriph);
    
    /* now, timer needs to be configured.  Timer halves are used for PWM
     * generation. The TimerConfigure() driverlib function always configures
     * both halves at once, and also disables both halves.  So it is hard
     * to configure one half at a time this way.  Therefore, this code assumes
     * that both halves are always used for PWM and configures both halves for
     * PWM regardless of whether this is A or B half. It remembers the enabled
     * state for both halves and restores after the call to TimerConfigure().
     * That way if the other half was already configured and enabled, then it
     * will not be left disabled
     */
    uint32_t
    enabledState = HWREG(pTimer->timerBase + TIMER_O_CTL) & (TIMER_CTL_TAEN |
                                                             TIMER_CTL_TBEN);
    
    /* configure both halves for PWM, regardless of which half this is
     */
    ROM_TimerConfigure(pTimer->timerBase, TIMER_CFG_SPLIT_PAIR |
                       TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);
    
    /* restore the enabled state of the two timer halves
     */
    HWREG(pTimer->timerBase + TIMER_O_CTL) |= enabledState;
    
    /* set up timer for periodic rate, that was passed in Init()
     */
    ROM_TimerLoadSet(pTimer->timerBase, pTimer->timerHalf,
                     gTimerPeriodUsecs * gCyclesPerUsec);
    
    /* set match to be same as the reload value.  This should prevent the
     * output from going high at all, until a real match value is set.
     * Set the default initial position, as an assumption of the actual
     * location
     */
    pServo->currentUsecs = DEFAULT_INIT_USECS;
    ROM_TimerMatchSet(pTimer->timerBase, pTimer->timerHalf,
                      gTimerPeriodUsecs * gCyclesPerUsec);
    
    /*
     * Set the bit that controls match register update, to update on the
     * timeout and not immediately.  This ensures that changes to the
     * match register (that controls PWM pulse width) occurs synchronously
     * with the timer timeout.  There is no driverlib API for this.
     * The PLO bit should also be set which allows the output to go low
     * when match==period.
     */
    if(pTimer->timerHalf == TIMER_A)
    {
        HWREG(pTimer->timerBase + TIMER_O_TAMR) |= TIMER_TAMR_TAMRSU |
        TIMER_TAMR_TAPLO;
    }
    else
    {
        HWREG(pTimer->timerBase + TIMER_O_TBMR) |= TIMER_TBMR_TBMRSU |
        TIMER_TBMR_TBPLO;
    }
    
    /* set up the timer to allow interrupt generation on the rising edge
     * (at the start of a pulse).  updates are made synchronously (per above)
     * so any updates made after the rising edge will not take effect until
     * the next rising edge.  This allows the entire period for calculations
     * etc for updating for the next period
     */
    ROM_TimerControlEvent(pTimer->timerBase, pTimer->timerHalf,
                          TIMER_EVENT_POS_EDGE);
    
    /* enable the timer to allow it to start running, and enable the interrupt
     */
    ROM_TimerEnable(pTimer->timerBase, pTimer->timerHalf);
    ROM_TimerIntEnable(pTimer->timerBase, (pTimer->timerHalf == TIMER_A) ?
                       TIMER_CAPA_EVENT :
                       TIMER_CAPB_EVENT);
    
    /* If PD7 is used, it must be unlocked.  This is preconfigured as NMI
     * and cant be changed without a special procedure
     */
    if((pTimer->timerGpioBase == GPIO_PORTD_BASE) &&
       (pTimer->timerPin == GPIO_PIN_7))
    {
        DbgPrintf(" unlocking PD7\n", 0);
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x4C4F434B;
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = GPIO_PIN_7;
    }
    
    /* now that the timer output can be assured to be low, it is safe to
     * configure the GPIO pin for timer control.  Up to now it has been a
     * GPIO input (assuming no other code has changed it since reset)
     */
    DbgPrintf(" enabling timer output pin\n", 0);
    ROM_SysCtlPeripheralEnable(pTimer->timerGpioPeriph);
    ROM_GPIOPinConfigure(pTimer->timerPinConfig);
    ROM_GPIOPinTypeTimer(pTimer->timerGpioBase, pTimer->timerPin);
    
    /* If it was PD7 then relock
     */
    if((pTimer->timerGpioBase == GPIO_PORTD_BASE) &&
       (pTimer->timerPin == GPIO_PIN_7))
    {
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 1;
    }

    /* return the handle to the caller
     */
    DbgPrintf("-ServoConfig(), ret handle 0x%08X\n", pServo);
    return((ServoHandle_t)pServo);
}

/**
 * Enable servo operation.
 *
 * @param[in] hServo is the servo handle
 *
 * This function will begin generating the PWM pulse to the servo at the
 * present position.  If the servo is not already at that position it will
 * move there (without rate control).
 *
 * @returns Zero is returned for success, non-zero for the following reasons:
 *          input parameter is bad (-1)
 */
int
ServoEnable(ServoHandle_t hServo)
{
    Servo_t *pServo = (Servo_t *)hServo;
    
    /* validate inputs
     */
    if(hServo == NULL)
    {
        return(-1);
    }
    
    return(ServoSetPositionUsecs(hServo, pServo->currentUsecs));
}

/**
 * Disable servo operation.
 *
 * @param[in] hServo is the servo handle
 *
 * This function will disable the servo.  All callbacks will be deleted and
 * interrupts disabled, and the PWM output will be turned off.
 *
 * @returns Zero is returned for success, non-zero for the following reasons:
 *          input parameter is bad (-1); timer not initialized (-2)
 */
int
ServoDisable(ServoHandle_t hServo)
{
    Servo_t *pServo;
    const Timer_t *pTimer;
    
    DbgPrintf("+ServoDisable(hServo=0x%08X)\n", hServo);
    
    /* validate inputs
     */
    if(hServo == NULL)
    {
        DbgPrintf(" error: bad input parameter\n", 0);
        return(-1);
    }
    
    /* get pointers to servo and timer instance
     */
    pServo = (Servo_t *)hServo;
    pTimer = pServo->pTimer;
    
    /* make sure this servo has been initted with a timer
     */
    if(pTimer == NULL)
    {
        DbgPrintf(" error: timer not initted\n", 0);
        return(-1);
    }
    
    /* turn off any interrupts and remove any observers
     */
    ROM_IntDisable(pTimer->timerInt);
    pServo->pfnTickObserver = NULL;
    pServo->pfnMotionObserver = NULL;
    
    /* set the match value the same as the load register value, this will cause
     * the output to stop pulsing and go to a low state
     */
    ROM_TimerMatchSet(pTimer->timerBase, pTimer->timerHalf,
                      gTimerPeriodUsecs * gCyclesPerUsec);
    
    DbgPrintf("-ServoDisable(), ret(0)\n", 0);
    return(0);
}

/**
 * Set the calibration parameters for the servo.
 *
 * @param[in] hServo is the servo handle
 * @param[in] usecsPer90 is the number of microseconds of pulse witdth change
 *            that represents 90 degrees of motion
 * @param[in] usecsAtZero is the pulse width in microseconds when the servo is
 *            at the position designated as zero
 * @param[in] reverseAngle is a flag that means that angle calculations should
 *            be reversed (reverses the sign of motion)
 *
 * This function sets the parameters that are used for calibration of the
 * servo movement.  The values are dependent on the specific servos and the
 * physical arrangement.
 *
 * @returns Zero is returned if successful, non-zero if there is an error.
 * Reasons for error are: bad input parameter (-1).
 */
int
ServoCalibrate(ServoHandle_t hServo, int32_t usecsPer90, int32_t usecsAtZero,
               bool reverseAngle)
{
    Servo_t *pServo = (Servo_t *)hServo;
    
    /* validate inputs
     */
    if((hServo == NULL) || (usecsPer90 <= 0) || (usecsAtZero <= 0))
    {
        return(-1);
    }
    
    /* set the specified calibration values
     */
    pServo->usecsPer90 = usecsPer90;
    pServo->usecsAtZero = usecsAtZero;
    pServo->reverseAngle = reverseAngle;
    
    return(0);
}

/**
 * Set the limits for the servo.
 *
 * @param[in] hServo is the servo handle
 * @param[in] usecsMin is the minimum pulse width in microseconds
 * @param[in] usecsMax is the maximum pulse width in microseconds
 *
 * This function sets the limits that are used for determining the lower
 * and upper bound for the pulse width that can be applied to the servo
 * and represents the limits of range of motion.  The values are dependent
 * on the specific servos and the physical arrangement.
 *
 * @returns Zero is returned if successful, non-zero if there is an error.
 * Reasons for error are: bad input parameter (-1).
 */
int
ServoSetLimits(ServoHandle_t hServo, int32_t usecsMin, int32_t usecsMax)
{
    Servo_t *pServo = (Servo_t *)hServo;
    
    /* validate inputs
     */
    if((hServo == NULL) || (usecsMin <= 0) || (usecsMax <= 0))
    {
        return(-1);
    }
    
    /* set the specified limits
     */
    pServo->usecsMin = usecsMin;
    pServo->usecsMax = usecsMax;
    
    return(0);
}

/**
 * Set servo motion parameters.
 *
 * @param[in] hServo is the servo handle
 * @param[in] rateDegPerSec is the motion rate in degrees per second
 *
 * This function will set the motion rate as specified in degrees per
 * second.  Internally this will be silently constrained to a minimum of
 * 9 degrees per second.  This is the rate that the servo will move when
 * commanded using a Move() function.
 *
 * @returns Zero if successful, non-zero for the following reasons:
 *          bad input parameter (-1)
 */
int
ServoSetMotionParameters(ServoHandle_t hServo, int32_t rateDegPerSec)
{
    int32_t usecs;
    Servo_t *pServo = (Servo_t *)hServo;
    
    DbgPrintf("+ServoSetMotionParameters(hServo=0x%08X, rateDegPerSec=%d\n",
              hServo, rateDegPerSec);
    
    /* validate input handle
     */
    if(hServo == NULL)
    {
        DbgPrintf(" error: bad input parameter\n", 0);
        return(-1);
    }
    
    /* convert the deg part of the rate to usecs(angle)
     */
    usecs = CalcUsecsFromCentiDegreesRel(pServo, rateDegPerSec * 10);
    
    /* convert the rest so we end up with usecs(angle) per timer tick
     */
    usecs *= gTimerPeriodUsecs;
    usecs /= 1000000;
    
    /* make sure it can not be less than 1, which gives 9 deg/sec
     */
    usecs = (usecs < 1) ? 1 : usecs;
    DbgPrintf(" computed rate: %d usecs(angle) per timer tick\n", usecs);
    
    /* save the computed rate
     */
    pServo->rateUsecsPerTick = usecs;
    
    DbgPrintf("-ServoSetMotionParameters(), ret(0)\n", 0);
    return(0);
}

/**
 * Set the servo position using the pulse width in microseconds.
 *
 * @param[in] hServo is the servo handle
 * @param[in] usecs is the pulse width in microseconds
 *
 * This function will set the servo PWM pulse width which represents a
 * position.  If the usecs value is outside the min and max defined for
 * the servo, then the value will be constrained to the min or max with no
 * error indicated.
 *
 * @returns Zero is returned upon success, non-zero for the following reasons:
 *               input parameter is bad (-1); timer not initialized (-2)
 */
int
ServoSetPositionUsecs(ServoHandle_t hServo, int32_t usecs)
{
    Servo_t *pServo;
    const Timer_t *pTimer;
    
    DbgPrintf("+ServoSetPositionUsecs(hServo=0x%08X, usecs=%d)\n", hServo, usecs);
    
    /* validate input handle
     */
    if((hServo == NULL) || (usecs <= 0))
    {
        DbgPrintf(" error: bad input parameter\n", 0);
        return(-1);
    }
    
    /* get servo and timer instance pointers
     */
    pServo = (Servo_t *)hServo;
    pTimer = pServo->pTimer;
    
    /* make sure this servo has been initted with a timer
     */
    if(pTimer == NULL)
    {
        DbgPrintf(" error: timer not initted\n", 0);
        return(-2);
    }
    
    /* constrain the setting to the min and max allowable
     */
    usecs = (usecs > pServo->usecsMax) ? pServo->usecsMax : usecs;
    usecs = (usecs < pServo->usecsMin) ? pServo->usecsMin : usecs;
    DbgPrintf(" constrained to %d\n", usecs);
    
    /* set the output pulse width to the specified usecs
     */
    pServo->currentUsecs = usecs;
    ROM_TimerMatchSet(pTimer->timerBase, pTimer->timerHalf,
                      (gTimerPeriodUsecs - usecs) * gCyclesPerUsec);
    
    DbgPrintf("-ServoSetPositionUsecs(), ret (0)\n", 0);
    return(0);
}

/**
 * Set the servo position using the pulse width in centi-degrees.
 *
 * @param[in] hServo is the servo handle
 * @param[in] centiDegrees is the angle in centi-degrees
 *
 * This function will set the servo position as specified in centi-degrees.
 * The position will be constrained per the default or prior set limits.
 *
 * @returns Zero is returned upon success, non-zero for the following reasons:
 *               input parameter is bad (-1); timer not initialized (-2)
 */
int
ServoSetPosition(ServoHandle_t hServo, int32_t centiDegrees)
{
    int32_t usecs;
    int ret;
    
    DbgPrintf("+ServoSetPosition(hServo=0x%08X, centiDegrees=%d)\n", hServo,
              centiDegrees);
    
    /* validate input handle
     */
    if((hServo == NULL))
    {
        DbgPrintf(" error: bad input parameter\n", 0);
        return(-1);
    }
    
    /* convert the specified angle to microseconds of pulse width
     */
    usecs = CalcUsecsFromCentiDegreesAbs((Servo_t *)hServo, centiDegrees);
    
    /* set the new position
     */
    ret = ServoSetPositionUsecs(hServo, usecs);
    
    DbgPrintf("-ServoSetPosition(), ret (%d)\n", ret);
    return(ret);
}

/**
 * Get the position of the servo in microseconds of pulse width
 *
 * @param[in] hServo is the servo handle
 *
 * This function returns the current, absolute position of the servo in
 * units of microseconds of pulse width.
 *
 * @returns The pulse with in microseconds or 0 if there is an error.
 */
int32_t
ServoGetPositionUsecs(ServoHandle_t hServo)
{
    Servo_t *pServo;
    
    DbgPrintf("+ServoGetPositionUsecs(hServo=0x%08X)\n", hServo);
    
    /* validate input handle
     */
    if((hServo == NULL))
    {
        DbgPrintf(" error: bad input parameter\n", 0);
        return(0);
    }
    
    /* get servo instance pointers
     */
    pServo = (Servo_t *)hServo;

    DbgPrintf("-ServoGetPositionUsecs(), ret (%d)\n", pServo->currentUsecs);
    return(pServo->currentUsecs);
}

/**
 * Get the position of the servo in centi-degrees
 *
 * @param[in] hServo is the servo handle
 *
 * This function returns the current, absolute position of the servo in
 * units of centi-degrees.
 *
 * @returns The angle in centi-degrees or 0 if there is an error.
 *          Zero does not necessarily indicate an error because it is also
 *          a legitimate value for the angle.
 */
int32_t
ServoGetPosition(ServoHandle_t hServo)
{
    Servo_t *pServo;
    int32_t cdeg;
    
    DbgPrintf("+ServoGetPosition(hServo=0x%08X)\n", hServo);
    
    /* validate input handle
     */
    if((hServo == NULL))
    {
        DbgPrintf(" error: bad input parameter\n", 0);
        return(0);
    }
    
    /* get servo instance pointers
     */
    pServo = (Servo_t *)hServo;
    
    /* convert position in usecs to centideg
     */
    cdeg = CalcCentiDegreesFromUsecsAbs(hServo, pServo->currentUsecs);
    
    DbgPrintf("-ServoGetPosition(), ret (%d)\n", cdeg);
    return(cdeg);
}

/**
 * Move to position in microseconds pulse width, with motion control
 *
 * @param[in] hServo is the servo handle
 * @param[in] usecs is the position in microseconds of pulse width
 * @param[in] pfnMotionObserver is a callback function, called when motion
 *            is complete
 * @param[in] pObserverData is caller supplied data pointer to be passed
 *            to the callback function
 *
 * This function will move the servo to a new position.  The position is
 * specified as pulse width in microseconds.  The servo is moved with
 * motion control, which means that the servo is not immediately commanded
 * to the new position but is moved gradually according to the motion
 * parameters.  This allows for smoother, more controlled motion.  This starts
 * the motion and returns right away, before the motion is complete.
 * If the called supplied a callback function, then it will be called when
 * the motion is complete.
 *
 * @note The callback function (pfnMotionObserver) will be called in
 * interrupt context, so it must be short and non-blocking.
 *
 * @returns Zero is returned if successful, non-zero for the following reasons:
 *          servo handle is bad (-1); or any error returned by
 *          ServoSetTickObserver().
 */
int
ServoMoveUsecs(ServoHandle_t hServo, int32_t usecs,
               void (*pfnMotionObserver)(void *pObserverData),
               void *pObserverData)
{
    int ret;
    Servo_t *pServo = (Servo_t *)hServo;
    
    DbgPrintf("+ServoMoveUsecs(hServo=0x%08X, usecs=%d, pfnObserver=0x%08X, pData=0x%08X\n",
              hServo, usecs, pfnMotionObserver, pObserverData);
    
    /* validate input handle
     */
    if(hServo == NULL)
    {
        DbgPrintf(" error: NULL handle\n", 0);
        return(-1);
    }
    
    /* Stop any ongoing motion, or any other notification and interrupts
     * for the timer
     */
    ret = ServoSetTickObserver(hServo, NULL, NULL);
    if(ret)
    {
        return(ret);
    }
    
    /* constrain to limits
     */
    usecs = (usecs > pServo->usecsMax) ? pServo->usecsMax : usecs;
    usecs = (usecs < pServo->usecsMin) ? pServo->usecsMin : usecs;
    DbgPrintf(" constrained to %d\n", usecs);

    /* save the destination angle
     */
    pServo->targetUsecs = usecs;
    
    /* set up the motion callback, if any was specified
     */
    pServo->pfnMotionObserver = pfnMotionObserver;
    pServo->pMotionObserverData = pObserverData;
    pServo->motionComplete = false;
    
    /* enable the motion handler, this will start the servo motion
     */
    ret = ServoSetTickObserver(hServo, ServoMotionHandler, pServo);
    
    DbgPrintf("-ServoMoveUsecs(), ret (%d)\n", ret);
    return(ret);
}

/**
 * Move to position in centi-degrees, with motion control
 *
 * @param[in] hServo is the servo handle
 * @param[in] centiDegrees is the position in centi-degrees
 * @param[in] pfnMotionObserver is a callback function, called when motion
 *            is complete
 * @param[in] pObserverData is caller supplied data pointer to be passed
 *            to the callback function
 *
 * This function will move the servo to a new position.  The position is
 * specified as centi-degrees.  The servo is moved with motion control, which
 * means that the servo is not immediately commanded to the new position but
 * is moved gradually according to the motion parameters.  This allows for
 * smoother, more controlled motion.  This starts the motion and returns right
 * away, before the motion is complete.
 * If the called supplied a callback function, then it will be called when
 * the motion is complete.
 *
 * @note The callback function (pfnMotionObserver) will be called in
 * interrupt context, so it must be short and non-blocking.
 *
 * @returns Zero is returned if successful, non-zero for the following reasons:
 *          servo handle is bad (-1); or any error returned by
 *          ServoSetTickObserver().
 */
int
ServoMove(ServoHandle_t hServo, int32_t centiDegrees,
          void (*pfnMotionObserver)(void *pObserverData),
          void *pObserverData)
{
    int ret;
    int usecs;
    Servo_t *pServo = (Servo_t *)hServo;
    
    DbgPrintf("+ServoMove(hServo=0x%08X, centiDegrees=%d, pfnObserver=0x%08X, pData=0x%08X\n",
              hServo, centiDegrees, pfnMotionObserver, pObserverData);
    
    /* validate input handle
     */
    if(hServo == NULL)
    {
        DbgPrintf(" error: NULL handle\n", 0);
        return(-1);
    }

    /* convert the input angle to microseconds
     */
    usecs = CalcUsecsFromCentiDegreesAbs(pServo, centiDegrees);
    
    /* call the move function
     */
    ret = ServoMoveUsecs(hServo, usecs, pfnMotionObserver, pObserverData);
    
    DbgPrintf("-ServoMove(), ret (%d)\n", ret);
    return(ret);
}

/**
 * Set a callback function on the PWM timer tick.
 *
 * @param[in] hServo is the servo handle
 * @param[in] pfnServoTickObserver is the callback function for timer ticks
 * @param[in] pObserverData is the data to pass to the callback
 *
 * This function will install a callback function that will be called once
 * for each timer PWM period.  Then the timer interrupts will be enabled so
 * the callback should start getting called immediately.  If NULL is passed
 * as the callback function, then the callback is removed and interrupts
 * disabled.
 *
 * @note The callback will be called in interrupt context, so it must be
 *       short and non-blocking.
 *
 * @returns Zero if successful, non-zero for the following reasons:
 *          bad handle (-1); timer not initialized (-2)
 */
int
ServoSetTickObserver(ServoHandle_t hServo,
                     void (*pfnServoTickObserver)(void *pObserverData),
                     void *pObserverData)
{
    Servo_t *pServo;
    const Timer_t *pTimer;
    
    DbgPrintf("+ServoSetTickObserver(hServo=0x%08X, pfnObserver=0x%08X, pData=0x%08X\n",
              hServo, pfnServoTickObserver, pObserverData);
    
    /* validate input handle
     */
    if(hServo == NULL)
    {
        DbgPrintf(" error: bad servo handle\n", 0);
        return(-1);
    }
    
    /* get instance handles
     */
    pServo = (Servo_t *)hServo;
    pTimer = pServo->pTimer;
    
    /* make sure this servo has been initted with a timer
     */
    if(pTimer == NULL)
    {
        DbgPrintf(" error: timer not initted\n", 0);
        return(-2);
    }
    
    /* if a function is provided, then install it as the callback
     * and then enable interrupts, this should allow the timer to start
     * generating interrupts and calling the callback
     */
    if(pfnServoTickObserver)
    {
        pServo->pfnTickObserver = pfnServoTickObserver;
        pServo->pTickObserverData = pObserverData;
        ROM_IntEnable(pServo->pTimer->timerInt);
    }
    
    /* otherwise the callback is NULL which means to remove the
     * callback and disable the interrupt
     */
    else
    {
        ROM_IntDisable(pServo->pTimer->timerInt);
        pServo->pfnTickObserver = NULL;
    }
    
    DbgPrintf("-ServoSetTickObserver(), ret (0)\n", 0);
    return(0);
}

/* @internal
 *
 * Following are the specific timer interrupt handlers, which just pass
 * control to the general wide timer handler (above).
 */

#define CALL_HANDLER(t,h) WTimerIntHandler(((t) * 2) + (h))
#define WTMR_HANDLER(t,h) void WTimer ## t ## h ## IntHandler(void){CALL_HANDLER(t, SERVO_TIMER_##h);}

/* To work with macro above, do not use semicolons
 */
WTMR_HANDLER(0, A)
WTMR_HANDLER(0, B)
WTMR_HANDLER(1, A)
WTMR_HANDLER(1, B)
WTMR_HANDLER(2, A)
WTMR_HANDLER(2, B)
WTMR_HANDLER(3, A)
WTMR_HANDLER(3, B)
WTMR_HANDLER(4, A)
WTMR_HANDLER(4, B)
WTMR_HANDLER(5, A)
WTMR_HANDLER(5, B)

/**
 * @}
 */
