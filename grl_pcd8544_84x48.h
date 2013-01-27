/******************************************************************************
 *
 * grl_pcd8544_84x48.h - TI Stellaris graphics library display driver for
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

#ifndef grl_pcd8544_84x48_h
#define grl_pcd8544_84x48_h

/* Function prototypes
 */
extern const tDisplay g_sPCD8544_84x48;
extern void GRL_PCD8544_84x48_Init(unsigned long ulSysClock);
extern void GRL_PCD8544_84x48_Test(void);

#endif
