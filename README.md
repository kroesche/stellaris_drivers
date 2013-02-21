Summary
=======

Miscellaneous drivers for TI Stellaris microcontrollers
======================================================
by Joe Kroesche (kroesche.org)

---

License
-------
Copyright (c) 2013, Joseph Kroesche (kroesche.org).  All rights reserved.
This software is released under the FreeBSD license, found in the accompanying file LICENSE.txt and at the following URL:
  http://www.freebsd.org/copyright/freebsd-license.html
This software is provided as-is and without warranty.

Drivers
=======
This is a collection of drivers for Stellaris microcontrollers:

grl_pcd8544_84x48
-----------------
This is a driver for the Philips PCD8544 display controller, which is used in the commonly available Nokia 5110/3310 displays.  These displays are very inexpensive and can be found at adafruit and sparkfun.  They are easy to connect and use.  This driver is specifically meant to be used with the StellarisWare graphics library.

servo
-----
This is a driver that uses Stellaris LM4F wide timers as servo controllers.  Up to 12 servos are supported.

