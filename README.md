Summary
=======

Miscellaneous drivers for TI Stellaris&reg; and/or Tiva microcontrollers
======================================================
by Joe Kroesche (kroesche.org)

---

License
-------
Copyright &copy; 2013-2014, Joseph Kroesche (kroesche.org).  All rights reserved.
This software is released under the FreeBSD license, found in the accompanying file LICENSE.txt and at the following URL:
  http://www.freebsd.org/copyright/freebsd-license.html
This software is provided as-is and without warranty.

**The author and this software is not affiliated with Texas Instruments.  This is not a product of Texas Instruments, and TI is not responsible for this software.  Please do not ask TI for support with this software.  The Stellaris trademark is owned by TI.**

Documentation
=============
The Doxygen-generated documentation can be found at http://kroesche.github.io/stellaris_drivers/

Device Compatibility
====================
At the time of this writing and to the best of my knowledge, Stellaris and Tiva microcontollers are the same thing at least in regard to peripheral compatibility.  Tiva is just a rebranding of the Stellaris line.  Therefore, any driver that works on Stellaris should work on Tiva and so on, assuming the microcontroller has the same peripherals.  For example, the SPI on a Stellaris should be the same as the SPI on a Tiva etc.  So even though I may specifically refer to Stellaris or Tiva in the documentation, generally they are interchangeable unless I state otherwise.  This applies up to the Tiva part TM4C123.  For newer Tiva parts I have no personal knowledge of the compatibility although I will be surprised if they are not fully or mostly compatible.

Drivers
=======
This is a collection of drivers for Stellaris/Tiva microcontrollers:

grl_pcd8544_84x48
-----------------
This is a driver for the Philips PCD8544 display controller, which is used in the commonly available Nokia 5110/3310 displays.  These displays are very inexpensive and can be found at adafruit and sparkfun.  They are easy to connect and use.  This driver is specifically meant to be used with the StellarisWare graphics library.  This driver uses a SPI port and some GPIOs.

servo-wt (Servo Driver for Wide Timers)
--------
This is a driver that uses Stellaris (or Tiva) LM4F wide timers as servo controllers.  Up to 12 servos are supported.  Though not required, this driver was written to be used with the [12-servo BoosterPack](http://tronics.kroesche.org/servo-boosterpack.html).  See also https://github.com/kroesche/lp-servo12

lpd8806_ledstrip (Driver for RGB LED strip)
----------------
This is a driver that is meant to control an RGB LED strip that is based on the LPD8806 controller.  These kind of LED lighting strips are commonly available from places like Adafruit and Sparkfun.  It uses a SPI peripheral from the microcontroller.