<h1 align="center"><b>gpsctl 0.5</b></h1>

##What is gpsctl?
_gpsctl_ is a utility program written for Raspberry Pi computers using a U-Blox GPS board.  The 
author's rig is a 
[Raspberry Pi 3 Model B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) with a 
[Uputronix GPS board](https://store.uputronics.com/index.php?route=product/product&product_id=81),
which uses a [U-Blox MAX-M8Q](https://www.u-blox.com/en/product/max-m8-series) GPS module.  Some 
of _gpsctl_'s functions are generic for any GPS that streams NMEA data to a Linux serial port, and 
these should work on any Linux system.  Most of _gpsctl_'s functions, however, are quite specific
to the U-Blox products, and use their proprietary UBX protocol (again, on a serial port).

##Why does the world need gpsctl?
Configuring and controlling a U-Blox GPS connected to a Linux host is a frustrating exercise.  The
author ran into this when implementing a 
[stratum 1 NTP server](https://www.endruntechnologies.com/stratum1.htm) using the aforementioned 
hardware.  He needed to make some configuration changes to better support this.  Those configuration 
changes required the use of the UBX protocol, and support for this on Linux is ... minimal.  Hence 
gpsctl!

##What, exactly, does _gpsctl_ do?
Some _gpsctl_ functions work for any GPS streaming NMEA data over a serial port:
* Echoes NMEA data to stdout, optionally filtered by NMEA message type.
* Infers the GPS' baud rate from the stream of NMEA data.

Other _gpsctl_ functions are specific to U-Blox GPS hardware:
* Configures the baud rate that the U-Blox GPS uses for communication.
* Configures whether the U-Blox GPS transmits NMEA data.
* Infers the GPS' baud rate using the UBX protocol only.
* Queries the U-Blox GPS for position, time, version, and more.
* Tests the pulse-per-second output of the U-Blox GPS (fundamental for NTP).
* Configures the U-Blox GPS for stationary or portable (moving) mode.
* Saves the U-Blox GPS configuration to on-module flash memory.

##Why is _gpsctl_'s code so awful?
Mainly because this is the first C program the author has written in over 30 years!
##How is _gpsctl_ licensed?
_gpsctl_ is licensed with the quite permissive MIT license:
> Created: October 18, 2017
> Author: Tom Dilatush <tom@dilatush.com>  
> Github:  
> License: MIT
> 
> Copyright 2017 Tom Dilatush
> 
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
> documentation files (the "Software"), to deal in the Software without restriction, including without limitation
> the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
> to permit persons to whom the Software is furnished to do so.
> 
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of
> the Software.
> 
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
> THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE A
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
> TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
> SOFTWARE.
