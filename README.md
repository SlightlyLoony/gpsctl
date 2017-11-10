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
Well, probably the world doesn't actually need _gpsctl_ - but if you happen to be attempting the same
thing the author was, you might find it useful.  Configuring and controlling a U-Blox GPS connected to a 
Linux host is a frustrating exercise.  The author ran into this when implementing a 
[stratum 1 NTP server](https://www.endruntechnologies.com/stratum1.htm) using the aforementioned 
hardware.  He needed to make some configuration changes to better support this.  Those configuration 
changes required the use of the UBX protocol, and support for this on Linux is ... minimal.  Hence 
_gpsctl_!  

On the other hand, anyone trying to "talk" to a U-Blox GPS from a C program might find the
code in _gpsctl_ to be a useful building block.  The code in _gpsctl_ includes functions for sending and
receiving UBX protocol messages, for interpreting them, modifying them, etc.  This could easily be extended to 
serve other purposes; after all, not everyone lusts for a stratum 1 NTP server, and there might actually be other
useful purposes for a GPS (though I can't think of one right now).

##What, exactly, does _gpsctl_ do?
Some _gpsctl_ functions work for any GPS streaming NMEA data over a serial port:
* Echoes NMEA data to stdout, optionally filtered by NMEA message type.
* Infers the GPS' baud rate from the stream of NMEA data.

Other _gpsctl_ functions are specific to U-Blox GPS hardware:
* Configures the baud rate that the U-Blox GPS uses for communication.
* Configures whether the U-Blox GPS transmits NMEA data.
* Infers the GPS' baud rate using the UBX protocol only.
* Queries the U-Blox GPS for position, time, GPS version, GPS configuration, and GPS satellite information.
  The format of query results are selectable: either plain English or JSON.
* Tests the pulse-per-second output of the U-Blox GPS (fundamental for NTP).
* Configures the U-Blox GPS for maximum timing pulse accuracy (useful for building a stratum 1 NTP server 
based on the GPS' notion of time).
* Saves the U-Blox GPS configuration to on-module battery-backed memory.
* Resets the U-Blox GPS.

##Dependencies
The only dependencies _gpsctl_ has (other than the standard C library) is on the JSON library 
[cJSON](https://github.com/DaveGamble/cJSON) and the popular Raspberry Pi I/O library 
[WiringPi](http://wiringpi.com/).  The source for cJSON (which has an MIT license) is incorporated in this project.  
The WiringPi library must be present and linked.

##Why is _gpsctl_'s code so awful?
Mainly because this is the first C program the author has written in over 30 years, but also because the author
has serious deficiencies in aptitude, intelligence, and knowledge (not to mention choices of hobbies).

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
