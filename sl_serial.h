//
// Created by Tom Dilatush on 10/22/17.
//

#ifndef GPSCTL_SL_SERIAL_H
#define GPSCTL_SL_SERIAL_H

#include <stdbool.h>
#include "sl_return.h"

typedef bool baudRateSynchronizer( const char, void ** );

typedef struct speedInfo {
    int baudRate;  // baud rate (in bits/second)
    int nsBit;     // number of nanoseconds to receive or transmit a bit
    int nsChar;    // number of nanoseconds to receive or transmit a character
} speedInfo;

int getBaudRateCookie( int baudRate );
bool ASCIIBaudRateSynchronizer( const char c, void **state);

slReturn getSpeedInfo( int fdPort, speedInfo* );
slReturn verifySerialDevice( const char *deviceName );
slReturn readSerialChar( int fdPort, long long msTimeout );
slReturn getBaudRate( int cookie );
slReturn synchronize( int fdPort, baudRateSynchronizer, int );
slReturn setTermOptions( int fdPort, int baud, int dataBits, int stopBits, bool parityEnable, bool odd );
slReturn setTermOptionsBaud( int fdPort, int baud );

int setBaudRate( int fdPort, int baudRate, bool synchronize, bool autoRate,
                 baudRateSynchronizer synchronizer, bool verbose );

#endif //GPSCTL_SL_SERIAL_H
