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
speedInfo getSpeedInfo( int fdPort );

slReturn verifySerialDevice( const char *deviceName );

#define RSC_READ_ERROR (-1)
#define RSC_TIMED_OUT  (-2)
int readSerialChar( int fdPort, long long msTimeout );

int getBaudRate( int cookie );
int getBaudRateCookie( int baudRate );
bool ASCIIBaudRateSynchronizer( const char c, void **state);
bool synchronize( int fdPort, baudRateSynchronizer, int );

slReturn setTermOptions( int fdPort, int baud, int dataBits, int stopBits, bool parityEnable, bool odd );

slReturn setTermOptionsBaud( int fdPort, int baud );

int setBaudRate( int fdPort, int baudRate, bool synchronize, bool autoRate,
                 baudRateSynchronizer synchronizer, bool verbose );

#endif //GPSCTL_SL_SERIAL_H
