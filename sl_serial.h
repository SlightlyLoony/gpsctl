//
// Created by Tom Dilatush on 10/22/17.
//

#ifndef GPSCTL_SL_SERIAL_H
#define GPSCTL_SL_SERIAL_H

#include <stdbool.h>

typedef bool baudRateSynchronizer( const char, void ** );

typedef struct speedInfo {
    int baudRate;  // baud rate (in bits/second)
    int nsBit;     // number of nanoseconds to receive or transmit a bit
    int nsChar;    // number of nanoseconds to receive or transmit a character
} speedInfo;
speedInfo getSpeedInfo( int fdPort );

#define VSD_CANT_OPEN     6
#define VSD_NULL          5
#define VSD_NONEXISTENT   4
#define VSD_NOT_DEVICE    3
#define VSD_NOT_CHARACTER 2
#define VSD_NOT_TERMINAL  1
#define VSD_IS_SERIAL     0

int verifySerialDevice( const char *deviceName );

#define RSC_READ_ERROR (-1)
#define RSC_TIMED_OUT  (-2)
int readSerialChar( int fdPort, long long msTimeout );

int getBaudRate( int cookie );
int getBaudRateCookie( int baudRate );
bool ASCIIBaudRateSynchronizer( const char c, void **state);
bool synchronize( int fdPort, baudRateSynchronizer, int );

typedef enum { stoOK, stoInvalidBaudRate, stoInvalidFileDescriptor, stoInvalidNumberOfDataBits, stoInvalidNumberOfStopBits } stoResult;
stoResult setTermOptions( int fdPort, int baud, int dataBits, int stopBits, bool parityEnable, bool odd );

stoResult setTermOptionsBaud( int fdPort, int baud );
char* stoResultStr( stoResult result );

#define SBR_SET_SYNC        0
#define SBR_SET_NOSYNC      1
#define SBR_INVALID         2
#define SBR_FAILED_SYNC     3
#define SBR_PORT_ERROR      4
int setBaudRate( int fdPort, int baudRate, bool synchronize, bool autoRate,
                 baudRateSynchronizer synchronizer, bool verbose );

#endif //GPSCTL_SL_SERIAL_H
