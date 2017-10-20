//
// Created by Tom Dilatush on 10/19/17.
//

#ifndef GPSCTL_SLIGHTLY_LOONY_H
#define GPSCTL_SLIGHTLY_LOONY_H

#endif //GPSCTL_SLIGHTLY_LOONY_H

#define ARRAY_SIZE(x) (sizeof (x) / sizeof *(x))

void stackDump( void );

void *safeMalloc( size_t size );

char* concat( const char *s1, const char *s2 );

#define VSD_CANT_OPEN     6
#define VSD_NULL          5
#define VSD_NONEXISTENT   4
#define VSD_NOT_DEVICE    3
#define VSD_NOT_CHARACTER 2
#define VSD_NOT_TERMINAL  1
#define VSD_IS_SERIAL     0

int verifySerialDevice( const char *deviceName );

int getBaudRateCookie( int baudRate );

#define STO_INVALID_BAUD_RATE 1
#define STO_OK 0
#define STO_INVALID_FILE_DESCRIPTOR 2
#define STO_INVALID_NUMBER_OF_DATA_BITS 3
#define STO_INVALID_NUMBER_OF_STOP_BITS 4
int setTermOptions( int fd, int baud, int dataBits, int stopBits, bool parityEnable, bool odd );