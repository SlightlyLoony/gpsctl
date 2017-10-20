//
// Created by Tom Dilatush on 10/19/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#ifdef Macintosh
#define _DARWIN_C_SOURCE
#endif /* Macintosh */
#define _POSIX_C_SOURCE 199309L

#include <memory.h>
#include <stdlib.h>
#include <execinfo.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include "slightly_loony.h"

// Dumps a stack trace to stderr.
extern void stackDump( void ) {
    #define STACK_DUMP_SIZE 250
    void *stackFrames[STACK_DUMP_SIZE];
    char **strings;

    int numFrames = backtrace( stackFrames, STACK_DUMP_SIZE );
    backtrace_symbols_fd( stackFrames, numFrames, STDERR_FILENO );
}


// A safe replacement for malloc that checks the result.  On failure, prints an error message and stack trace
// to stderr.  Otherwise, it behaves EXACTLY like malloc().
extern void *safeMalloc( size_t size ) {
    void *result = malloc( size );
    if( !result ) {
        fprintf( stderr, "ERROR: malloc() failed\n" );
        stackDump();
        exit(1);
    }
    return result;
}


// Return the concatenation of the two given string in a newly allocated memory on the heap.  If the two
// given strings are null, a null is returned.  If one of the given strings is null, then a COPY (in newly allocated
// memory) of the non-null given string is returned.  Note that this behavior means that ANY non-null return value
// represents a block of memory that must be freed by the caller.  If malloc fails, a stack trace is dumped to stderr
// and the program is aborted.
extern char* concat( const char *s1, const char *s2 ) {

    // get the lengths just once (optimization)...
    const size_t len1 = s1 ? strlen(s1) : 0;
    const size_t len2 = s2 ? strlen(s2) : 0;

    // if both arguments were null, return with a null...
    if( len1 + len2 == 0 ) return NULL;

    // get the memory required for the concatenated strings plus the terminator...
    char *result = safeMalloc( len1 + len2 + 1 );

    // build our result...
    memcpy(result, s1, len1);
    memcpy(result + len1, s2, len2 + 1 ); //+1 to copy the null-terminator

    return result;
}


// Tests whether the given device name is non-null, exists, is a device, is a character device, and is a terminal.
// Returns one of the following values:
//   VSD_NULL          // if the given device name is null
//   VSD_NONEXISTENT   // if the given device name does not exist (or is not accessible) in the file system
//   VSD_NOT_DEVICE    // if the given device name is not a device (but exists in the file system)
//   VSD_NOT_CHARACTER // if the given device name is not a character device (but is a device)
//   VSD_NOT_TERMINAL  // if the given device name is not a terminal (but is a character device)
//   VSD_CANT_OPEN     // if the given device name can't be opened (but is a terminal)
//   VSD_IS_SERIAL     // if the given device name is a serial device (exists, is a character device, and is a terminal)
extern int verifySerialDevice( const char *deviceName ) {

    if( !deviceName ) return VSD_NULL;

    struct stat stats;
    int r = stat( deviceName, &stats );
    if( r ) return VSD_NONEXISTENT;
    if( ((stats.st_mode & S_IFMT) != S_IFBLK) && ((stats.st_mode & S_IFMT) != S_IFCHR) ) return VSD_NOT_DEVICE;
    if( (stats.st_mode & S_IFMT) != S_IFCHR ) return VSD_NOT_CHARACTER;

    int fd = open( deviceName, O_RDWR | O_NOCTTY | O_NONBLOCK );
    if( fd < 0 ) return VSD_CANT_OPEN;

    struct termios termios;
    r = tcgetattr( fd, &termios );
    if( r ) return VSD_NOT_TERMINAL;

    close( fd );

    return VSD_IS_SERIAL;
}


// Translate the given integer baud rate into the constant needed by termios to specify that baud rate.  If the given
// baud rate can't be translated, it is not a valid baud rate value.  If the give baud rate is valid, the termios
// constant is returned.  Otherwise, -1 is returned.
extern int getBaudRateCookie( int baudRate ) {

    struct baudTrans { int baud; int cookie; };
    struct baudTrans validRates[] = {
            {50, B50}, {75, B75}, {110, B110}, {134, B134}, {150, B150}, {200, B200}, {300, B300}, {600, B600},
            {1200, B1200}, {1800, B1800}, {2400, B2400}, {4800, B4800}, {9600, B9600}, {19200, B19200},
            {38400, B38400}, {57600, B57600}, {115200, B115200}, {230400, B230400}
    };

    for( int i = 0; i < ARRAY_SIZE(validRates); i++ ) {
        if( baudRate == validRates[i].baud )
            return validRates[i].cookie;
    }
    return -1;
}


// Changes the terminal options on the given serial port to the given baud rate, number of data bits, number of stop
// bits, parity enable, and parity polarity (true for odd, false for even).  The given arguments are first checked for
// validity.  If there is a problem, then the appropriate error code is returned.  If there were no errors, then
// zero is returned.
extern int setTermOptions( int fd, int baud, int dataBits, int stopBits, bool parityEnable, bool parityOdd ) {

    if( fd < 0 ) return STO_INVALID_FILE_DESCRIPTOR;
    int baudCookie = getBaudRateCookie( baud );
    if( baudCookie < 0 ) return STO_INVALID_BAUD_RATE;
    int dataBitCookie;
    switch( dataBits ) {
        case 5: dataBitCookie = CS5; break;
        case 6: dataBitCookie = CS6; break;
        case 7: dataBitCookie = CS7; break;
        case 8: dataBitCookie = CS8; break;
        default: return STO_INVALID_NUMBER_OF_DATA_BITS;
    }
    int stopBitCookie;
    if( stopBits == 1 ) stopBitCookie = 0;
    else if( stopBits == 2 ) stopBitCookie = CSTOPB; else return STO_INVALID_NUMBER_OF_STOP_BITS;
    int parityCookie = (parityEnable ? PARENB | (parityOdd ? PARODD : 0) : 0);

    struct termios options;
    tcgetattr( fd, &options );
    options.c_cflag = (tcflag_t) (baudCookie | dataBitCookie | stopBitCookie | parityCookie | CLOCAL | CREAD);
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush( fd, TCIFLUSH );
    tcsetattr( fd, TCSANOW, &options );
    return STO_OK;
}


// Sets the baud rate on the given serial port.