//
// Created by Tom Dilatush on 10/22/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
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
#include <errno.h>
#include "sl_general.h"
#include "sl_serial.h"
#include "sl_bits.h"
#include "sl_return.h"


// Returns speed information for the given port.
extern slReturn getSpeedInfo( int fdPort, speedInfo* result ) {
    struct termios options;
    tcgetattr( fdPort, &options );
    slReturn brResp = getBaudRate((int) cfgetispeed( &options ));
    if( isErrorReturn( brResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( brResp ), "could not get baud rate" );
    result->baudRate =  getReturnInfoInt32( brResp );
    result->nsBit = 1000000000 / result->baudRate;
    int stopBits = (options.c_cflag & CSTOPB) ? 2 : 1;
    int dataBits = (int) getBitField_slBits( options.c_cflag, CSIZE );
    int bits = stopBits + dataBits;
    result->nsChar = bits * result->nsBit;
    return makeOkReturn();
}


// Tests whether the given device name is non-null, exists, is a device, is a character device, and is a terminal.
extern slReturn verifySerialDevice( const char* deviceName ) {

    if( !deviceName )
        return makeErrorMsgReturn( ERR_ROOT, "attempted to verify a NULL device name" );

    struct stat stats;
    int r = stat( deviceName, &stats );

    if( r != 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "stat() returned error on \"%s\": %s", deviceName, strerror(errno) );

    if( ((stats.st_mode & S_IFMT) != S_IFBLK) && ((stats.st_mode & S_IFMT) != S_IFCHR) )
        return makeErrorFmtMsgReturn( ERR_ROOT, "\"%s\" is not a device", deviceName );

    if( (stats.st_mode & S_IFMT) != S_IFCHR )
        return makeErrorFmtMsgReturn( ERR_ROOT, "\"%s\" is not a character mode device", deviceName );;

    int fd = open( deviceName, O_RDWR | O_NOCTTY | O_NONBLOCK );
    if( fd < 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "can't open \"%s\": %s", deviceName, strerror(errno) );

    struct termios termios;
    r = tcgetattr( fd, &termios );
    if( r != 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "can't retrieve terminal attributes for \"%s\": %s", deviceName,
                                     strerror(errno) );

    close( fd );

    return makeOkReturn();
}


// Reads a single character from the given serial port, waiting at most the given number of milliseconds.  On an Ok
// response, returns the character read as a char in additional info.  Returns a warning if it timed out.
extern slReturn readSerialChar( int fdPort, long long msTimeout ) {

    unsigned char c;
    long long startTime = currentTimeMs();

    int count = 0;

    // figure out how many nanoseconds to sleep between port queries...
    speedInfo si;
    slReturn gsiResp = getSpeedInfo( fdPort, &si );
    if( isErrorReturn( gsiResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( gsiResp ), "could not get speed information" );
    int sleepyNs = si.nsChar >> 1;

    // keep trying unless we run out of time...
    while((currentTimeMs() - startTime) < msTimeout ) {

        count++;

        // try to read a character...
        ssize_t rx_length = read( fdPort, &c, 1 );

        // if we got an error, tell our caller...
        if((rx_length < 0) && (errno != EAGAIN))
            return makeErrorFmtMsgReturn( ERR_ROOT, "read() returned error: %s", strerror( errno ) );

        // if we read a character, return it...
        if( rx_length == 1 )
            return makeOkInfoReturn( char2info(c) );

        // otherwise, we didn't read anything..
        // delay a half character time before we try again...
        sleep_ns( sleepyNs );
    }

    // if we get here, then we timed out...
    return makeWarningReturn();
}


typedef struct baudTrans {
    int baud;
    speed_t cookie;
} baudTrans;
static baudTrans validRates[] = {
        {     50,     B50 },
        {     75,     B75 },
        {    110,    B110 },
        {    134,    B134 },
        {    150,    B150 },
        {    200,    B200 },
        {    300,    B300 },
        {    600,    B600 },
        {   1200,   B1200 },
        {   1800,   B1800 },
        {   2400,   B2400 },
        {   4800,   B4800 },
        {   9600,   B9600 },
        {  19200,  B19200 },
        {  38400,  B38400 },
        {  57600,  B57600 },
        { 115200, B115200 },
        { 230400, B230400 }
};

// Translate the given integer baud rate into the constant needed by termios to specify that baud rate.  If the given
// baud rate can't be translated, it is not a valid baud rate value.  If the given baud rate is valid, the termios
// constant is returned.  Otherwise, -1 is returned.
extern int getBaudRateCookie( int baudRate ) {

    for( int i = 0; i < ARRAY_SIZE( validRates ); i++ ) {
        if( baudRate == validRates[i].baud )
            return (int) validRates[i].cookie;
    }
    return -1;
}


// Translate the given baud rate cookie into the actual baud rate.  If the given cookie can't be translated, it wasn't
// a valid cookie.  If the given cookie is valid, the actual baud rate is returned as an int in info.
extern slReturn getBaudRate( int cookie ) {

    for( int i = 0; i < ARRAY_SIZE( validRates ); i++ ) {
        if( cookie == validRates[i].cookie )
            return makeOkInfoReturn( int2info( validRates[i].baud ) );
    }
    return makeErrorFmtMsgReturn( ERR_ROOT, "unrecognized baud rate cookie: %d", cookie );
}


// Flush the receive buffer.
extern slReturn flushRx( int fdPort ) {
    int result = tcflush( fdPort, TCIFLUSH );
    if( result < 0 )
        return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcflush(): %s", strerror(errno) );
    return makeOkReturn();
}


// Change the baud rate ONLY.  First waits for transmitter to finish sending any buffered data, then flushes any
// received data after changing the rate.
extern slReturn setTermOptionsBaud( int fdPort, int baud ) {

    if( fdPort < 0 ) return makeErrorMsgReturn( ERR_ROOT, "Invalid serial port file descriptor" );
    int baudCookie = getBaudRateCookie( baud );
    if( baudCookie < 0 ) return makeErrorMsgReturn( ERR_ROOT, "Invalid baud rate" );

    struct termios options;
    int result;
    result = tcdrain( fdPort );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcdrain(): %s", strerror(errno) );
    result = tcgetattr( fdPort, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcgetattr(): %s", strerror(errno) );
    result = cfsetispeed( &options, (speed_t) baudCookie );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from cfsetispeed(): %s", strerror(errno) );
    result = cfsetospeed( &options, (speed_t) baudCookie );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from cfsetospeed(): %s", strerror(errno) );
    result = tcflush( fdPort, TCIFLUSH );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcflush(): %s", strerror(errno) );
    result = tcsetattr( fdPort, TCSANOW, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn( ERR_ROOT, "error from tcsetattr(): %s", strerror(errno) );
    return makeOkReturn();
}


// Changes the terminal options on the given serial port to the given baud rate, number of data bits, number of stop
// bits, parity enable, and parity polarity (true for odd, false for even).  The given arguments are first checked for
// validity.
extern slReturn setTermOptions( int fdPort, int baud, int dataBits, int stopBits, bool parityEnable, bool parityOdd ) {

    if( fdPort < 0 ) return makeErrorMsgReturn(ERR_ROOT, "Invalid serial port file descriptor" );;
    int baudCookie = getBaudRateCookie( baud );
    if( baudCookie < 0 ) return makeErrorMsgReturn(ERR_ROOT, "Invalid baud rate" );
    int dataBitCookie = 0;
    switch( dataBits ) {
        case 5:
            dataBitCookie = CS5;
            break;
        case 6:
            dataBitCookie = CS6;
            break;
        case 7:
            dataBitCookie = CS7;
            break;
        case 8:
            dataBitCookie = CS8;
            break;
        default:
            return makeErrorFmtMsgReturn(ERR_ROOT, "Invalid number of data bits: %d", dataBits );
    }
    int stopBitCookie;
    if( stopBits == 1 ) stopBitCookie = 0;
    else if( stopBits == 2 ) stopBitCookie = CSTOPB;
    else return makeErrorFmtMsgReturn(ERR_ROOT, "Invalid number of stop bits: %d", stopBits );
    int parityCookie = (parityEnable ? PARENB | (parityOdd ? PARODD : 0) : 0);

    struct termios options;
    int result;
    result = tcgetattr( fdPort, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn(ERR_ROOT, "error from tcgetattr: %s", strerror(errno));
    options.c_cflag = (tcflag_t) (baudCookie | dataBitCookie | stopBitCookie | parityCookie | CLOCAL | CREAD);
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    result = tcflush( fdPort, TCIFLUSH );
    if( result < 0 ) return makeErrorFmtMsgReturn(ERR_ROOT, "error from tcflush: %s", strerror(errno));
    result = tcsetattr( fdPort, TCSANOW, &options );
    if( result < 0 ) return makeErrorFmtMsgReturn(ERR_ROOT, "error from tcsetattr: %s", strerror(errno));
    return makeOkReturn();
}


// Default baud rate synchronizer. This looks for 50 sequential characters in the normal range of ASCII codes, and
// then declares victory.
// This function takes one of three different actions depending on the argument values:
//   *state == NULL:            initializes its state and sets *state to point to it, returns false
//   *state != NULL && c >= 0:  updates the state, if synchronized returns true, sets *state to NULL
//   *state != NULL && c < 0:   releases resources, sets *state to NULL, returns false
extern bool ASCIIBaudRateSynchronizer( const char c, void** state ) {

    typedef struct dbrState {
        int count;
    } dbrState;

    // if we have no state, initialize it...
    if( !*state ) {
        *state = safeMalloc( sizeof( dbrState ));
        ((dbrState*) (*state))->count = 0;
        return false;
    }

    // if we are being told to release resources...
    if( c < 0 ) {
        free( *state );
        *state = NULL;
        return false;
    }

    // otherwise, update our state...
    if(((c >= 32) && (c <= 127)) || (c == 10) || (c == 9) || (c == 13))
        ((dbrState*) (*state))->count++;
    else
        ((dbrState*) (*state))->count = 0;

    // if we're synchronized...
    if(((dbrState*) (*state))->count >= 50 ) {
        free( *state );
        *state = NULL;
        return true;
    }

    // if we're not there yet...
    return false;
}


// Attempts to synchronize at the host's current baud rate.  If response is Ok, returns true for success, false
// for failure, in additional information.
extern slReturn synchronize( int fdPort, baudRateSynchronizer synchronizer, int verbosity ) {

    void* state = NULL;
    synchronizer( 0, &state );  // initialize the synchronizer

    speedInfo si;
    slReturn gsiResp = getSpeedInfo( fdPort, &si );
    if( isErrorReturn( gsiResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( gsiResp ), "couldn't get speed information" );

    if( verbosity > 0 ) printf( "Synchronizing at %d baud...\n", si.baudRate );
    long long start = currentTimeMs();
    long long timeout = max_ll( 1000, 200 * si.nsChar / 1000000 ); // max of one second or 200 character times...
    while((currentTimeMs() - start) < timeout ) {
        slReturn rscResp = readSerialChar( fdPort, start + timeout - currentTimeMs());
        if( isErrorReturn( rscResp ) ) {
            synchronizer( -1, &state );  // close synchronizer...
            return makeErrorMsgReturn( ERR_CAUSE( rscResp ), "couldn't read serial character" );
        }
        if( isWarningReturn( rscResp ) )
            break;  // if we timed out...

        int c = getReturnInfoChar( rscResp );

        // are we synchronized yet?
        if( verbosity >= 3 ) printf( "%c", c );
        if( synchronizer((char) c, &state )) {
            if( verbosity > 0 ) printf( "Synchronized at %d baud...\n", si.baudRate );
            return makeOkInfoReturn( bool2info( true ) );
        }
    }
    // we get here if we timed out...
    if( verbosity > 0 ) printf( "Timed out while attempting to synchronize at %d baud...\n", si.baudRate );
    synchronizer( -1, &state );  // close synchronizer...
    return makeOkInfoReturn( bool2info( false ) );
}


// Sets the baud rate on the given serial port.  If the given synchronizer is NULL, then the default synchronizer
// will be used.  The actions taken depend on the give values for baudRate and autoRate:
//   0, true       automatic baud rate discovery, starting at the highest baud rate and working down
//   n, true       automatic baud rate discovery, starting at the given rate (a hint) and then working down
//   n, false      set the baud rate to n, don't synchronize
// Return values:
//   SBR_SET_SYNC         baud rate is set and synchronized
//   SBR_SET_NOSYNC       baud rate is set, but wasn't synchronized
//   SBR_FAILED_SYNC      baud rate is not set and synchronization failed
//   SBR_INVALID          specified baud rate was invalid
//   SBR_PORT_ERROR       encountered a problem with the serial port
//extern int setBaudRate( int fdPort, int baudRate, bool synchronize, bool autoRate,
//                        baudRateSynchronizer synchronizer, bool verbose ) {
//
//    // if we didn't get a synchronizer, use the default...
//    if( !synchronizer ) synchronizer = ASCIIBaudRateSynchronizer;
//
//    // validate our baud rate, if it's specified...
//    if( baudRate ) {
//        int cookie = getBaudRateCookie( baudRate );
//        if( cookie < 0 ) return SBR_INVALID;
//    }
//
//    // if we don't have automatic baud rate discovery, then we must have a specified baud rate...
//    if( !autoRate && !baudRate ) return SBR_INVALID;
//
//    // set up the baud rates to try...
//    int bauds[] = { 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800,
//                    2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
//    if( !autoRate ) {
//        for( int i = 0; i < ARRAY_SIZE( bauds ); i++ ) {
//            if( bauds[i] != baudRate ) bauds[i] = 0;
//        }
//    }
//
//    // try all the baud rates that we're supposed to try (might be just one)...
//    int nextBaudIndex = ARRAY_SIZE( bauds ) - 1;
//    int nextBaud = baudRate;
//    bool done = false;
//    void *state = NULL;
//    synchronizer( 0, &state );  // initialize the synchronizer
//    while( !done ) {
//
//        // mark this baud rate as done...
//        for( int i = 0; i < ARRAY_SIZE( bauds ); i++ ) {
//            if( bauds[i] == nextBaud ) {
//                bauds[i] = 0;
//                break;
//            }
//        }
//
//        // set the new baud rate...
//        if( verbose ) printf( "Setting baud rate to %d...\n", nextBaud );
//        if( setTermOptionsBaud( fdPort, nextBaud ) != stoOK ) {
//            synchronizer( -1, &state );  // close synchronizer...
//            return SBR_INVALID;
//        }
//
//        // synchronize for auto or if requested...
//        if( autoRate || synchronize ) {
//
//            if( verbose ) printf( "Synchronizing at %d baud...\n", nextBaud );
//            long long start = currentTimeMs();
//            speedInfo si = getSpeedInfo( fdPort );
//            long long timeout = max_ll( 1000, 200 * si.nsChar / 1000000 ); // max of one second or 200 character times...
//            while( (currentTimeMs() - start) < timeout ) {
//                int c = readSerialChar( fdPort, start + timeout - currentTimeMs() );
//                if( c == RSC_READ_ERROR ) {
//                    synchronizer( -1, &state );  // close synchronizer...
//                    return SBR_PORT_ERROR;
//                }
//                if( c == RSC_TIMED_OUT ) {
//                    if( verbose ) printf( "Timed out when attempting to read serial character...\n" );
//                    break;
//                }
//
//                // are we synchronized yet?
//                if( synchronizer((char) c, &state) ) {
//                    if( verbose ) printf( "Synchronized at %d baud...\n", nextBaud );
//                    return SBR_SET_SYNC;
//                }
//            }
//            // we get here if we timed out...
//            if( verbose ) printf( "Timed out while attempting to synchronize at %d baud...\n", nextBaud );
//            synchronizer( -1, &state );  // close synchronizer...
//        }
//
//        // get the next baud rate to try...
//        while( (nextBaudIndex >= 0) && (bauds[nextBaudIndex] == 0) ) {
//            nextBaudIndex--;
//        }
//        if( nextBaudIndex < 0 )
//            done = true;
//        else {
//            nextBaud = bauds[nextBaudIndex];
//            bauds[nextBaudIndex] = 0;
//        }
//    }
//
//    // if we get here, it's time to leave...
//    return synchronize ? SBR_FAILED_SYNC : SBR_SET_NOSYNC;
//}