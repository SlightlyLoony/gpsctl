/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * G P S C T L
 * (GPS control)
 *
 * A utility for querying, configuring, and using a U-Blox GPS on Linux.
 *
 * This program was tested ONLY on a Raspberry Pi 3B, running Jessie, with a Uputronics board that uses a
 * U-Blox M8 GPS module.
 *
 * Created: October 18, 2017
 *  Author: Tom Dilatush <tom@dilatush.com>
 *  Github:
 * License: MIT
 *
 * Copyright <YEAR> <COPYRIGHT HOLDER>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 * to permit persons to whom the Software is furnished to do so.
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE A
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// TODO: make my own get_options_long() equivalent, easier to configure?

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "sl_general.h"
#include "sl_serial.h"
#include "ublox.h"


struct parsed_args {
    bool verbose;
    bool autobaud;
    char *port;
    int baud;
    int newbaud;
    bool usage;
    bool help;
    bool version;
    bool report;
    bool echo;
    int echoMax;
    bool error;
    bool synchronize;
    char *errorMsg;
    bool errorMsgAllocated;
};


static const char *gpsctl_version        = "gpsctl 0.1\n";
static const char *gpsctl_pretty_version = "Version:\n  %s";
static const char *gpsctl_doc            = "Purpose:\n  gpsctl - control, query and configure U-Blox GPS on Raspberry Pi 3 serial port\n";
static const char *gpsctl_usage          = "Usage:\n  gpsctl [-?VUarstv] -p <port> -b <baud> -B <baud> -e [maxtime]\n";
static const char *gpsctl_help = "Options:\n" \
                    "  -?, --help      display this help message and exit\n"
                    "  -V, --version   display gpsctl version number\n"
                    "  -U, --usage     display short message on usage of gpsctl\n"
                    "  -p, --port      specify the device for the serial port (default is '/dev/serial0')\n"
                    "  -b, --baud      specify current baud rate (default is 9600); any standard rate is allowed\n"
                    "  -a, --autobaud  enable baud rate discovery (if -b is specified, starting with that rate)\n"
                    "  -B, --newbaud   change the U-Blox baud rate to the specified standard rate\n"
                    "  -s, --sync      synchronize after setting baud rate\n"
                    "  -v, --verbose   get verbose messages\n"
                    "  -e, --echo      echo GPS (presumably NMEA) data to the stdout, optionally with a maximum time";


static void showHelp( void )    {
    printf( gpsctl_doc);
    printf( gpsctl_pretty_version, gpsctl_version );
    printf( gpsctl_usage );
    printf( gpsctl_help );
}

static void showVersion( void ) { printf( gpsctl_version ); }
static void showUsage( void )   { printf( gpsctl_usage );   }


// Used by getopt_long to parse long options.  Note that we're using it basically to map long options to short
// options, though it also gives us the additional benefit of using "equals" format (like "--baud=9600"
static struct option options[] = {
    { "help",        no_argument,       0, '?' },
    { "version",     no_argument,       0, 'V' },
    { "usage",       no_argument,       0, 'U' },
    { "port",        required_argument, 0, 'p' },
    { "baud",        required_argument, 0, 'b' },
    { "autobaud",    no_argument,       0, 'a' },
    { "new_baud",    required_argument, 0, 'B' },
    { "verbose",     no_argument,       0, 'v' },
    { "synchronize", no_argument,       0, 's' },
    { "echo",        optional_argument, 0, 'e' },
    { 0,             0,                 0, 0   }
};


// Given a short option character, returns the long version of the option (or null if there wasn't any).
static char* getLongOption( const char shortOption ) {
    struct option* curOption = options;
    while( curOption->name ) {
        if( shortOption == curOption->val )
            return (char *) curOption->name;
        curOption++;
    }
    return NULL;
}


// Helper function to create error messages with the argument embedded.  Always returns -1.
static int errMsgHelper( char *format, const char *arg, struct parsed_args *parsed_args ) {
    size_t len = strlen( format ) + strlen( arg ) + 10;
    parsed_args->errorMsg = safeMalloc( len );
    snprintf( parsed_args->errorMsg, len - 1, format, arg );
    parsed_args->errorMsgAllocated = true;
    return -1;
}


// Parse the given argument, which is presumed to be a string representing a positive integer that is one of the
// standard baud rates.  If the parsing fails for any reason, the error flag is set, an error message set, and -1
// returned.
static int getBaud( const char * arg, struct parsed_args *parsed_args ) {

    int validRates[] = { 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800,
                         2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

    // sanity check...
    if( !arg ) {
        parsed_args->errorMsg = "missing baud rate argument";
        return -1;
    }

    // make sure we can parse the entire argument into an integer...
    char *endPtr;
    int baud = (int) strtol( arg, &endPtr, 10 );
    if( *endPtr != 0 ) return errMsgHelper( "specified baud rate (\"%s\") not an integer", arg, parsed_args );

    // make sure the result is a valid baud rate...
    if( getBaudRateCookie( baud ) >0 ) return baud;

    // we can only get here if the given baud rate WASN'T a valid one...
    return errMsgHelper( "specified baud rate (\"%s\") is not valid", arg, parsed_args );
}


// Parses the given argument, which is presumed to be a string that is the name of a serial device.  If the specified
// device does not exist, or is not a serial device, produces an error.  If there were no errors, the device in
// parsed_args is changed to the argument.  The return value is 0 if there was no error, or -1 if there was an error.
static int getPort( const char *arg, struct parsed_args *parsed_args ) {

    int vsd = verifySerialDevice( arg );
    if( vsd == VSD_NULL          ) return errMsgHelper( "no serial device was specified",                          arg, parsed_args );
    if( vsd == VSD_NOT_TERMINAL  ) return errMsgHelper( "the specified device (\"%s\") is not a terminal",         arg, parsed_args );
    if( vsd == VSD_CANT_OPEN     ) return errMsgHelper( "the specified device (\"%s\") cannot be opened",          arg, parsed_args );
    if( vsd == VSD_NONEXISTENT   ) return errMsgHelper( "the specified device (\"%s\") doesn't exist",             arg, parsed_args );
    if( vsd == VSD_NOT_CHARACTER ) return errMsgHelper( "the specified device (\"%s\") is not a character device", arg, parsed_args );
    if( vsd == VSD_NOT_DEVICE    ) return errMsgHelper( "the specified device (\"%s\") is not a device",           arg, parsed_args );

    parsed_args->port = (char *) arg;
    return VSD_IS_SERIAL;
}


static int getEcho( const char *arg, struct parsed_args *parsed_args ) {

    // get any max time parameter...
    parsed_args->echoMax = 0;
    if( arg ) {

        // make sure we can parse the entire argument into an integer...
        char *endPtr;
        parsed_args->echoMax = (int) strtol(arg, &endPtr, 10);
        if (*endPtr != 0) return errMsgHelper("specified max echo time (\"%s\") not an integer", arg, parsed_args);
    }

    parsed_args->echo = true;
    return 0;
}


// Process program options.  Returns true on success, false if there was any error.
bool getOptions( int argc, char *argv[], struct parsed_args *parsed_args ) {

    int option_index;
    int opt;

    // set our default values...
    parsed_args->port = "/dev/serial0";
    parsed_args->baud = 9600;

    // process each option on the command line...
    while( (opt = getopt_long(argc, argv, ":VUp:b:arB:vse::?", options, &option_index)) != -1 ) {

        if (opt == -1) break;  // detect end of options...

        switch( opt ) {
            case 0:   /* get here if long option sets a flag */              break;  // naught to do here...
            case 'b': parsed_args->baud = getBaud( optarg, parsed_args );    break;  // -b or --baud
            case 'B': parsed_args->newbaud = getBaud( optarg, parsed_args ); break;  // -B or --newbaud
            case '?': parsed_args->help = true;                              break;  // -? or --help
            case 'U': parsed_args->usage = true;                             break;  // -U or --usage
            case 'V': parsed_args->version = true;                           break;  // -V or --version
            case 'p': getPort( optarg, parsed_args);                         break;  // -p or --port
            case 'v': parsed_args->verbose = true;                           break;  // -v or --verbose
            case 's': parsed_args->synchronize = true;                       break;  // -s or --sync
            case 'a': parsed_args->autobaud = true;                          break;  // -a or --autobaud
            case 'e': getEcho( optarg, parsed_args );                        break;  // -e or --echo
            case ':':                                                        break;  // required argument is missing
            default:                                                         break;
        }

        // check for an error on the option we just processed...
        if( parsed_args->errorMsg ) {
            parsed_args->error = true;
            printf("Error in program option -%c or --%s: %s\n", opt,
                   getLongOption((const char) opt), parsed_args->errorMsg );
            if( parsed_args->errorMsgAllocated ) free( parsed_args->errorMsg );
            parsed_args->errorMsg = NULL;
        }
    }

    return !parsed_args->error;
}


// NMEA baud rate synchronizer. This looks for a complete, valid NMEA sentence and then declares victory.
// This function takes one of three different actions depending on the argument values:
//   *state == NULL:            initializes its state and sets *state to point to it, returns false
//   *state != NULL && c >= 0:  updates the state, if synchronized returns true, sets *state to NULL
//   *state != NULL && c < 0:   releases resources, sets *state to NULL, returns false
#define NBRS ((nbrsState*)(*state))
static bool NMEABaudRateSynchronizer( const char c, void **state ) {

    typedef struct nbrsState {
        bool inLine;   // true if we're currently in a line...
        int cksm;      // the running checksum (actually longitudnal parity)...
        char last[3];  // the three most recently received characters...
    } nbrsState;

    // if we have no state, initialize it...
    if( !*state ) {
        *state = safeMalloc( sizeof( nbrsState ) );
        NBRS->inLine = false;
        return false;
    }

    // if we are being told to release resources...
    if( c < 0 ) {
        free( *state );
        *state = NULL;
        return false;
    }

    // otherwise, update our state...
    if( NBRS->inLine ) {
        if( (c == '\r') || (c == '\n') ) {
            if( NBRS->last[0] == '*') {
                int un = hex2int( NBRS->last[1] );
                if( un >= 0 ) {
                    int ln = hex2int( NBRS->last[2] );
                    if( ln >= 0 ) {
                        int ck = (un << 4) | ln;

                        // correct checksum for the last three characters received...
                        NBRS->cksm ^= NBRS->last[0];
                        NBRS->cksm ^= NBRS->last[1];
                        NBRS->cksm ^= NBRS->last[2];

                        if( ck == NBRS->cksm ) {
                            free( *state );
                            *state = NULL;
                            return true;
                        }
                    }
                }
            }
            NBRS->inLine = false;
        }
        else {
            NBRS->cksm ^= c;
            NBRS->last[0] = NBRS->last[1];
            NBRS->last[1] = NBRS->last[2];
            NBRS->last[2] = c;
        }
    }
    else if( c == '$' ) {
        NBRS->inLine = true;
        NBRS->cksm = 0;
    }

    // if we're not there yet...
    return false;
}


// Open and setup the serial port, including the baud rate.
static int openSerialPort( struct parsed_args *parsed_args ) {
    int fdPort = open( parsed_args->port, O_RDWR | O_NOCTTY | O_NONBLOCK );
    if( fdPort < 0 ) {
        printf( "Failed to open serial port (\"%s\")!\n", parsed_args->port );
        exit(1);
    }
    if( parsed_args->verbose ) printf( "Serial port (\"%s\") open...\n", parsed_args->port );

    int result = setTermOptions( fdPort, 9600, 8, 1, false, false );
    if( result != STO_OK ) {
        printf( "Error when setting terminal options (code: %d)!\n", result );
        close( fdPort );
        exit( 1 );
    }

    result = setBaudRate( fdPort, parsed_args->baud, parsed_args->synchronize, parsed_args->autobaud,
                          NMEABaudRateSynchronizer, parsed_args->verbose );
    int expected = (parsed_args->autobaud || parsed_args->synchronize) ? SBR_SET_SYNC : SBR_SET_NOSYNC;
    if( result != expected ) {
        printf( "Error when setting baud rate (code: %d)!\n", result );
        close( fdPort );
        exit( 1 );
    }
    if( parsed_args->verbose ) printf( "Serial port open and configured...\n" );
    return fdPort;
}


static void echo( int fdPort, bool verbose, int maxSeconds ) {

    if( verbose ) printf( "Starting echo...\n" );

    // figure out our timing...
    speedInfo si = getSpeedInfo( fdPort );
    long long startTime = currentTimeMs();
    long long maxEchoMS = 1000 * maxSeconds;
    int maxCharNS = si.nsChar >> 1;

    while( (maxSeconds == 0) || ((currentTimeMs() - startTime) < maxEchoMS) ) {
        int c = readSerialChar( fdPort, maxCharNS );
        if( c >= 0 ) printf( "%c", c );
    }

    if( verbose ) printf( "\nEnding echo...\n" );
}


// Print report on U-Blox GPS...
static void report( int fdPort, bool verbose ) {

}


// The entry point for gpsctl...
int main( int argc, char *argv[] ) {

    struct parsed_args parsed_args = {.verbose = false };  // initializes all members to zeroes...

    if( getOptions( argc, argv, &parsed_args ) ) {

        if( parsed_args.version ) showVersion();
        if( parsed_args.usage   ) showUsage();
        if( parsed_args.help    ) showHelp();

        // open and set up our serial port...
        int fdPort = openSerialPort( &parsed_args );

        test( fdPort );

        if( parsed_args.report  ) report( fdPort, parsed_args.verbose );
        if( parsed_args.echo    ) echo( fdPort, parsed_args.verbose, parsed_args.echoMax );

        close( fdPort );
        exit( 0 );
    }
    printf( "Errors prevented gpsctl from running..." );
    exit( 1 );
}