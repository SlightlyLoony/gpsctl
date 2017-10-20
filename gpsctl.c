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

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#ifdef Macintosh
    #define _DARWIN_C_SOURCE
#endif /* Macintosh */
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include "slightly_loony.h"

static struct parsed_args {
    bool verbose;
    bool autobaud;
    char *port;
    int baud;
    int newbaud;
    bool test;
    bool usage;
    bool help;
    bool version;
    bool report;
    bool error;
    char *errorMsg;
    bool errorMsgAllocated;
} parsed_args;


const char *gpsctl_version = "version: gpsctl 0.1\n";
const char *gpsctl_doc     = "purpose: gpsctl - control, query and configure U-Blox GPS on Raspberry Pi 3 serial port\n";
char *gpsctl_usage = "usage: gpsctl [-?VUartv] -p \n";
char *gpsctl_help = "options:\n" \
                    "  -?, --help      display this help message and exit\n"
                    "  -V, --version   display gpsctl version number\n"
                    "  -U, --usage     display short message on usage of gpsctl\n"
                    "  -p, --port      specify the device for the serial port (default is '/dev/serial0')\n"
                    "  -b, --baud      specify current baud rate (default is 9600); any standard rate is allowed\n"
                    "  -a, --autobaud  enable baud rate discovery (if -b is specified, starting with that rate)\n"
                    "  -r, --report    print a report on the U-Blox GPS configuration\n"
                    "  -B, --newbaud   change the U-Blox baud rate to the specified standard rate\n"
                    "  -t, --test      test for the presence of NMEA data on the serial port\n"
                    "  -v, --verbose   get verbose messages\n";


static void showHelp( void )    { printf( gpsctl_help );    }
static void showVersion( void ) { printf( gpsctl_version ); }
static void showUsage( void )   { printf( gpsctl_usage );   }


// Used by getopt_long to parse long options.  Note that we're using it basically to map long options to short
// options, though it also gives us the additional benefit of using "equals" format (like "--baud=9600"
static struct option options[] = {
    { "help",     no_argument,       0, '?' },
    { "version",  no_argument,       0, 'V' },
    { "usage",    no_argument,       0, 'U' },
    { "port",     required_argument, 0, 'p' },
    { "baud",     required_argument, 0, 'b' },
    { "autobaud", no_argument,       0, 'a' },
    { "report",   no_argument,       0, 'r' },
    { "new_baud", required_argument, 0, 'B' },
    { "test",     no_argument,       0, 't' },
    { "verbose",  no_argument,       0, 'v' },
    { 0,          0,                 0, 0   }
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
static int errMsgHelper( char *format, const char *arg ) {
    size_t len = strlen( format ) + strlen( arg ) + 10;
    parsed_args.errorMsg = safeMalloc( len );
    snprintf( parsed_args.errorMsg, len - 1, format, arg );
    parsed_args.errorMsgAllocated = true;
    return -1;
}


// Parse the given argument, which is presumed to be a string representing a positive integer that is one of the
// standard baud rates.  If the parsing fails for any reason, the error flag is set, an error message set, and -1
// returned.
static int getBaud( const char * arg ) {

    int validRates[] = { 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800,
                         2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

    // sanity check...
    if( !arg ) {
        parsed_args.errorMsg = "missing baud rate argument";
        return -1;
    }

    // make sure we can parse the entire argument into an integer...
    char *endPtr;
    int baud = (int) strtol( arg, &endPtr, 10 );
    if( *endPtr != 0 ) return errMsgHelper( "specified baud rate (\"%s\") not an integer", arg );

    // make sure the result is a valid baud rate...
    if( getBaudRateCookie( baud ) >0 ) return baud;

    // we can only get here if the given baud rate WASN'T a valid one...
    return errMsgHelper( "specified baud rate (\"%s\") is not valid", arg );
}


// Parses the given argument, which is presumed to be a string that is the name of a serial device.  If the specified
// device does not exist, or is not a serial device, produces an error.  If there were no errors, the device in
// parsed_args is changed to the argument.  The return value is 0 if there was no error, or -1 if there was an error.
static int getPort( const char *arg ) {

    int vsd = verifySerialDevice( arg );
    if( vsd == VSD_NULL          ) return errMsgHelper( "no serial device was specified", arg );
    if( vsd == VSD_NOT_TERMINAL  ) return errMsgHelper( "the specified device (\"%s\") is not a terminal", arg );
    if( vsd == VSD_CANT_OPEN     ) return errMsgHelper( "the specified device (\"%s\") cannot be opened", arg );
    if( vsd == VSD_NONEXISTENT   ) return errMsgHelper( "the specified device (\"%s\") doesn't exist", arg );
    if( vsd == VSD_NOT_CHARACTER ) return errMsgHelper( "the specified device (\"%s\") is not a character device", arg );
    if( vsd == VSD_NOT_DEVICE    ) return errMsgHelper( "the specified device (\"%s\") is not a device", arg );

    parsed_args.port = (char *) arg;
    return VSD_IS_SERIAL;
}


// Process program options.  Returns true on success, false if there was any error.
bool getOptions( int argc, char *argv[] ) {

    int option_index;
    int opt;

    // set our default values...
    parsed_args.port = "/dev/serial0";
    parsed_args.baud = 9600;

    // process each option on the command line...
    while( (opt = getopt_long(argc, argv, "VUp:b:arB:tv?", options, &option_index)) != -1 ) {

        if (opt == -1) break;  // detect end of options...

        switch( opt ) {
            case 0:                                            break;  // none of our options set flags, so naught to do here...
            case 'b': parsed_args.baud = getBaud( optarg );    break;  // -b or --baud
            case 'B': parsed_args.newbaud = getBaud( optarg ); break;  // -B or --newbaud
            case '?': parsed_args.help = true;                 break;  // -? or --help
            case 'U': parsed_args.usage = true;                break;  // -U or --usage
            case 'V': parsed_args.version = true;              break;  // -V or --version
            case 'p': getPort( optarg);                        break;  // -p or --port
            case 'v': parsed_args.verbose = true;              break;  // -v or --verbose
            default:                                           break;
        }

        // check for an error on the option we just processed...
        if( parsed_args.errorMsg ) {
            parsed_args.error = true;
            printf("Error in program option -%c or --%s: %s\n", opt,
                   getLongOption((const char) opt), parsed_args.errorMsg );
            if( parsed_args.errorMsgAllocated ) free( parsed_args.errorMsg );
            parsed_args.errorMsg = NULL;
        }
    }

    return !parsed_args.error;
}


// The entry point for gpsctl...
int main( int argc, char *argv[] ) {

    if( getOptions( argc, argv ) ) {

        if( parsed_args.version ) showVersion();
        if( parsed_args.usage   ) showUsage();
        if( parsed_args.help    ) showHelp();

        // open our serial port for reading or writing, non-blocking...
        int serial = open( parsed_args.port, O_RDWR | O_NOCTTY | O_NONBLOCK );
        if( serial < 0 ) {
            printf( "Failed to open serial port (\"%s\")!\n", parsed_args.port );
            exit(1);
        }
        if( parsed_args.verbose ) printf( "Serial port (\"%s\") open...\n", parsed_args.port );

        close( serial );
        exit( 0 );
    }
    printf( "Errors prevented gpsctl from running..." );
    exit( 1 );
}