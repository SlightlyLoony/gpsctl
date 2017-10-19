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
#define _XOPEN_SOURCE
#ifdef Macintosh
    #define _DARWIN_C_SOURCE
#endif /* Macintosh */
#define _POSIX_C_SOURCE 199309L

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <getopt.h>
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
} parsed_args;


const char *gpsctl_version = "version: gpscfg 0.1\n";
const char *gpsctl_doc     = "purpose: gpscfg - control and configure U-Blox GPS on Raspberry Pi 3 port serial0\n";
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
                    "  -v, --verbose   get verbose messages";

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


// Parse the given argument, which is presumed to be a string representing a positive integer that is one of the
// standard baud rates.  If the parsing fails for any reason, the error flag is set, an error message set, and -1
// returned.
static int getBaud( const char * arg ) {

    int validRates[] = { 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800,
                         2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

    // sanity check...
    if( !arg ) {
        parsed_args.error = true;
        parsed_args.errorMsg = "missing baud rate argument";
        return -1;
    }

    // make sure we can parse the entire argument into an integer...
    char *endPtr;
    int baud = (int) strtol( arg, &endPtr, 10 );
    if( *endPtr != 0 ) {
        parsed_args.error = true;
        parsed_args.errorMsg = concat( "Baud rate not an integer: ", arg );
        return -1;
    }

    // make sure the result is a valid baud rate...
    for( int i = 0; i < ARRAY_SIZE(validRates); i++ ) {
        if( baud == validRates[i] )
            return baud;
    }
    parsed_args.error = true;
    parsed_args.errorMsg = concat( "Invalid baud rate ", arg );
    return -1;
}


// Process program options.  Returns true on success, false if there was any error.
bool getOptions( int argc, char *argv[] ) {
    int option_index;
    int c;
    while ((c = getopt_long(argc, argv, "VUp:b:arB:tv?", options, &option_index)) != -1) {

        if (c == -1) break;  // detect end of options...

        switch( c ) {
            case 0:         break;  // none of our options set flags, so naught to do here...
            case 'b': parsed_args.baud = getBaud( optarg );    break;
            default:                            break;
        }

        // check for an error on the option we just processed...
        if( parsed_args.error ) {
            puts( concat( "Error in program options: ", parsed_args.errorMsg ) );
            return false;
        }
    }

    return true;
}

int main( int argc, char *argv[] ) {

    if( getOptions( argc, argv ) ) {

        exit( 0 );
    }
    exit( 1 );
}