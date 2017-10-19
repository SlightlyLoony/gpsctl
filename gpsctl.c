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
#include <stdio.h>
#include <getopt.h>

#define ARG_ERROR   0
#define ARG_USAGE   1
#define ARG_HELP    2
#define ARG_VERSION 3
#define ARG_BAUD    4
#define ARG_READ    5
#define ARG_TOGGLE  6
#define ARG_ECHO    7

static int mode = 0;
static int quiet = 0;
static int high = 0;


const char *gpsctl_version = "gpscfg 0.1\n";
const char *gpsctl_doc     = "gpscfg - control and configure U-Blox GPS on Raspberry Pi 3 port serial0\n";
char *gpsctl_usage = "usage: gpsctl [-qh] -b | -r | -t | -e | -? | -V\n";
char *gpsctl_help = "Options:\n" \
                    "  -?, --help      display this help message and exit\n"\
                    "  -V, --version   display gpsctl version number\n"\
                    "  -b, --baud      report the GPS's baud rate\n" \
                    "  -e, --echo      echo GPS NMEA data to stdout\n"\
                    "  -h, --high      try synchronizing to 115,200 baud first, otherwise 9,600 baud\n"\
                    "  -q, --quiet     suppress non-error messages\n"\
                    "  -r, --read      read GPS port configuration\n"\
                    "  -t, --toggle    toggle GPS between 9,600 baud and 115,200 baud\n"\
                    "  --usage         display short message on usage of gpsctl\n";

static struct option options[] = {
    { "usage",   no_argument, &mode,  ARG_USAGE   },
    { "help",    no_argument, &mode,  ARG_HELP    },
    { "version", no_argument, &mode,  ARG_VERSION },
    { "baud",    no_argument, &mode,  ARG_BAUD    },
    { "read",    no_argument, &mode,  ARG_READ    },
    { "toggle",  no_argument, &mode,  ARG_TOGGLE  },
    { "echo",    no_argument, &mode,  ARG_ECHO    },
    { "quiet",   no_argument, &quiet, 1           },
    { "high",    no_argument, &high,  1           },
    { 0, 0, 0, 0 }
};


// get program options...
void getOptions( int argc, char *argv[] ) {
    int option_index;
    int c;
    int mc = 0;
    while( ( c = getopt_long( argc, argv, "breqth?V", options, &option_index) ) != -1 ) {

        if( c == -1 ) break;  // detect end of options...

        switch( c ) {
            case 0:
                if( options[option_index].flag == &mode ) mc++;
                break;
            case 'b': mc++; mode = ARG_BAUD;    break;
            case 'r': mc++; mode = ARG_READ;    break;
            case 't': mc++; mode = ARG_TOGGLE;  break;
            case 'e': mc++; mode = ARG_ECHO;    break;
            case 'V': mc++; mode = ARG_VERSION; break;
            case 'q': quiet = 1;                break;
            case 'h': high = 1;                 break;
            case '?': mc++; mode = ARG_HELP;    break;
            default:                            break;
        }
    }
    if( mc != 1 ) mode = ARG_ERROR;
}

int main( int argc, char *argv[] ) {

    getOptions( argc, argv );

    if( mode != ARG_ERROR ) {

        int result;
        int fd;

        switch( mode ) {

            case ARG_USAGE:
                puts( gpsctl_usage );
                break;

            case ARG_HELP:
            case ARG_ERROR:
                puts( gpsctl_doc );
                puts( gpsctl_usage );
                puts( gpsctl_help );
                break;

            case ARG_VERSION:
                puts( gpsctl_version );
                break;

            default:
                break;
        }
        exit( 0 );
    }
    else {
        exit( 1 );
    }
}