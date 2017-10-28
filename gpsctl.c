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
// TODO: add save configuration
// TODO: add set stationary or portable mode
// TODO: add UBX-only synchronizer (so it will synchronize even without NMEA data)
// TODO: choose sync type
// TODO: set antenna and cable length specs
// TODO: make sure we free any allocated slBuffers!
// TODO: add needed printfs for verbose mode...


// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include "sl_options.h"
#include "sl_general.h"
#include "sl_serial.h"
#include "ublox.h"
#include "cJSON.h"


typedef struct parsed_args {
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
    bool nmea;
    bool nmeaState;
    bool json;
    int echoMax;
    int queryType;
    bool error;
    bool synchronize;
    char *errorMsg;
    bool errorMsgAllocated;
} parsed_args;

struct clientData_slOptions {
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
    bool nmea;
    bool nmeaState;
    bool json;
    int echoMax;
    int queryType;
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
                    "  -e, --echo      echo human-readable GPS (presumably NMEA) data to stdout for 'n' seconds (0 means forever)\n"
                    "  -q, --query     query the GPS for info (see QUERY TYPES below)\n"
                    "  -j, --json      produces query results in JSON format instead of English\n"
                    "  -n  --nmea      configure whether the GPS sends NMEA data to the UART (see YES OR NO VALUES below)\n"
                    "\n"
                    "Query types:\n"
                    "   fix       reports latitude, longitude, altitude, and time with accuracy figures\n"
                    "   version   reports the version information from the GPS\n"
                    "\n"
                    "Yes or no values\n"
                    "   Yes       indicated by an initial character of 'y', 'Y', 't', 'T', or '1'\n"
                    "   No        indicated by anything else\n";


static void showHelp( void )    {
    printf( gpsctl_doc);
    printf( gpsctl_pretty_version, gpsctl_version );
    printf( gpsctl_usage );
    printf( gpsctl_help );
}

static void showVersion( void ) { printf( gpsctl_version ); }
static void showUsage( void )   { printf( gpsctl_usage );   }



static parseResult_slOptions parsePort(optionDef_slOptions *, char *, clientData_slOptions *);
static parseResult_slOptions parseBaud(optionDef_slOptions *, char *, clientData_slOptions *);


static optionDef_slOptions new_options[] = {
  //  long       short max  argtype      parsefunc   cnstrfunc    help
    { "port",     'p',  1,  argRequired, parsePort,  NULL,        "specify current baud rate (default is 9600); any standard rate is allowed" },
    { "verbose",  'v',  3,  argNone,     NULL,       NULL,        "get verbose messages, use up to three times for more verbosity" },
    { "autobaud", 'a',  1,  argNone,     NULL,       NULL,        "enable baud rate discovery (if -b is also specified, starting with that rate)" },
    { "baud",     'b',  1,  argRequired, parseBaud,  NULL,        "specify current baud rate (default is 9600); any standard rate is allowed"},
    { "newbaud",  'B',  1,  argRequired, parseBaud,  NULL,        "change the U-Blox and host baud rates to the specified standard rate"},
    { NULL }
};


// Helper function to create error messages with given meta pattern, the subpattern, and arguments embedded.  Returns an
// error response with the given pattern and arguments resolved into a string.  Note that the returned value was
// allocated, and should be freed when no longer needed.  The meta pattern must specify (in order) a %s to contain the
// option identity, and another %s to contain the resolved error message.
static char* msgHelper( char* meta, optionDef_slOptions* def, char *pattern, va_list args) {

    // first we resolve the pattern we were given...
    char* resolved;
    vasprintf( &resolved, pattern, args );

    // then we insert that into our pattern...
    char* errMsg;
    asprintf( &errMsg, meta , getName_slOptions( def ), resolved );
    free( resolved );
    
    return errMsg;
}


static parseResult_slOptions parseMsgHelper( optionDef_slOptions* def, optParseRC_slOptions rc, char *pattern, ... ) {

    // get our error message...
    va_list args;
    va_start( args, pattern );
    char* errMsg = msgHelper( "Option parsing problem: [%s] %s", def, pattern, args );
    va_end( args );
    
    // return our bad news...
    parseResult_slOptions result;
    result.rc = rc;
    result.msg = errMsg;
    return result;
}

// Test the argument to see if it represents a valid serial port.  On failure, return an error.
static parseResult_slOptions parsePort(optionDef_slOptions *def, char *arg, clientData_slOptions *clientData) {

    char* id = getName_slOptions( def );
    int vsd = verifySerialDevice( arg );
    switch( vsd ) {
        case VSD_IS_SERIAL:
            clientData->port = arg;
            parseResult_slOptions result = { optParseOk, NULL };
            return result;
        case VSD_NULL:          return parseMsgHelper( def, optParseError, "no serial device was specified", id );
        case VSD_NOT_TERMINAL:  return parseMsgHelper( def, optParseError, "\"%s\" is not a terminal", id, arg );
        case VSD_CANT_OPEN:     return parseMsgHelper( def, optParseError, "\"%s\" cannot be opened", id, arg );
        case VSD_NONEXISTENT:   return parseMsgHelper( def, optParseError, "\"%s\" doesn't exist", id, arg );
        case VSD_NOT_CHARACTER: return parseMsgHelper( def, optParseError, "\"%s\" is not a character device", id, arg );
        case VSD_NOT_DEVICE:    return parseMsgHelper( def, optParseError, "\"%s\" is not a device", id, arg );
        default:                return parseMsgHelper( def, optParseError, "\"%s\" failed for unknown reasons (code %d)", id, arg, vsd );
    }
}


// Parse the given argument, which is presumed to be a string representing a positive integer that is one of the
// standard baud rates.  If the parsing fails for any reason, return an error
static parseResult_slOptions parseBaud( optionDef_slOptions* def, char* arg, clientData_slOptions* clientData ) {

    // make sure we can parse the entire argument into an integer...
    char *endPtr;
    int baud = (int) strtol( arg, &endPtr, 10 );
    if( *endPtr != 0 ) return parseMsgHelper( def, optParseError, "the specified baud rate (\"%s\") is not an integer", arg );

    // make sure the result is a valid baud rate...
    if( getBaudRateCookie( baud ) == 0 )
        return parseMsgHelper( def, optParseError, "the specified baud rate (\"%s\") is not a valid one (4800, 9600, 19200, etc.", arg );

    // we have a valid baud rate, so stuff it away (for either -b or -B) and return in victory...
    if( def->shortOpt == 'b' )
        clientData->baud = baud;
    else if( def->shortOpt == 'B' )
        clientData->newbaud = baud;
    parseResult_slOptions response = { optParseOk, NULL };
    return response;
}


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
    { "echo",        required_argument, 0, 'e' },
    { "query",       required_argument, 0, 'q' },
    { "json",        no_argument,       0, 'j' },
    { "nmea",        required_argument, 0, 'n' },
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
static int errMsgHelper( char *format, const char *arg, parsed_args *parsed_args ) {
    size_t len = strlen( format ) + strlen( arg ) + 10;
    parsed_args->errorMsg = safeMalloc( len );
    snprintf( parsed_args->errorMsg, len - 1, format, arg );
    parsed_args->errorMsgAllocated = true;
    return -1;
}




// Parses the given argument, which is presumed to be a string that is the name of a serial device.  If the specified
// device does not exist, or is not a serial device, produces an error.  If there were no errors, the device in
// parsed_args is changed to the argument.  The return value is 0 if there was no error, or -1 if there was an error.
static int getPort( const char *arg, parsed_args *parsed_args ) {

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


static char *query_types[] = {
        "fix",
        "version"
};

// Parses the given argument and sets queryType in parsed_args to indicate the type.  Returns 0 on success, or -1
// if the argument wasn't recognized.
static int getQueryType( const char *arg, parsed_args *parsed_args ) {

    for( int i = 0; i < sizeof( query_types); i++ ) {

        if( strcmp( arg, query_types[i] ) == 0 ) {
            parsed_args->queryType = i + 1;
            return 0;
        }
    }
    return errMsgHelper( "the given argument (\"%s\") is not a valid option for -q/--query", arg, parsed_args );
}


static int getEcho( const char *arg, parsed_args *parsed_args ) {

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


// Reads the argument (first character only) to see if it represents a true or a false, then sets that value.
static void getBool( const char *arg, bool *boolArg ) {
    char *answer = strchr( "yYtT1", *arg );
    *boolArg = (answer != NULL);
}


// Process program options.  Returns true on success, false if there was any error.
static bool getOptions( int argc, char *argv[], parsed_args *pa ) {

    int option_index;
    int opt;

    // set our default values...
    pa->port = "/dev/serial0";
    pa->baud = 9600;

    // process each option on the command line...
    while( (opt = getopt_long(argc, argv, ":VUp:b:arB:vsn:jq:e:?", options, &option_index)) != -1 ) {

        if (opt == -1) break;  // detect end of options...

        switch( opt ) {
            case 0:   /* get here if long option sets a flag */           break;  // naught to do here...
//            case 'b': pa->baud = getBaud( optarg, pa );                   break;  // -b or --baud
//            case 'B': pa->newbaud = getBaud( optarg, pa );                break;  // -B or --newbaud
            case '?': pa->help = true;                                    break;  // -? or --help
            case 'U': pa->usage = true;                                   break;  // -U or --usage
            case 'V': pa->version = true;                                 break;  // -V or --version
            case 'p': getPort( optarg, pa);                               break;  // -p or --port
            case 'v': pa->verbose = true;                                 break;  // -v or --verbose
            case 's': pa->synchronize = true;                             break;  // -s or --sync
            case 'a': pa->autobaud = true;                                break;  // -a or --autobaud
            case 'e': getEcho( optarg, pa );                              break;  // -e or --echo
            case 'q': getQueryType( optarg, pa );                         break;  // -q or --query
            case 'j': pa->json = true;                                    break;  // -j or --json
            case 'n': pa->nmea = true; getBool( optarg, &pa->nmeaState ); break;  // -n or --nmea
            case ':':                                                     break;  // required argument is missing
            default:                                                      break;
        }

        // check for an error on the option we just processed...
        if( pa->errorMsg ) {
            pa->error = true;
            printf("Error in program option -%c or --%s: %s\n", opt,
                   getLongOption((const char) opt), pa->errorMsg );
            if( pa->errorMsgAllocated ) free( pa->errorMsg );
            pa->errorMsg = NULL;
        }
    }

    return !pa->error;
}


// NMEA baud rate synchronizer. This looks for a complete, valid NMEA sentence and then declares victory.
// This function takes one of three different actions depending on the argument values:
//   *state == NULL:            initializes its state and sets *state to point to it, returns false
//   *state != NULL && c >= 0:  updates the state, if synchronized returns true, sets *state to NULL
//   *state != NULL && c < 0:   releases resources, sets *state to NULL, returns false
static bool NMEABaudRateSynchronizer( const char c, void **state ) {

    typedef struct nbrsState {
        bool inLine;   // true if we're currently in a line...
        int cksm;      // the running checksum (actually longitudnal parity)...
        char last[3];  // the three most recently received characters...
    } nbrsState;
    
    // convenience variable...
    nbrsState *nbrs = ((nbrsState*)(*state));

    // if we have no state, initialize it...
    if( !*state ) {
        *state = safeMalloc( sizeof( nbrsState ) );
        nbrs = ((nbrsState*)(*state));
        nbrs->inLine = false;
        return false;
    }

    // if we are being told to release resources...
    if( c < 0 ) {
        free( *state );
        *state = NULL;
        return false;
    }

    // otherwise, update our state...
    if( nbrs->inLine ) {
        if( (c == '\r') || (c == '\n') ) {
            if( nbrs->last[0] == '*') {
                int un = hex2int( nbrs->last[1] );
                if( un >= 0 ) {
                    int ln = hex2int( nbrs->last[2] );
                    if( ln >= 0 ) {
                        int ck = (un << 4) | ln;

                        // correct checksum for the last three characters received...
                        nbrs->cksm ^= nbrs->last[0];
                        nbrs->cksm ^= nbrs->last[1];
                        nbrs->cksm ^= nbrs->last[2];

                        if( ck == nbrs->cksm ) {
                            free( *state );
                            *state = NULL;
                            return true;
                        }
                    }
                }
            }
            nbrs->inLine = false;
        }
        else {
            nbrs->cksm ^= c;
            nbrs->last[0] = nbrs->last[1];
            nbrs->last[1] = nbrs->last[2];
            nbrs->last[2] = c;
        }
    }
    else if( c == '$' ) {
        nbrs->inLine = true;
        nbrs->cksm = 0;
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
        switch( result ) {
            case SBR_INVALID:
                printf( "Baud rate is invalid or not specified!\n" );
                break;
            case SBR_FAILED_SYNC:
                printf( "Failed to synchronize serial port receiver!\n" );
                break;
            case SBR_PORT_ERROR:
                printf( "Error reading from serial port!\n" );
                break;
            default:
                printf( "Unknown error when setting baud rate (code: %d)!\n", result );
                break;
        }
        close( fdPort );
        exit( 1 );
    }
    if( parsed_args->verbose ) printf( "Serial port open and configured...\n" );
    return fdPort;
}


static void echo( int fdPort, bool verbose, int maxSeconds ) {

    if( verbose ) printf( "Starting echo...\n" );

    // figure out our timing...
    long long startTime = currentTimeMs();
    long long maxEchoMS = 1000 * maxSeconds;

    while( (maxSeconds == 0) || ((currentTimeMs() - startTime) < maxEchoMS) ) {
        int c = readSerialChar( fdPort, 1000 );
        if( c >= 0 ) printf( "%c", c );
    }

    if( verbose ) printf( "\nEnding echo...\n" );
}


// Change baud rate on GPS and on host port...
static void newBaud( int fdPort, int newBaud, bool verbose ) {

    changeBaudResponse resp = changeBaudRate( fdPort, (unsigned) newBaud, verbose );
    if( resp == ChangeBaudFailure ) {
        printf( "Failed to change baud rate!" );
        exit(1);
    }
}


// Outputs a fix query, in English or JSON.
static void fixQuery( int fdPort, bool json, bool verbose ) {

    pvt_fix fix = {0}; // zeroes all elements...
    reportResponse result = getFix( fdPort, verbose, &fix );
    double msl_height_feet = 0.00328084 * fix.height_above_sea_level_mm;
    double height_accuracy_feet = 0.00328084 * fix.height_accuracy_mm;
    double horizontal_accuracy_feet = 0.00328084 * fix.horizontal_accuracy_mm;
    double speed_mph = fix.ground_speed_mm_s * 0.00223694;
    double speed_accuracy_mph = fix.ground_speed_accuracy_mm_s * 0.00223694;
    if( result == reportOK ) {
        if( json ) {
            cJSON *root;
            cJSON *objFix;
            cJSON *time;
            root = cJSON_CreateObject();

            cJSON_AddItemToObject( root, "fix", objFix = cJSON_CreateObject() );
            cJSON_AddNumberToObject( objFix, "latitude_deg", fix.latitude_deg );
            cJSON_AddNumberToObject( objFix, "longitude_deg", fix.longitude_deg );
            cJSON_AddNumberToObject( objFix, "height_above_ellipsoid_mm", fix.height_above_ellipsoid_mm );
            cJSON_AddNumberToObject( objFix, "height_above_mean_sea_level_mm", fix.height_above_sea_level_mm );
            cJSON_AddNumberToObject( objFix, "horizontal_accuracy_mm", fix.horizontal_accuracy_mm );
            cJSON_AddNumberToObject( objFix, "height_accuracy_mm", fix.height_accuracy_mm );
            cJSON_AddNumberToObject( objFix, "ground_speed_mm_sec", fix.ground_speed_mm_s );
            cJSON_AddNumberToObject( objFix, "ground_speed_accuracy_mm_sec", fix.ground_speed_accuracy_mm_s );
            cJSON_AddNumberToObject( objFix, "heading_deg", fix.heading_deg );
            cJSON_AddNumberToObject( objFix, "heading_accuracy_deg", fix.heading_accuracy_deg );
            cJSON_AddBoolToObject( objFix, "valid", fix.fix_valid );
            cJSON_AddBoolToObject( objFix, "3d", fix.fix_is_3d );

            cJSON_AddNumberToObject( root, "number_of_satellites_used", fix.number_of_satellites_used );

            cJSON_AddItemToObject( root, "time", time = cJSON_CreateObject() );
            cJSON_AddNumberToObject( time, "month", fix.month );
            cJSON_AddNumberToObject( time, "day", fix.day );
            cJSON_AddNumberToObject( time, "year", fix.year );
            cJSON_AddNumberToObject( time, "hour", fix.hour );
            cJSON_AddNumberToObject( time, "minute", fix.minute );
            cJSON_AddNumberToObject( time, "second", fix.second );
            cJSON_AddNumberToObject( time, "accuracy_ns", fix.time_accuracy_ns );
            cJSON_AddBoolToObject( time, "valid", fix.time_valid );
            cJSON_AddBoolToObject( time, "resolved", fix.time_resolved );

            char *jsonStr = cJSON_PrintUnformatted( root );
            printf( "%s\n", jsonStr );

            cJSON_Delete( root );
        }
        else {
            if( fix.time_resolved && fix.time_valid && fix.date_valid )
                printf( " Time (UTC): %02d/%02d/%04d %02d:%02d:%02.0f (mm/dd/yyyy hh:mm:ss)\n",
                        fix.month, fix.day, fix.year, fix.hour, fix.minute, fix.second );
            else
                printf( " Time (UTC): not resolved or not valid\n" );
            if( fix.fix_valid ) {
                printf( "   Latitude: %12.8f %c\n", fabs(fix.latitude_deg), (fix.latitude_deg < 0) ? 'S' : 'N');
                printf( "  Longitude: %12.8f %c\n", fabs(fix.longitude_deg), (fix.longitude_deg < 0) ? 'W' : 'E');
                if( fix.fix_is_3d )
                    printf( "   Altitude: %.3f feet\n", msl_height_feet);
                else
                    printf( "   Altitude: unknown\n" );
                printf( "     Motion: %.3f mph at %.3f degrees heading\n", speed_mph, fix.heading_deg);
            }
            else
                printf( "        Fix: invalid\n" );
            printf( " Satellites: %d (used for computing this fix)\n", fix.number_of_satellites_used );
            printf( "   Accuracy: time (%d ns), height (+/-%.3f feet), position (+/-%.3f feet), heading(+/-%.3f degrees), speed(+/-%.3f mph)\n",
                    fix.time_accuracy_ns, height_accuracy_feet, horizontal_accuracy_feet, fix.heading_accuracy_deg, speed_accuracy_mph );
        }
    }
    else {
        printf( "Problem getting fix information from GPS!" );
        exit(1);
    }
}


// Outputs a version query, in English or JSON.
static void versionQuery( int fdPort, bool json, bool verbose ) {

    ubxVersion version = {0};  // zeroes all elements...
    reportResponse result = getVersion( fdPort, verbose, &version );

    if( result == reportOK ) {

        if( json ) {

            cJSON *root;
            root = cJSON_CreateObject();
            cJSON_AddItemToObject( root, "software_version", cJSON_CreateString( version.software ) );
            cJSON_AddItemToObject( root, "hardware_version", cJSON_CreateString( version.hardware ) );
            cJSON_AddItemToObject( root, "extensions",
                                   cJSON_CreateStringArray( (const char**) version.extensions, version.number_of_extensions ) );

            char *jsonStr = cJSON_PrintUnformatted( root );
            printf( "%s\n", jsonStr );

            cJSON_Delete( root );
        }
        else {
            printf( "Software version: %s\n", version.software );
            free( version.software );
            printf( "Hardware version: %s\n", version.hardware );
            free( version.hardware );
            char **ptr = version.extensions;
            while( (*ptr) != NULL ) {
                printf( "       Extension: %s\n", *ptr );
                free( (*ptr) );
                ptr++;
            }
        }
    }
    else {
        printf( "Problem getting version information from GPS!" );
        exit(1);
    }
}


// Query the GPS for different things.
static void query( int fdPort, int queryType, bool json, bool verbose ) {

    switch( queryType ) {

        case 1: fixQuery( fdPort, json, verbose ); break;  // "fix"
        case 2: versionQuery( fdPort, json, verbose );
        default: break;
    }
}


// Set whether the GPS emits NMEA data on the UART...
static void configureNmea( int fdPort, bool verbose, bool nmeaOn ) {
    configResponse resp = setNmeaData( fdPort, verbose, nmeaOn );
    if( resp == configError ) {
        printf( "Could not configure NMEA data!\n" );
        exit(1);
    }
}


// The entry point for gpsctl...
int main( int argc, char *argv[] ) {

    clientData_slOptions clientData;
    psloConfig config = { new_options, &clientData, false };
    psloResponse resp = process_slOptions(argc, (const char **) argv, &config );

    exit( EXIT_SUCCESS );

    struct parsed_args pas = {.verbose = false };  // initializes all members to zeroes...


    if( getOptions( argc, argv, &pas ) ) {

        if( pas.version ) showVersion();
        if( pas.usage   ) showUsage();
        if( pas.help    ) showHelp();

        // open and set up our serial port...
        int fdPort = openSerialPort( &pas );

        if( pas.nmea      ) configureNmea( fdPort, pas.verbose, pas.nmeaState );
        if( pas.queryType ) query( fdPort, pas.queryType, pas.json, pas.verbose );
        if( pas.newbaud   ) newBaud( fdPort, pas.newbaud, pas.verbose );
        if( pas.echo      ) echo( fdPort, pas.verbose, pas.echoMax );

        close( fdPort );
        exit( EXIT_SUCCESS );
    }
    printf( "Errors prevented gpsctl from running..." );
    exit( EXIT_FAILURE );
}
