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
 * Copyright 2017 Tom Dilatush
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

// TODO: add save configuration
// TODO: add set stationary or portable mode
// TODO: set antenna and cable length specs
// TODO: make sure we free any allocated slBuffers!
// TODO: add needed printfs for verbose mode...
// TODO: add filter for NMEA message types on echo, also message verifier (only echo good messages)
// TODO: helper function (or macro?) for the awkward structure return values


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

typedef enum { fixQuery, versionQuery } queryType;
typedef enum { syncNone, syncASCII, syncNMEA, syncUBX } syncType;
typedef struct { char* name; syncType type; } syncTypeRec;

static syncTypeRec syncTypeRecs[] = {
        { "ascii", syncASCII },
        { "nmea",  syncNMEA  },
        { "ubx",   syncUBX   }
};

struct clientData_slOptions {
    int verbosity;
    syncType syncMethod;
    const char *port;
    int baud;
    int minBaud;
    int newbaud;
    bool report;
    bool echo;
    bool nmea;
    bool json;
    int echoSeconds;
    queryType queryType;
    bool error;
    int fdPort;
};


// Set the sync method to that specified in the argument.  Return true if there's no argument (and do nothing) or the
// argument was recognized; false if there was an unrecognizable argument.
static bool setSyncMethod( const char* arg, clientData_slOptions* clientData ) {

    // if no argument was supplied, don't do anything at all...
    if( arg == NULL ) return true;

    // otherwise, try to match the argument...
    for (int i = 0; i < ARRAY_SIZE(syncTypeRecs); i++) {
        if (0 == strncmp(arg, syncTypeRecs[i].name, strlen(arg))) {
            clientData->syncMethod = syncTypeRecs[i].type;
            return true;
        }
    }

    // if we get here, we couldn't find the method, so do nothing but return a problem...
    return false;
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


// Attempt to synchronize, using the given method at the current host baud rate.  Returns true if the synchronization
// was successful, false otherwise.
static bool syncSerial( const syncType type, const int fdPort, const int verbosity ) {

    // if our type is ubx, then we have to send messages and see if we get a valid response...
    if( type == syncUBX ) {
        return ubxSynchronize( fdPort, verbosity );
    }

    // otherwise, we're going to sync on a presumed data stream (generally NMEA), so we're just listening...
    else {
        baudRateSynchronizer* syncer = (type == syncNMEA) ? NMEABaudRateSynchronizer : ASCIIBaudRateSynchronizer;
        return synchronize( fdPort, syncer, verbosity );
    }
}

// TODO: better error handling here
// Outputs a fix query, in English or JSON.
static void doFixQuery( const clientData_slOptions* clientData ) {

    pvt_fix fix = {0}; // zeroes all elements...
    reportResponse result = getFix( clientData->fdPort, clientData->verbosity, &fix );
    double msl_height_feet = 0.00328084 * fix.height_above_sea_level_mm;
    double height_accuracy_feet = 0.00328084 * fix.height_accuracy_mm;
    double horizontal_accuracy_feet = 0.00328084 * fix.horizontal_accuracy_mm;
    double speed_mph = fix.ground_speed_mm_s * 0.00223694;
    double speed_accuracy_mph = fix.ground_speed_accuracy_mm_s * 0.00223694;
    if( result == reportOK ) {
        if( clientData->json ) {
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
        printf( "Problem getting fix information from GPS!\n" );
        exit(1);
    }
}

// TODO: better error handling here...
// Outputs a version query, in English or JSON.
static void doVersionQuery( const clientData_slOptions* clientData ) {

    ubxVersion version = {0};  // zeroes all elements...
    reportResponse result = getVersion( clientData->fdPort, clientData->verbosity, &version );

    if( result == reportOK ) {

        if( clientData->json ) {

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
        printf( "Problem getting version information from GPS!\n" );
        exit(1);
    }
}


// Helper function to create error messages with given meta pattern, the subpattern, and arguments embedded.  Returns an
// error response with the given pattern and arguments resolved into a string.  Note that the returned value was
// allocated, and should be freed when no longer needed.  The meta pattern must specify (in order) a %s to contain the
// option identity, and another %s to contain the resolved error message.
static char* msgHelper( char* meta, const optionDef_slOptions* def, char *pattern, va_list args) {

    // first we resolve the pattern we were given...
    char* resolved;
    vasprintf( &resolved, pattern, args );

    // then we insert that into our pattern...
    char* errMsg;
    asprintf( &errMsg, meta , getName_slOptions( def ), resolved );
    free( resolved );
    
    return errMsg;
}


static result_slOptions parseMsgHelper( const optionDef_slOptions* def, resultType_slOptions type, char *pattern, ... ) {

    // get our error message...
    va_list args;
    va_start( args, pattern );
    char* errMsg = msgHelper( "Option parsing problem: [%s] %s", def, pattern, args );
    va_end( args );
    
    // return our bad news...
    result_slOptions result;
    result.type = type;
    result.msg = errMsg;
    return result;
}

// Test the argument to see if it represents a valid serial port.  On failure, return an error.
static result_slOptions parsePort( void* ptrArg, int intArg, const optionDef_slOptions *def, const char *arg, clientData_slOptions *clientData ) {

    int vsd = verifySerialDevice( arg );
    switch( vsd ) {
        case VSD_IS_SERIAL:
            clientData->port = arg;
            result_slOptions result = { resultOk, NULL };
            return result;
        case VSD_NULL:          return parseMsgHelper( def, resultError, "no serial device was specified" );
        case VSD_NOT_TERMINAL:  return parseMsgHelper( def, resultError, "\"%s\" is not a terminal", arg );
        case VSD_CANT_OPEN:     return parseMsgHelper( def, resultError, "\"%s\" cannot be opened", arg );
        case VSD_NONEXISTENT:   return parseMsgHelper( def, resultError, "\"%s\" doesn't exist", arg );
        case VSD_NOT_CHARACTER: return parseMsgHelper( def, resultError, "\"%s\" is not a character device", arg );
        case VSD_NOT_DEVICE:    return parseMsgHelper( def, resultError, "\"%s\" is not a device", arg );
        default:                return parseMsgHelper( def, resultError, "\"%s\" failed for unknown reasons (code %d)", arg, vsd );
    }
}


// Parse the given argument, which is presumed to be a string representing a positive integer that is one of the
// standard baud rates.  If the parsing fails for any reason, return an error
static result_slOptions parseBaud( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // make sure we can parse the entire argument into an integer...
    char *endPtr;
    int baud = (int) strtol( arg, &endPtr, 10 );
    if( *endPtr != 0 ) return parseMsgHelper( def, resultError, "the specified baud rate (\"%s\") is not an integer", arg );

    // make sure the result is a valid baud rate...
    if( getBaudRateCookie( baud ) < 0 )
        return parseMsgHelper( def, resultError, "the specified baud rate (\"%s\") is not a valid one (4800, 9600, 19200, etc.)", arg );

    // we have a valid baud rate, so stuff it away and return in victory...
    *((int*)ptrArg) = baud;
    result_slOptions response = { resultOk, NULL };
    return response;
}


// Parse autobaud, which has an optional argument of ascii, nmea, or ubx.
static result_slOptions parseAutobaud( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // if we get here, we couldn't find the argument...
    if( !setSyncMethod( arg, clientData ) )
        return parseMsgHelper( def, resultError, "the specified baud rate inference method (\"%s\") is unrecognizable; should be \"ascii\", \"nmea\", or \"ubx\"", arg );

    result_slOptions result = { resultOk };
    return result;
}


// Parse sync, which has an optional argument of ascii, nmea, or ubx.
static result_slOptions parseSync( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // if we get here, we couldn't find the argument...
    if( !setSyncMethod( arg, clientData ) )
        return parseMsgHelper( def, resultError, "the specified baud rate synchronization method (\"%s\") is unrecognizable; should be \"ascii\", \"nmea\", or \"ubx\"", arg );

    result_slOptions result = { resultOk };
    return result;
}


// Parse the given argument, which is presumed to be a string representing a yes or no value, which if parsed without
// error is stored to the argPtr. If the parsing fails for any reason, return an error.
static result_slOptions parseBool( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // see if we've got anything that like a yes or true (otherwise, we make it a false)...
    *((bool*)ptrArg) = (arg == NULL) ? false : (strchr( "yYtT1", *arg ) != NULL);

    // then return with no error...
    result_slOptions response = { resultOk, NULL };
    return response;
}


// Parse the given argument, which is presumed to be a string representing an integer, which if parsed without error is
// stored to the argPtr. If the parsing fails for any reason, return an error.
static result_slOptions parseInt( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // make sure we can parse the entire argument into an integer...
    int n = 0;
    if( arg != NULL ) {
        char *endPtr;
        n = (int) strtol(arg, &endPtr, 10);
        if (*endPtr != 0)
            return parseMsgHelper(def, resultError, "the specified %s (\"%s\") is not an integer", def->argName, arg);
    }

    // stuff it away to the given pointer...
    *((int*)ptrArg) = n;

    // then return with no error...
    result_slOptions response = { resultOk, NULL };
    return response;
}


// Set a boolean flag at the given ptrArg to the given intArg.
static result_slOptions parseFlag( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // do what we've been told to do...
    *((bool*)ptrArg) = (bool) intArg;

    // then return with no error...
    result_slOptions response = { resultOk, NULL };
    return response;
}

typedef struct { char* name; queryType queryType; } queryDef;
static queryDef queryDefs[] = {
        { "fix",     fixQuery     },
        { "version", versionQuery }
};


// Parse the given query type and store the results.  Returns an error on any parsing problem.
static result_slOptions parseQuery( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // see what kind of query we've got here...
    for( int i = 0; i < ARRAY_SIZE( queryDefs ); i++ ) {
        if( 0 == strcmp( arg, queryDefs[i].name ) ) {
            clientData->queryType = queryDefs[i].queryType;
            result_slOptions response = { resultOk, NULL };
            return response;
        }
    }

    // if we get here then we have a bogus query type...
    return parseMsgHelper( def, resultError, "the specified %s (\"%s\") is not a valid query type", def->argName, arg );
}


// Increment the int counter at the given ptrArg.
static result_slOptions parseCount( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // do what we've been told to do...
    (*((int*)ptrArg))++;

    // then return with no error...
    result_slOptions response = { resultOk, NULL };
    return response;
}

#undef V3
#undef V2
#undef V1
#undef V0
#undef CD


// Sync may only be specified if auto baud rate was NOT specified...
static result_slOptions  constrainSync( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {
    result_slOptions result = { resultOk };
    if( hasShortOption_slOptions( 'a', state ) ) {
        result.type = resultError;
        result.msg = "-s, --sync may only be specified if -a, --autobaud is NOT specified";
    }
    return result;
}


// JSON may only be specified if a query was specified...
static result_slOptions  constrainJSON( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {
    result_slOptions result = { resultOk };
    if( !hasShortOption_slOptions( 'q', state ) ) {
        result.type = resultError;
        result.msg = "-j, --json may only be specified if -a, --query is also specified";
    }
    return result;
}


// Baud may only be specified if auto baud rate was NOT specified...
static result_slOptions  constrainBaud( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {
    result_slOptions result = { resultOk };
    if( hasShortOption_slOptions( 'a', state ) ) {
        result.type = resultError;
        result.msg = "-b, --baud may only be specified if -a, --autobaud is NOT specified";
    }
    return result;
}


// Minbaud may only be specified if auto baud rate was specified...
static result_slOptions  constrainMinBaud( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {
    result_slOptions result = { resultOk };
    if( !hasShortOption_slOptions( 'a', state ) ) {
        result.type = resultError;
        result.msg = "-M, --minbaud may only be specified if -a, --autobaud is also specified";
    }
    return result;
}

#define CD (config->clientData)
#define V0 (CD->verbosity >= 0)
#define V1 (CD->verbosity >= 1)
#define V2 (CD->verbosity >= 2)
#define V3 (CD->verbosity >= 3)


// Setup action function, which opens and configures the serial port.
static result_slOptions actionSetup( const optionDef_slOptions* defs, const psloConfig* config ) {

    result_slOptions resp = { resultOk };

    // first we try opening the device for the port...
    int fdPort = open( CD->port, O_RDWR | O_NOCTTY | O_NONBLOCK );
    if( fdPort < 0 ) {
        printf( "Failed to open serial port: %s\n", strerror( errno ) );
        exit(1);
    }
    if( V2 ) printf( "Serial port (\"%s\") open...\n", CD->port );

    // now we set the correct options (the configured baud rate, 8 data bits, 1 stop bit, no parity)
    // note that this baud rate may be incorrect, and we might try to infer it in a moment...
    stoResult result = setTermOptions( fdPort, CD->baud, 8, 1, false, false );
    if( result != stoOK ) {
        resp.type = resultError;
        asprintf( &resp.msg, "Error when setting terminal options: %s\n", stoResultStr( result ) );
        close( fdPort );
        return resp;
    }

    // stuff our file descriptor and return in victory...
    CD->fdPort = fdPort;
    if( V1 ) printf( "Serial port open and configured...\n" );
    return resp;

    // TODO: move this code into separate actions for synchronizing or inferring...
//    result = setBaudRate( fdPort, clientData->baud, clientData->synchronize, clientData->autobaud,
//                          NMEABaudRateSynchronizer, clientData->verbose );
//    int expected = (clientData->autobaud || clientData->synchronize) ? SBR_SET_SYNC : SBR_SET_NOSYNC;
//    if( result != expected ) {
//        switch( result ) {
//            case SBR_INVALID:
//                printf( "Baud rate is invalid or not specified!\n" );
//                break;
//            case SBR_FAILED_SYNC:
//                printf( "Failed to synchronize serial port receiver!\n" );
//                break;
//            case SBR_PORT_ERROR:
//                printf( "Error reading from serial port!\n" );
//                break;
//            default:
//                printf( "Unknown error when setting baud rate (code: %d)!\n", result );
//                break;
//        }
//        close( fdPort );
//        exit( 1 );
//    }

}


// Teardown action function, which closes the serial port.
static result_slOptions  actionTeardown(  const optionDef_slOptions* defs, const psloConfig* config ) {

    close( CD->fdPort );
    result_slOptions resp = { resultOk };
    return resp;
}


// Sync action function, which synchronizes serial port receiver with the data stream using the configured method.
static result_slOptions  actionSync(  const optionDef_slOptions* defs, const psloConfig* config ) {

    if( syncSerial( CD->syncMethod, CD->fdPort, CD->verbosity ) ) {
        result_slOptions resp = {resultOk};
        return resp;
    }
    else {
        result_slOptions resp = {resultError};
        resp.msg = "failed to synchronize";
        return resp;
    }
}


// NMEA action function, which turns NMEA data on or off on the U-Blox GPS.
static result_slOptions  actionNMEA(  const optionDef_slOptions* defs, const psloConfig* config ) {

    clientData_slOptions* cd = config->clientData;
    configResponse resp = setNMEAData(cd->fdPort, cd->verbosity, cd->nmea);
    if( resp == configError ) {
        result_slOptions result = {resultError};
        result.msg = "failed to set NMEA data mode";
        return result;
    }
    else {
        result_slOptions result = {resultOk};
        return result;
    }
}


// Query action function, which queries the U-Blox GPS for the specified data.
static result_slOptions  actionQuery(  const optionDef_slOptions* defs, const psloConfig* config ) {

    switch( config->clientData->queryType ) {

        case fixQuery:     doFixQuery(     config->clientData ); break;
        case versionQuery: doVersionQuery( config->clientData ); break;
        default: break;
    }

    result_slOptions resp = {resultOk};
    return resp;
}


// Echo action function, which echoes NMEA data to stdout for a configurable period of time.
static result_slOptions  actionEcho(  const optionDef_slOptions* defs, const psloConfig* config ) {

    if( V2 ) printf( "Starting echo...\n" );

    // figure out our timing...
    long long startTime = currentTimeMs();
    long long maxEchoMS = 1000 * CD->echoSeconds;

    while( (CD->echoSeconds == 0) || ((currentTimeMs() - startTime) < maxEchoMS) ) {
        int c = readSerialChar( CD->fdPort, 1000 );
        if( c >= 0 ) printf( "%c", c );
    }

    if( V2 ) printf( "\nEnding echo...\n" );

    result_slOptions resp = { resultOk };
    return resp;
}


// Change baud rate on GPS and on host port...
static void newBaud( int fdPort, int newBaud, bool verbose ) {

    changeBaudResponse resp = changeBaudRate( fdPort, (unsigned) newBaud, verbose );
    if( resp == ChangeBaudFailure ) {
        printf( "Failed to change baud rate!" );
        exit(1);
    }
}


static optionDef_slOptions* getOptionDefs( const clientData_slOptions* clientData ) {

    optionDef_slOptions echoDef = {
            1, "echo", 'e', argOptional,
            parseInt, (void*) &clientData->echoSeconds, 0,
            NULL,
            actionEcho,
            "seconds",
            "echo human-readable GPS data to stdout for the given time (0 or unspecified means forever)"
    };

    optionDef_slOptions portDef = {
            1, "port", 'p', argRequired,
            parsePort, NULL, 0,
            NULL,
            NULL,
            "device",
            "specify the port device (default is '/dev/serial0')"
    };

    optionDef_slOptions autobaudDef = {
            1, "autobaud", 'a', argOptional,
            parseAutobaud, NULL, 0,
            NULL,
            NULL,
            "method",
            "infer baud rate using ascii, nmea, ubx data (default is ubx)"
    };


    optionDef_slOptions verboseDef = {
            3, "verbose", 'v', argNone,
            parseCount, (void*) &clientData->verbosity, 0,
            NULL,
            NULL,
            NULL,
            "get verbose messages"
    };


    optionDef_slOptions jsonDef = {
            3, "json", 'j', argNone,
            parseFlag, (void*) &clientData->json, 1,
            constrainJSON,
            NULL,
            NULL,
            "select JSON query output"
    };


    optionDef_slOptions baudDef = {
            1, "baud", 'b', argRequired,
            parseBaud, (void*) &clientData->baud, 0,
            constrainBaud,
            NULL,
            "baud rate",
            "specify host baud rate (default is '9600'); any standard rate is allowed"
    };


    optionDef_slOptions minBaudDef = {
            1, "minbaud", 'M', argRequired,
            parseBaud, (void*) &clientData->minBaud, 0,
            constrainMinBaud,
            NULL,
            "baud rate",
            "specify minimum baud rate for autobaud (default is '9600'); any standard rate is allowed"
    };


    optionDef_slOptions newbaudDef = {
            1, "newbaud", 'B', argRequired,
            parseBaud, (void*) &clientData->newbaud, 0,
            NULL,
            NULL,
            "baud rate",
            "change the U-Blox and host baud rates to the specified standard rate"
    };

    optionDef_slOptions queryDef = {
            1, "query", 'q', argRequired,
            parseQuery, NULL, 0,
            NULL,
            actionQuery,
            "query type",
            "query the GPS for info (see \"Query types\" below)"
    };

    optionDef_slOptions nmeaDef = {
            1, "nmea", 'n', argRequired,
            parseBool, (void*) &clientData->nmea, 0,
            NULL,
            actionNMEA,
            "yes/no",
            "configure whether the GPS sends NMEA data to the serial port (see \"Yes or no values\" below)"
    };

    optionDef_slOptions syncDef = {
            1, "sync", 's', argOptional,
            parseSync, NULL, 0,
            constrainSync,
            actionSync,
            "method",
            "synchronize using ascii, nmea, ubx data (default is ubx)"
    };

    optionDef_slOptions optionDefs[] = {
            verboseDef,
            autobaudDef,
            baudDef,
            syncDef,
            newbaudDef,
            minBaudDef,
            portDef,
            nmeaDef,
            jsonDef,
            queryDef,
            echoDef,
            { 0 }
    };

    // allocate memory for our option definitions, and copy them in...
    int numDefs = ARRAY_SIZE( optionDefs );
    optionDef_slOptions* result = safeMalloc( sizeof( optionDefs ) );
    for( int i = 0; i < numDefs; i++ )
        result[i] = optionDefs[i];

    return result;
}

static char* exampleText =
            "Additional information:\n"
            "   Query types:\n"
            "      fix      returns position, altitude, and time information\n"
            "      version  returns hardware and software version of the GPS\n"
            "\n"
            "Examples:\n"
            "   gpsctl --port=/dev/serial1 --baud 9600 --echo=5\n"
            "      Selects the serial port \"/dev/serial1\", sets the host serial port baud rate to 9600, and\n"
            "      echoes any received characters (presumably NMEA data) to stdout.\n"
            "   gpsctl -p /dev/serial1 -b 9600 -e=5\n"
            "      Exactly the same as the preceding example, using short options.\n"
            "\n"
            "Yes or no values:\n"
            "   Yes       indicated by an initial character of 'y', 'Y', 't', 'T', or '1'\n"
            "   No        indicated by anything else\n"
            "";


// The entry point for gpsctl...
int main( int argc, char *argv[] ) {

    // initialize our client data structure, and set defaults...
    clientData_slOptions clientData = { 0 };
    clientData.baud = 9600;
    clientData.minBaud = 9600;
    clientData.port = "/dev/serial0";
    clientData.syncMethod = syncUBX;

    psloConfig config = {
            getOptionDefs( &clientData ),                       // our command-line option definitions...
            &clientData,                                        // our options processing data structure...
            NULL, NULL,                                         // before, after constraint-checking functions...
            actionSetup, actionTeardown,                        // before, after action functions.
            "gpsctl",                                           // name of gpsctl...
            "0.5",                                              // version of gpsctl...
            exampleText,                                        // usage examples...
            SL_OPTIONS_CONFIG_NORMAL                            // slOptions configuration options...
    };

    psloResponse resp = process_slOptions(argc, (const char **) argv, &config );

    if( resp.error ) printf( "Error: %s\n", resp.errMsg );

    exit( EXIT_SUCCESS );
}
