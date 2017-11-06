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
// TODO: add filter for NMEA message types on echo, also message verifier (only echo good messages)?
// TODO: troll all headers, inserting parameter names and return value comments
// TODO: do save config to get a backup

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
#include "sl_return.h"
#include "sl_options.h"
#include "sl_general.h"
#include "sl_serial.h"
#include "ublox.h"
#include "cJSON.h"

typedef enum { fixQuery, versionQuery, configQuery } queryType;
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


// Set the sync method to that specified in the argument.  Returns Ok if there's no argument (and do nothing) or the
// argument was recognized; or an Error if there was an unrecognizable argument.
static slReturn setSyncMethod( const char* arg, clientData_slOptions* clientData ) {

    // if no argument was supplied, don't do anything at all...
    if( arg == NULL ) return makeOkReturn();

    // otherwise, try to match the argument...
    for (int i = 0; i < ARRAY_SIZE(syncTypeRecs); i++) {
        if (0 == strncmp(arg, syncTypeRecs[i].name, strlen(arg))) {
            clientData->syncMethod = syncTypeRecs[i].type;
            return makeOkReturn();
        }
    }

    // if we get here, we couldn't find the method, so do nothing but return a problem...
    return makeErrorFmtMsgReturn( ERR_ROOT, "\"%s\" is not a recognized synchronization method", arg );
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


// Attempt to synchronize, using the given method at the current host baud rate.  On an Ok response, returns true
// or false for synchronization success as a bool in additional info.
static slReturn syncSerial( const syncType type, const int fdPort, const int verbosity ) {

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


// Outputs a fix query, in English or JSON.
static slReturn doFixQuery( const clientData_slOptions* clientData ) {

    ubxFix fix = {0}; // zeroes all elements...
    slReturn result = getFix( clientData->fdPort, clientData->verbosity, &fix );
    double msl_height_feet = 0.00328084 * fix.height_above_sea_level_mm;
    double height_accuracy_feet = 0.00328084 * fix.height_accuracy_mm;
    double horizontal_accuracy_feet = 0.00328084 * fix.horizontal_accuracy_mm;
    double speed_mph = fix.ground_speed_mm_s * 0.00223694;
    double speed_accuracy_mph = fix.ground_speed_accuracy_mm_s * 0.00223694;
    if( isErrorReturn( result ) )
        return makeErrorMsgReturn( ERR_CAUSE( result ), "Problem obtaining fix from GPS" );

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
        printf( " Satellites: %d used for computing this fix\n", fix.number_of_satellites_used );
        printf( "   Accuracy: time (%d ns), height (+/-%.3f feet), position (+/-%.3f feet), heading(+/-%.3f degrees), speed(+/-%.3f mph)\n",
                fix.time_accuracy_ns, height_accuracy_feet, horizontal_accuracy_feet, fix.heading_accuracy_deg, speed_accuracy_mph );
    }

    return makeOkReturn();
}


// Outputs a version query, in English or JSON.
static slReturn doVersionQuery( const clientData_slOptions* clientData ) {

    ubxVersion version = {0};  // zeroes all elements...
    slReturn result = getVersion( clientData->fdPort, clientData->verbosity, &version );
    if( isErrorReturn( result ) )
        return makeErrorMsgReturn(ERR_CAUSE( result ), "Problem obtaining version information from GPS" );

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

    return makeOkReturn();
}


// Outputs a configuration query, in English or JSON.
static slReturn doConfigQuery( const clientData_slOptions* clientData ) {

    ubxConfig config = {};  // zeroes all elements...
    slReturn result = getConfig( clientData->fdPort, clientData->verbosity, &config );
    if( isErrorReturn( result ) )
        return makeErrorMsgReturn(ERR_CAUSE( result ), "Problem obtaining configuration information from GPS" );

    if( clientData->json ) {
//
//        cJSON *root;
//        root = cJSON_CreateObject();
//        cJSON_AddItemToObject( root, "software_version", cJSON_CreateString( version.software ) );
//        cJSON_AddItemToObject( root, "hardware_version", cJSON_CreateString( version.hardware ) );
//        cJSON_AddItemToObject( root, "extensions",
//                               cJSON_CreateStringArray( (const char**) version.extensions, version.number_of_extensions ) );
//
//        char *jsonStr = cJSON_PrintUnformatted( root );
//        printf( "%s\n", jsonStr );
//
//        cJSON_Delete( root );
    }
    else {
//        printf( "Software version: %s\n", version.software );
//        free( version.software );
//        printf( "Hardware version: %s\n", version.hardware );
//        free( version.hardware );
//        char **ptr = version.extensions;
//        while( (*ptr) != NULL ) {
//            printf( "       Extension: %s\n", *ptr );
//            free( (*ptr) );
//            ptr++;
//        }
    }

    return makeOkReturn();
}


static slReturn parseMsgHelper( errorInfo_slReturn error, const optionDef_slOptions* def, char *pattern, ... ) {

    char* resolved;

    // first resolve the pattern we were given...
    va_list args;
    va_start( args, pattern );
    vasprintf( &resolved, pattern, args );
    va_end( args );
    
    // return our bad news...
    slReturn result = makeErrorFmtMsgReturn( error, "option parsing problem: [%s] %s", getName_slOptions( def ), resolved );
    free( resolved );
    return result;
}

// Test the argument to see if it represents a valid serial port.  On failure, return an error.
static slReturn parsePort( void* ptrArg, int intArg, const optionDef_slOptions *def, const char *arg, clientData_slOptions *clientData ) {

    slReturn vsd = verifySerialDevice( arg );
    if( isErrorReturn( vsd ) )
        return makeErrorFmtMsgReturn(ERR_CAUSE( vsd ), "device \"%s\" is not a terminal", arg );

    return makeOkReturn();
}


// Parse the given argument, which is presumed to be a string representing a positive integer that is one of the
// standard baud rates.  If the parsing fails for any reason, return an error
static slReturn parseBaud( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // make sure we can parse the entire argument into an integer...
    char *endPtr;
    int baud = (int) strtol( arg, &endPtr, 10 );
    if( *endPtr != 0 )
        return parseMsgHelper(ERR_ROOT, def, "the specified baud rate (\"%s\") is not an integer", arg );

    // make sure the result is a valid baud rate...
    if( getBaudRateCookie( baud ) < 0 )
        return parseMsgHelper(ERR_ROOT, def, "the specified baud rate (\"%s\") is not a valid one (4800, 9600, 19200, etc.)", arg );

    // we have a valid baud rate, so stuff it away and return in victory...
    *((int*)ptrArg) = baud;
    return makeOkReturn();
}


// Parse autobaud or sync, which have an optional argument of ascii, nmea, or ubx.
static slReturn parseSyncMethod( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // if we get here, we couldn't find the argument...
    slReturn ssmResp = setSyncMethod( arg, clientData );
    if( isErrorReturn( ssmResp ) )
        return parseMsgHelper(ERR_CAUSE( ssmResp ), def, "the specified baud rate inference method (\"%s\") is unrecognizable; should be \"ascii\", \"nmea\", or \"ubx\"", arg );

    return makeOkReturn();
}


// Parse the given argument, which is presumed to be a string representing a yes or no value, which if parsed without
// error is stored to the argPtr. If the parsing fails for any reason, return an error.
static slReturn parseBool( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // see if we've got anything that like a yes or true (otherwise, we make it a false)...
    *((bool*)ptrArg) = (arg == NULL) ? false : (strchr( "yYtT1", *arg ) != NULL);

    // then return with no error...
    return makeOkReturn();
}


// Parse the given argument, which is presumed to be a string representing an integer, which if parsed without error is
// stored to the argPtr. If the parsing fails for any reason, return an error.
static slReturn parseInt( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // make sure we can parse the entire argument into an integer...
    int n = 0;
    if( arg != NULL ) {
        char *endPtr;
        n = (int) strtol(arg, &endPtr, 10);
        if (*endPtr != 0)
            return parseMsgHelper(ERR_ROOT, def, "the specified %s (\"%s\") is not an integer", def->argName, arg);
    }

    // stuff it away to the given pointer...
    *((int*)ptrArg) = n;

    // then return with no error...
    return makeOkReturn();
}


// Set a boolean flag at the given ptrArg to the given intArg.
static slReturn parseFlag( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // do what we've been told to do...
    *((bool*)ptrArg) = (bool) intArg;

    // then return with no error...
    return makeOkReturn();
}

typedef struct { char* name; queryType queryType; } queryDef;
static queryDef queryDefs[] = {
        { "fix",     fixQuery     },
        { "version", versionQuery },
        { "config",  configQuery  }
};


// Parse the given query type and store the results.  Returns an error on any parsing problem.
static slReturn parseQuery( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // see what kind of query we've got here...
    for( int i = 0; i < ARRAY_SIZE( queryDefs ); i++ ) {
        if( 0 == strcmp( arg, queryDefs[i].name ) ) {
            clientData->queryType = queryDefs[i].queryType;
            return makeOkReturn();
        }
    }

    // if we get here then we have a bogus query type...
    return parseMsgHelper(ERR_ROOT, def, "the specified %s (\"%s\") is not a valid query type", def->argName, arg );
}


// Increment the int counter at the given ptrArg.
static slReturn parseCount( void* ptrArg, int intArg, const optionDef_slOptions* def, const char* arg, clientData_slOptions* clientData ) {

    // do what we've been told to do...
    (*((int*)ptrArg))++;

    // then return with no error...
    return makeOkReturn();
}

#undef V3
#undef V2
#undef V1
#undef V0
#undef CD


// Sync may only be specified if auto baud rate was NOT specified...
static slReturn  constrainSync( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {

    if( hasShortOption_slOptions( 'a', state ) )
        return makeErrorMsgReturn(ERR_ROOT, "-s, --sync may only be specified if -a, --autobaud is NOT specified" );

    return makeOkReturn();
}


// JSON may only be specified if a query was specified...
static slReturn  constrainJSON( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {

    if( !hasShortOption_slOptions( 'q', state ) )
        return makeErrorMsgReturn(ERR_ROOT, "-j, --json may only be specified if -a, --query is also specified" );

    return makeOkReturn();
}


// Baud may only be specified if auto baud rate was NOT specified...
static slReturn  constrainBaud( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {

    if( hasShortOption_slOptions( 'a', state ) )
        return makeErrorMsgReturn(ERR_ROOT, "-b, --baud may only be specified if -a, --autobaud is NOT specified" );

    return makeOkReturn();
}


// Minbaud may only be specified if auto baud rate was specified...
static slReturn  constrainMinBaud( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {

    if( !hasShortOption_slOptions( 'a', state ) )
        return makeErrorMsgReturn(ERR_ROOT, "-M, --minbaud may only be specified if -a, --autobaud is also specified" );

    return makeOkReturn();
}


// Verbosity may only be specified if quiet was NOT specified...
static slReturn  constrainVerbosity( const optionDef_slOptions* defs, const psloConfig* config, const state_slOptions* state ) {

    if( hasShortOption_slOptions( 'q', state ) )
        return makeErrorMsgReturn(ERR_ROOT, "-v, --verbosity d may only be specified if -q, --quiet is NOT specified" );

    return makeOkReturn();
}

#define CD (config->clientData)
#define V0 (CD->verbosity >= 0)
#define V1 (CD->verbosity >= 1)
#define V2 (CD->verbosity >= 2)
#define V3 (CD->verbosity >= 3)


// Setup action function, which opens and configures the serial port.
static slReturn actionSetup( const optionDef_slOptions* defs, const psloConfig* config ) {

    // first we try opening the device for the port...
    int fdPort = open( CD->port, O_RDWR | O_NOCTTY | O_NONBLOCK );
    if( fdPort < 0 ) {
        printf( "Failed to open serial port: %s\n", strerror( errno ) );
        exit(1);
    }
    if( V2 ) printf( "Serial port (\"%s\") open...\n", CD->port );

    // now we set the correct options (the configured baud rate, 8 data bits, 1 stop bit, no parity)
    // note that this baud rate may be incorrect, and we might try to infer it in a moment...
    slReturn result = setTermOptions( fdPort, CD->baud, 8, 1, false, false );
    if( isErrorReturn( result ) ) {
        close( fdPort );
        return makeErrorMsgReturn(ERR_CAUSE( result ), "Error when setting terminal options" );
    }

    // stuff our file descriptor and return in victory...
    CD->fdPort = fdPort;
    if( V2 ) printf( "Serial port open and configured...\n" );
    return makeOkReturn();

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
static slReturn actionTeardown(  const optionDef_slOptions* defs, const psloConfig* config ) {

    close( CD->fdPort );
    return makeOkReturn();
}


// Sync action function, which synchronizes serial port receiver with the data stream using the configured method.
static slReturn actionSync(  const optionDef_slOptions* defs, const psloConfig* config ) {

    slReturn ssResp = syncSerial( CD->syncMethod, CD->fdPort, CD->verbosity );
    if( isErrorReturn( ssResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( ssResp ), "error while synchronizing on serial data" );

    bool success = getReturnInfoChar( ssResp );
    if( success )
        return makeOkReturn();
    else
        return makeErrorMsgReturn(ERR_ROOT, "failed to synchronize" );
}


// NMEA action function, which turns NMEA data on or off on the U-Blox GPS.
static slReturn  actionNMEA(  const optionDef_slOptions* defs, const psloConfig* config ) {

    clientData_slOptions* clientData = config->clientData;
    slReturn usResp = ubxSynchronize( clientData->fdPort, clientData->verbosity );
    if( isErrorReturn( usResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( usResp ), "could not synchronize UBX protocol" );

    slReturn resp = setNMEAData( clientData->fdPort, clientData->verbosity, clientData->nmea );
    if( isErrorReturn( resp ) )
        return makeErrorFmtMsgReturn(ERR_CAUSE( resp ), "failed to turn NMEA data %s", clientData->nmea ? "on" : "off" );

    return makeOkReturn();
}


// Query action function, which queries the U-Blox GPS for the specified data.
static slReturn  actionQuery(  const optionDef_slOptions* defs, const psloConfig* config ) {

    clientData_slOptions* clientData = config->clientData;
    slReturn usResp = ubxSynchronize( clientData->fdPort, clientData->verbosity );
    if( isErrorReturn( usResp ) )
        return makeErrorMsgReturn( ERR_CAUSE( usResp ), "could not synchronize UBX protocol" );

    slReturn resp;
    switch( clientData->queryType ) {

        case fixQuery:     resp = doFixQuery(     clientData ); break;
        case versionQuery: resp = doVersionQuery( clientData ); break;
        case configQuery:  resp = doConfigQuery(  clientData ); break;
        default: return makeErrorFmtMsgReturn(ERR_ROOT, "invalid query type: %d", clientData->queryType );
    }
    if( isErrorReturn( resp ) )
        return makeErrorMsgReturn(ERR_CAUSE( resp ), "problem executing GPS query" );
    return makeOkReturn();
}


// Echo action function, which echoes NMEA data to stdout for a configurable period of time.
static slReturn  actionEcho(  const optionDef_slOptions* defs, const psloConfig* config ) {

    if( V2 ) printf( "Starting echo...\n" );

    // figure out our timing...
    long long startTime = currentTimeMs();
    long long maxEchoMS = 1000 * CD->echoSeconds;

    while( (CD->echoSeconds == 0) || ((currentTimeMs() - startTime) < maxEchoMS) ) {
        slReturn rscResp = readSerialChar( CD->fdPort, 1000 );
        if( isErrorReturn( rscResp ) )
            return makeErrorMsgReturn( ERR_CAUSE( rscResp ), "couldn't read character while echoing" );
        if( isWarningReturn( rscResp ) )
            continue;  // we timed out; just try again...
        int c = getReturnInfoChar( rscResp );
        if( c >= 0 ) printf( "%c", c );
    }

    if( V2 ) printf( "\nEnding echo...\n" ); else printf( "\n" );

    return makeOkReturn();
}


// Quiet action function, which simply sets verbosity to zero.
static slReturn  actionQuiet(  const optionDef_slOptions* defs, const psloConfig* config ) {
    config->clientData->verbosity = 0;
    return makeOkReturn();
}


// Change baud rate on GPS and on host port...
static slReturn newBaud( int fdPort, int newBaud, bool verbose ) {

    slReturn resp = changeBaudRate( fdPort, (unsigned) newBaud, verbose );
    if( isErrorReturn( resp ) )
        return makeErrorMsgReturn(ERR_CAUSE( resp ), "failed to change baud rate" );

    return makeOkReturn();
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
            parseSyncMethod, NULL, 0,
            NULL,
            NULL,
            "method",
            "infer baud rate using ascii, nmea, ubx data (default is ubx)"
    };


    optionDef_slOptions verboseDef = {
            2, "verbose", 'v', argNone,
            parseCount, (void*) &clientData->verbosity, 0,
            constrainVerbosity,
            NULL,
            NULL,
            "get verbose messages"
    };


    optionDef_slOptions quietDef = {
            1, "quiet", 'q', argNone,
            NULL, NULL, 0,
            NULL,
            actionQuiet,
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
            1, "query", 'Q', argRequired,
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
            parseSyncMethod, NULL, 0,
            constrainSync,
            actionSync,
            "method",
            "synchronize using ascii, nmea, ubx data (default is ubx)"
    };

    optionDef_slOptions optionDefs[] = {
            verboseDef,
            quietDef,
            portDef,
            jsonDef,
            autobaudDef,
            baudDef,
            minBaudDef,
            syncDef,
            newbaudDef,
            nmeaDef,
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
    "\n"
    "  Query types:\n"
    "    fix      returns position, altitude, and time information\n"
    "    version  returns hardware and software version of the GPS\n"
    "    config   returns GPS configuration information\n"
    "\n"
    "  Yes or no values:\n"
    "    yes      indicated by an initial character of 'y', 'Y', 't', 'T', or '1'\n"
    "    no       indicated by anything else\n"
    "\n"
    "Examples:\n"
    "   gpsctl --port=/dev/serial1 --baud 9600 --echo=5\n"
    "      Selects the serial port \"/dev/serial1\", sets the host serial port baud rate to 9600, and\n"
    "      echoes any received characters (presumably NMEA data) to stdout.\n"
    "   gpsctl -p /dev/serial1 -b 9600 -e=5\n"
    "      Exactly the same as the preceding example, using short options.\n"
    "";


// The entry point for gpsctl...
int main( int argc, char *argv[] ) {

    // initialize our client data structure, and set defaults...
    clientData_slOptions clientData = { 0 };
    clientData.baud = 9600;
    clientData.minBaud = 9600;
    clientData.port = "/dev/serial0";
    clientData.syncMethod = syncUBX;
    clientData.verbosity = 1;

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

    slReturn resp = process_slOptions(argc, (const char **) argv, &config );

    if( isErrorReturn( resp ) ) printReturn( resp, true, true );

    freeReturn( resp );

    exit( EXIT_SUCCESS );
}
