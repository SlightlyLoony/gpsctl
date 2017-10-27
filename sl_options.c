//
// Created by Tom Dilatush on 10/26/17.
//


// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE

#include "sl_options.h"


#define MAX_SL_OPTIONS 100   // the maximum number of options that slOptions can parse...

typedef struct {
    optionDef_slOptions* def;
    char* arg;
} procOptionRecord;


// Searches through the client-defined options for one with a short option that matches the given character.  If such
// an option is found, a pointer to it is returned - otherwise, NULL is returned.
static optionDef_slOptions* findByCharOption( char opt, const psloConfig* config ) {
    optionDef_slOptions* curDef = config->optionDefs;
    while( curDef->shortOpt != 0 ) {
        if( curDef->shortOpt == opt )
            return curDef;
        curDef++;
    }
    return NULL;
}


// Returns an error response with the given pattern and arguments resolved into a string.  Note that the errMsg value
// in the response was allocated, and should be freed when no longer needed.
static psloResponse errorMessageHelper( char* pattern, ... ) {

    // first we resolve the pattern we were given...
    va_list args;
    va_start( args, pattern );
    char* resolved;
    vasprintf( &resolved, pattern, args );
    va_end( args );

    // then we insert that into our pattern...
    char* errMsg;
    asprintf( &errMsg, "Options parsing error: %s.\n", resolved );
    free( resolved );

    // return our bad news...
    psloResponse response;
    response.error = true;
    response.errMsg = errMsg;
    response.argc = 0;
    response.argv = NULL;
    return response;
}


static bool addOptionRecord( procOptionRecord* records, int* count, optionDef_slOptions* def ) {
    if( *count >= MAX_SL_OPTIONS ) return false;
    (records + *count)->def = def;
    (records + *count)->arg = NULL;
    (*count)++;
    return true;
}


#define CUR_ARG (*(argv + currentArgIndex))
#define LAST_OPT_REC (optionRecords[numOptionRecords - 1])
extern psloResponse process_slOptions( const int argc, char *argv[], const psloConfig* config ) {

    // start off with the first argument after the program name...
    int remainingArgs = argc - 1;                   // the number of arguments we haven't yet processed...
    int currentArgIndex = 1;                        // the index of the argument we're currently working on...

    // set up our processing state...
    psloResponse response = {0};  // initializes all elements to zero...
    procOptionRecord optionRecords[MAX_SL_OPTIONS];  // our list of option records...
    int numOptionRecords = 0;

    // in the options processing phase, we need to read each argument once (unless we get stopped by something)...
    while( remainingArgs > 0 ) {

        // if we're expecting an argument for a previously processed option, handle it...
        if( (numOptionRecords > 0) && (LAST_OPT_REC.def->arg == argRequired) && (LAST_OPT_REC.arg == NULL) ) {
            LAST_OPT_REC.arg = CUR_ARG;
            currentArgIndex++;
            remainingArgs--;
        }

        // if this argument is a standalone "--", stop our processing here...
        else if( 0 == strcmp( "--", CUR_ARG ) ) {
            remainingArgs = 0;
            currentArgIndex++;
        }

        // if this argument starts with a '-', handle it...
        else if( *CUR_ARG == '-' ) {

            // if the argument is JUST a hyphen, then we have a problem...
            size_t argLen = strlen( CUR_ARG );
            if( argLen == 1 ) return errorMessageHelper( "standalone hyphen ('-') in argument %d", currentArgIndex );

            // TODO: check for "--" here as start of long option, move isolated check from above to here

            // ok, we have one or more one-character options - process them one-by-one...
            char* curOpt = CUR_ARG + 1;
            while( *curOpt != 0 ) {

                optionDef_slOptions* def = findByCharOption( *curOpt, config );

                // if we found it, then process it...
                // TODO: we have to process optional arguments here...
                if( def != NULL ) {
                    if( !addOptionRecord( optionRecords, &numOptionRecords, def ) )
                        return errorMessageHelper( "exceeded maximum number of options (%d)", MAX_SL_OPTIONS );
                    curOpt++;
                }

                // otherwise, someone fed us an option we don't know about...
                else return errorMessageHelper( "unrecognized single character option (%c) in argument %d",
                                                *curOpt, currentArgIndex );
            }
            currentArgIndex++;
            remainingArgs--;
        }

        // otherwise, we have no idea what this is, so handle THAT...
        else {

            // if we're in strict mode, we're done...
            if( config->strict ) {
                remainingArgs = 0;
                currentArgIndex++;
            }

            // otherwise, it's time to permute...
            else {

                // we only need to do this if there are at least two arguments remaining...
                if( (1 + argc - currentArgIndex) >= 2 ) {
                    char* thisArg = *argv;
                    for( int i = currentArgIndex + 1; i < argc; i++ ) {
                        *(argv + i - 1) = *(argv + i);
                    }
                    *(argv + argc - 1) = thisArg;
                }
                remainingArgs--;
            }
        }
    }

    // when we get here, currentArgIndex indicates the first argument we didn't process...

    return response;
}

#undef LAST_OPT_REC
#undef CUR_ARG
