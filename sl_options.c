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

// TODO: add functions to aid constraint checking (e.g. isID())
// TODO: add debug trace (flag or --?)
// TODO: handle case of multiple uses (e.g., -vvv)
// TODO: handle case of last option missing required argument
// TODO: add automated help, usage, version stuff
// TODO: add validation of configuration (no duplicated short or long option names)

#define MAX_SL_OPTIONS 100   // the maximum number of options that slOptions can parse...

typedef struct {
    optionDef_slOptions* def;
    char* arg;
} procOptionRecord;


// Returns a string identifying the option in the given optionDef_slOptions record.  The string may be of the form
// "-l", "--long", or "-l/--long" depending on whether the short, long, or both option names are defined.  The string
// returned has been allocated and should be deleted when no longer needed.
extern char* getName_slOptions( optionDef_slOptions* def ) {

    // figure out how much memory we need for the result...
    size_t size = 0;
    bool shortDefined = (def->shortOpt != 0);
    bool longDefined = (def->longOpt != NULL);
    if( shortDefined ) size += 2;
    if( longDefined ) size += 2 + strlen( def->longOpt );
    if( shortDefined && longDefined ) size++;

    // now allocate the memory and build the result...
    char* result = safeMalloc( size );
    char* ptr = result;
    if( shortDefined ) {
        *ptr++ = '-';
        *ptr++ = def->shortOpt;
    }
    if( shortDefined && longDefined )
        *ptr++ = '/';
    if( longDefined ) {
        *ptr++ = '-';
        *ptr++ = '-';
        strcpy( ptr, def->longOpt );
    }
    return result;
}


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


// Searches through the client-defined options for one with a long option that matches the given string.  If such an
// option is found, a pointer to it is returned - otherwise, NULL is returned.  Note that the string match ONLY matches
// the first 'n' characters of the opt argument, where 'n' is the length of the long option in the definition.  This
// means that opt arguments that include an optional or required option argument will still be matched.
static optionDef_slOptions* findByStringOption( char* opt, const psloConfig* config ) {
    optionDef_slOptions* curDef = config->optionDefs;
    while( curDef->shortOpt != 0 ) {
        if( 0 == strncmp( opt, curDef->longOpt, strlen( curDef->longOpt ) ) )
            return curDef;
        curDef++;
    }
    return NULL;
}


// Returns an error response with the given pattern and arguments resolved into a string.  Note that the errMsg value
// in the response was allocated, and should be freed when no longer needed.
static psloResponse errorMessageHelper( bool debug, char* pattern, ... ) {

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

    if( debug ) printf( errMsg );

    // return our bad news...
    psloResponse response;
    response.error = true;
    response.errMsg = errMsg;
    response.argc = 0;
    response.argv = NULL;
    return response;
}


struct state_slOptions {
    int remainingArgs;                              // the number of arguments we haven't yet processed...
    int currentArgIndex;                            // the index of the argument we're currently working on...
    procOptionRecord optionRecords[MAX_SL_OPTIONS]; // our list of parsed option records...
    int numOptionRecords;                           // the number of parsed option records in optionRecords...
    char** ourArgv;                                 // our local and modifiable copy of the original argv...
    bool debug;                                     // true if debug tracing is on...
};


// If there is sufficient room, and if the maximum number of the given definition wouldn't be exceeed,
// adds the given definition to the list of option definition records parsed.
static psloResponse addOptionRecord( state_slOptions* state, optionDef_slOptions* def ) {

    // make sure we have room to record this one...
    if( state->numOptionRecords >= MAX_SL_OPTIONS ) return errorMessageHelper( state->debug, "exceeded maximum number of options (%d)", MAX_SL_OPTIONS);

    // make sure this definition hasn't already been recorded the max number of times...
    int hits = 0;
    for( int i = 0; i < state->numOptionRecords; i++ )
        if( def == (state->optionRecords[i]).def )
            hits++;
    if( hits >= def->maxCount ) return errorMessageHelper( state->debug, "too many occurrences of option \"%s\": max is %d", getName_slOptions( def ), def->maxCount );

    // ok, all is well - record it...
    (state->optionRecords[state->numOptionRecords]).def = def;
    (state->optionRecords[state->numOptionRecords]).arg = NULL;
    state->numOptionRecords++;
    psloResponse response = {0};
    return response;
}


// Prints the given arguments.
static void printArgs( int argc, char** args ) {
    for( int i = 0; i < argc; i++ ) {
        printf( "%s ", *(args + i) );
    }
}


#define CUR_ARG       (*(state->ourArgv + state->currentArgIndex))
#define LAST_OPT_REC  (state->optionRecords[state->numOptionRecords - 1])
#define REM_ARGS      (state->remainingArgs)
#define CUR_ARG_INDEX (state->currentArgIndex)
#define DEBUG         (state->debug)
static psloResponse parsePhase( int argc, const psloConfig* config, state_slOptions* state ) {

    // in the options processing phase, we need to read each argument once (unless we get stopped by something)...
    while( REM_ARGS > 0 ) {

        if( DEBUG ) printf( "Parsing argument %d (\"%s\")\n", CUR_ARG_INDEX, CUR_ARG );

        // if we're expecting an argument for a previously processed option, handle it...
        if( (state->numOptionRecords > 0) && (LAST_OPT_REC.def->arg == argRequired) && (LAST_OPT_REC.arg == NULL) ) {
            if( DEBUG ) printf( "Assigning argument \"%s\" to option \"%s\"\n", CUR_ARG, getName_slOptions( LAST_OPT_REC.def ) );
            LAST_OPT_REC.arg = CUR_ARG;
            CUR_ARG_INDEX++;
            REM_ARGS--;
        }

        // if this argument is a standalone "--?", and it's the first argument, then we're turning on debug tracing...
        else if( (CUR_ARG_INDEX == 1) && (0 == strcmp( "--?", CUR_ARG )) ) {
            DEBUG = true;
            printf( "Debug tracing turned on by --? option\n" );
            REM_ARGS--;
            CUR_ARG_INDEX++;
        }

        // if this argument is a standalone "--" or "-", stop our processing here...
        else if( (0 == strcmp( "--", CUR_ARG )) || (0 == strcmp( "-", CUR_ARG )) ) {
            REM_ARGS = 0;
            CUR_ARG_INDEX++;
        }

        // if this argument starts with a '-', handle it...
        else if( *CUR_ARG == '-' ) {

            // if the second character is ALSO a "-", the we've got a long option, so handle that...
            if( *(CUR_ARG + 1) == '-' ) {

                char* curOpt = CUR_ARG + 2;  // point to the start of the long option...

                // try to find this option in our option definitions...
                optionDef_slOptions* def = findByStringOption( curOpt, config );

                // if we found it, process it...
                if( def != NULL ) {

                    if( DEBUG ) printf( "Found long option: %s\n", curOpt );

                    // first we add the option record...
                    psloResponse response = addOptionRecord( state, def );
                    if( response.error ) return response;

                    // if we have a suffix (which might be an option argument), handle that...
                    if( strlen( curOpt ) > strlen( def->longOpt ) ) {
                        curOpt += strlen( def->longOpt );  // move to the beginning of the suffix...

                        // if we have an equals sign, then the rest of the suffix is the argument...
                        if( *curOpt == '=' ) {

                            // if the option is allowed to have an argument, assign it...
                            if( def->arg != argNone ) {
                                LAST_OPT_REC.arg = ++curOpt;
                                if( DEBUG ) printf( "Assigning argument \"%s\" to \"%s\" option\n", curOpt, LAST_OPT_REC.def->longOpt );
                            }
                            else
                                return errorMessageHelper( DEBUG, "unexpected argument (\"%s\") for long option (\"%s\")", ++curOpt, def->longOpt );
                        }

                        // otherwise, we've got something that makes no sense...
                        else return errorMessageHelper( DEBUG, "invalid option argument (possibly missing the \"=\"): \"%s\"", curOpt );
                    }
                }
                else
                    return errorMessageHelper( DEBUG, "unrecognized long option (\"%s\") in argument %d", curOpt, CUR_ARG_INDEX );
            }

            // otherwise, we have one or more one-character options - process them one-by-one...
            else {
                char* curOpt = CUR_ARG + 1;
                while( *curOpt != 0 ) {

                    // try to find this option in our option definitions...
                    optionDef_slOptions* def = findByCharOption( *curOpt, config );

                    // if we found it, then process it...
                    if( def != NULL ) {
                        if( DEBUG ) printf( "Found single-character option: %c\n", def->shortOpt );
                        psloResponse response = addOptionRecord( state, def );
                        if( response.error ) return response;

                        // if we have optional or required arguments, handle that...
                        bool haveMore = (*(curOpt + 1) != 0);  // true if the terminator isn't the next character...
                        if( (def->arg != argNone) && haveMore ) {
                            curOpt++;  // get past our option letter...
                            if( *curOpt == '=' ) curOpt++;  // move past the equals sign, if there is one...
                            LAST_OPT_REC.arg = curOpt;
                            if( DEBUG ) printf( "Assigning argument \"%s\" to \"%c\" option\n", curOpt, LAST_OPT_REC.def->shortOpt );
                            curOpt += strlen( curOpt );  // move to the null terminator...
                        }
                        else
                            curOpt++;  // move to the next character in the argument...
                    }

                        // otherwise, someone fed us an option we don't know about...
                    else
                        return errorMessageHelper( DEBUG, "unrecognized single character option (\"%c\") in argument %d", *curOpt, CUR_ARG_INDEX );
                }
            }
            CUR_ARG_INDEX++;
            REM_ARGS--;
        }

        // otherwise, we have no idea what this is, so handle THAT...
        else {

            // if we're in strict mode, we're done...
            if( config->strict ) {
                if( DEBUG ) printf( "Unidentified argument in strict mode; halting option processing\n" );
                REM_ARGS = 0;
                CUR_ARG_INDEX++;
            }

            // otherwise, it's time to permute...
            else {

                if( DEBUG ) printf( "Command line argument not recognized as either an option or an option argument\n" );

                // we only need to do this if there are at least two arguments remaining...
                if( (1 + argc - CUR_ARG_INDEX) >= 2 ) {
                    if( DEBUG ) { printf( "Before permuting: " ); printArgs( argc, state->ourArgv ); printf( "\n" ); }
                    char* thisArg = *(state->ourArgv + CUR_ARG_INDEX);
                    for( int i = CUR_ARG_INDEX + 1; i < argc; i++ ) {
                        *(state->ourArgv + i - 1) = *(state->ourArgv + i);
                    }
                    *(state->ourArgv + argc - 1) = thisArg;
                    if( DEBUG ) { printf( " After permuting: " ); printArgs( argc, state->ourArgv ); printf( "\n" ); }
                }
                REM_ARGS--;
            }
        }
    }

    // return with an error-free response...
    psloResponse response = { argc - CUR_ARG_INDEX, false, NULL, &CUR_ARG };
    return response;
}
#undef DEBUG
#undef REM_ARGS
#undef CUR_ARG_INDEX
#undef CUR_ARG
#undef LAST_OPT_REC


#define CUR_ARG (*(state.ourArgv + state.currentArgIndex))   //(*(state.ourArgv) + state.currentArgIndex)
#define LAST_OPT_REC (state.optionRecords[state.numOptionRecords - 1])
#define DEBUG (state.debug)
extern psloResponse process_slOptions( int argc, const char *argv[], const psloConfig* config ) {

    // make a local (and mutable) copy of the original argv...
    char* argv_copy[argc];
    for( int i = 0; i < argc; i++ ) argv_copy[i] = (char *) argv[i];

    // initialize our state...
    state_slOptions state;              // the current state of options processing...
    state.remainingArgs = argc - 1;     // the number of arguments we haven't yet processed...
    state.currentArgIndex = 1;          // the index of the argument we're currently working on...
    state.numOptionRecords = 0;         // of course we haven't parsed any yet...
    state.ourArgv = (char**)&argv_copy; // a copy of argv that we can modify...
    state.debug = config->debug;        // if the caller asked for debug, he's got it...

    // parse our options, and bail if we got an error...
    psloResponse response = parsePhase( argc, config, &state );
    if( response.error ) return response;

    // when we get here, response.argc is the count of unprocessed arguments pointed to by response.argv...

    // iterate through all the option records, calling the option parsing functions as we go...
    for( int i = 0; i < state.numOptionRecords; i++ ) {
        procOptionRecord* rec = &state.optionRecords[i];
        optionDef_slOptions* def = rec->def;
        if( def->parse != NULL ) {
            parseResult_slOptions result = def->parse( def, rec->arg, config->clientData );
            if( result.rc == optParseError ) {
                return errorMessageHelper( DEBUG, result.msg );
            }
            else if( result.rc == optParseWarning ) {
            }
        }
    }

    return response;
}

#undef DEBUG
#undef LAST_OPT_REC
#undef CUR_ARG
