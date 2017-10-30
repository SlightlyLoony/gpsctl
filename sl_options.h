//
// Created by Tom Dilatush on 10/26/17.
//

#ifndef GPSCTL_SL_OPTIONS_H
#define GPSCTL_SL_OPTIONS_H

#include <stdio.h>
#include "sl_buffer.h"

typedef enum { argRequired, argOptional,    argNone          } argMode_slOptions;
typedef enum { optParseOk,  optParseError,  optParseWarning  } optParseRC_slOptions;
typedef enum { optCnstrOk,  optCnstrError,  optCnstrWarning  } optCnstrRC_slOptions;
typedef enum { optActionOk, optActionError, optActionWarning } optActionRC_slOptions;

typedef struct clientData_slOptions clientData_slOptions;  // a client-defined structure that typically contains the results of options parsing
typedef struct state_slOptions state_slOptions;  // an slOptions-defined data structure containing the state of slOptions while it's processing

typedef struct {
    optParseRC_slOptions rc;        // the result of processing the option
    char* msg;                      // an error message if the result was not Ok
} parseResult_slOptions;            // the return value from any option parsing function


typedef struct {
    optCnstrRC_slOptions rc;        // the result of constraint checking on the option
    char* msg;                      // an error message if the result was not Ok
} cnstrResult_slOptions;


typedef struct {
    optActionRC_slOptions rc;        // the result of constraint checking on the option
    char* msg;                      // an error message if the result was not Ok
} actionResult_slOptions;

// the response to a process_slOptions() call
typedef struct {
    int argc;       // the number of arguments remaining after processing options
    bool error;     // true if a fatal error occurred in processing
    char* errMsg;   // a string describing the error (always allocated)
    char** argv;    // a pointer to the remaining arguments (NULL if argc is zero)
} psloResponse;

typedef struct optionDef_slOptions optionDef_slOptions;

typedef struct psloConfig psloConfig;

typedef parseResult_slOptions  parse_slOptions(  const optionDef_slOptions* def, char* arg, clientData_slOptions* );  // option parsing function def
typedef cnstrResult_slOptions  cnstr_slOptions(  char, clientData_slOptions* );  // constraint checking function def
typedef actionResult_slOptions action_slOptions( const state_slOptions* state, const psloConfig* config, char* arg );  // action function def

struct optionDef_slOptions {
    unsigned int            maxCount;   // how many times this option may appear in a command line
    char*                   longOpt;    // optional; the long form of the option
    char                    shortOpt;   // optional; the short form of the option
    argMode_slOptions       arg;        // controls whether the option has an argument, required or optional
    parse_slOptions*        parse;      // option parsing function, or NULL if not needed
    cnstr_slOptions*        cnstr;      // constraint checking function, or NULL if not needed
    action_slOptions*       action;     // action function, or NULL if not needed
    char*                   argName;    // short name of argument (REQUIRED if there could be an argument )
    char*                   helpMsg;    // help message for the option (REQUIRED)

};


#define SL_OPTIONS_CONFIG_DEBUG       (0x0001)   // enables debug tracing
#define SL_OPTIONS_CONFIG_STRICT      (0x0002)   // enables strict mode (no permutation)
#define SL_OPTIONS_CONFIG_ADD_USAGE   (0x0004)   // enables automatically adding usage option definition
#define SL_OPTIONS_CONFIG_ADD_HELP    (0x0008)   // enables automatically adding help option definition
#define SL_OPTIONS_CONFIG_ADD_VERSION (0x0010)   // enables automatically adding verson option definition
#define SL_OPTIONS_CONFIG_NORMAL      (SL_OPTIONS_CONFIG_ADD_USAGE | SL_OPTIONS_CONFIG_ADD_HELP | SL_OPTIONS_CONFIG_ADD_VERSION)

// the configuration for a process_slOptions() call
struct psloConfig {
    optionDef_slOptions* optionDefs;    // pointer to client-defined option definitions
    clientData_slOptions* clientData;   // pointer to client-defined data structure
    char* name;                         // a client-defined program name
    char* version;                      // a client-defined string representing the version of the program
    int options;                        // options for the behavior of slOptions
};


psloResponse process_slOptions( int argc, const char *argv[], const psloConfig* config );
char* getName_slOptions( const optionDef_slOptions* );

#endif //GPSCTL_SL_OPTIONS_H
