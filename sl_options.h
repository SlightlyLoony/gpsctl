//
// Created by Tom Dilatush on 10/26/17.
//

#ifndef GPSCTL_SL_OPTIONS_H
#define GPSCTL_SL_OPTIONS_H

#include <stdio.h>
#include "sl_buffer.h"

typedef enum { argRequired, argOptional, argNone }          argMode_slOptions;
typedef enum { optProcNULL, optProcOk, optProcError, optProcWarning }    optProcRC_slOptions;
typedef enum { optCnstrNULL, optCnstrOk, optCnstrError, optCnstrWarning } optCnstrRC_slOptions;

typedef struct clientData_slOptions clientData_slOptions;  // a client-defined structure that typically contains the results of options parsing

typedef struct {
    optProcRC_slOptions rc;         // the result of processing the option
    slBuffer* msg;                  // an error message if the result was not Ok
} procResult_slOptions;  // the return value from any option processing function

typedef struct {
    optCnstrRC_slOptions rc;   // the result of constraint checking on the option
    slBuffer* msg;                  // an error message if the result was not Ok
} cnstrResult_slOptions;

// the response to a process_slOptions() call
typedef struct {
    int argc;       // the number of arguments remaining after processing options
    bool error;     // true if a fatal error occurred in processing
    char* errMsg;   // a string describing the error (always allocated)
    char** argv;    // a pointer to the remaining arguments (NULL if argc is zero)
} psloResponse;

typedef procResult_slOptions proc_slOptions( char, char* arg, clientData_slOptions* );  // option processing function def
typedef cnstrResult_slOptions cnstr_slOptions( char, clientData_slOptions* );  // constraint checking function def

typedef struct {
    char*                   longOpt;    // optional; the long form of the option
    char                    shortOpt;   // optional; the short form of the option
    char                    id;         // a unique identifier for the option
    argMode_slOptions       arg;        // controls whether the option has an argument, required or optional
    proc_slOptions*         proc;       // option processing function, or NULL if not needed
    cnstr_slOptions*        cnstr;      // constraint checking function, or NULL if not needed

} optionDef_slOptions;

// the configuration for a process_slOptions() call
typedef struct {
    optionDef_slOptions* optionDefs;    // pointer to client-defined option definitions
    clientData_slOptions* clientData;   // pointer to client-defined data structure
    bool strict;                        // true to disable argument permutation
} psloConfig;

psloResponse process_slOptions( const int argc, char *argv[], const psloConfig* config );

#endif //GPSCTL_SL_OPTIONS_H
