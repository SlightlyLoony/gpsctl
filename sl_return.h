//
// Created by Tom Dilatush on 11/3/17.
//

#ifndef GPSCTL_SL_RESULT_H
#define GPSCTL_SL_RESULT_H

#include <stdint.h>
#include <stdbool.h>

// opaque type for function return values...
typedef void* slReturn;         // managed by slReturn internally; users shouldn't do anything with it directly...

// contains error information other than client-generated error message...
typedef struct {
    slReturn cause;
    char* fileName;
    uint32_t lineNumber;
} errorInfo_slReturn;


errorInfo_slReturn makeErrorInfo( char*, uint32_t, slReturn );
bool isOkReturn( slReturn );
bool isErrorReturn( slReturn );
bool isWarningReturn( slReturn );
const char* getReturnMsg( slReturn );
void* getReturnInfo( slReturn );
void printReturnChain( slReturn );
void freeReturn( slReturn  );
slReturn makeOkReturn();
slReturn makeErrorReturn( errorInfo_slReturn );
slReturn makeWarningReturn();
slReturn makeOkMsgReturn( const char* );
slReturn makeErrorMsgReturn( errorInfo_slReturn, const char* );
slReturn makeWarningMsgReturn( const char* );
slReturn makeOkFmtMsgReturn( const char*, ... );
slReturn makeErrorFmtMsgReturn( errorInfo_slReturn, const char*, ... );
slReturn makeWarningFmtMsgReturn( const char*, ... );
slReturn makeOkInfoReturn( void* );
slReturn makeErrorInfoReturn( errorInfo_slReturn, void* );
slReturn makeWarningInfoReturn( void* );
slReturn makeOkInfoMsgReturn( void*, const char* );
slReturn makeErrorInfoMsgReturn( errorInfo_slReturn, void*, const char* );
slReturn makeWarningInfoMsgReturn( void*, const char* );
slReturn makeOkInfoFmtMsgReturn( void*, const char*, ... );
slReturn makeErrorInfoFmtMsgReturn( errorInfo_slReturn, void*, const char*, ... );
slReturn makeWarningInfoFmtMsgReturn( void*, const char*, ... );

#define ERRINFO(x) (makeErrorInfo( __FILE__, __LINE__, (x) ) )

#endif //GPSCTL_SL_RESULT_H
