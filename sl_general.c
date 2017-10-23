//
// Created by Tom Dilatush on 10/19/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#define _DARWIN_C_SOURCE
#define _POSIX_C_SOURCE 199309L

#include <memory.h>
#include <stdlib.h>
#include <execinfo.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/stat.h>
#include <time.h>
#include <errno.h>
#include <ctype.h>
#include "sl_general.h"


extern sl_buffer allocateSlBuffer( size_t size ) {
    sl_buffer result;
    result.buffer = safeMalloc( size );
    result.size = size;
    result.allocated = true;
    return result;
}


extern void freeSlBuffer( sl_buffer buffer ) {
    if( buffer.allocated ) return;
    free( buffer.buffer );
    buffer.buffer = NULL;
}


extern void printSlBuffer( sl_buffer buffer ) {
    printf( ":");
    for( int i = 0; i < buffer.size; i++ )
        printf( "%02x:", buffer.buffer[i] );
    printf( "\n" );
}


// Dumps a stack trace to stderr.
extern void stackDump( void ) {
    #define STACK_DUMP_SIZE 250
    void *stackFrames[STACK_DUMP_SIZE];
    char **strings;

    int numFrames = backtrace( stackFrames, STACK_DUMP_SIZE );
    backtrace_symbols_fd( stackFrames, numFrames, STDERR_FILENO );
}


// A safe replacement for malloc that checks the result.  On failure, prints an error message and stack trace
// to stderr.  Otherwise, it behaves EXACTLY like malloc().
extern void *safeMalloc( size_t size ) {
    void *result = malloc( size );
    if( !result ) {
        fprintf( stderr, "ERROR: malloc() failed\n" );
        stackDump();
        exit(1);
    }
    return result;
}


// Return the concatenation of the two given string in a newly allocated memory on the heap.  If the two
// given strings are null, a null is returned.  If one of the given strings is null, then a COPY (in newly allocated
// memory) of the non-null given string is returned.  Note that this behavior means that ANY non-null return value
// represents a block of memory that must be freed by the caller.  If malloc fails, a stack trace is dumped to stderr
// and the program is aborted.
extern char* concat( const char *s1, const char *s2 ) {

    // get the lengths just once (optimization)...
    const size_t len1 = s1 ? strlen(s1) : 0;
    const size_t len2 = s2 ? strlen(s2) : 0;

    // if both arguments were null, return with a null...
    if( len1 + len2 == 0 ) return NULL;

    // get the memory required for the concatenated strings plus the terminator...
    char *result = safeMalloc( len1 + len2 + 1 );

    // build our result...
    memcpy(result, s1, len1);
    memcpy(result + len1, s2, len2 + 1 ); //+1 to copy the null-terminator

    return result;
}


extern long long max_ll( long long a, long long b ) {
    return (a > b) ? a : b;
}


// Returns the time since the epoch in milliseconds.
extern long long currentTimeMs() {
    struct timespec tp;
    int result = clock_gettime( CLOCK_MONOTONIC, &tp );
    return tp.tv_sec * 1000L + tp.tv_nsec / 1000000L;
}


// Sleep for the given number of nanoseconds.
extern void sleep_ns( int ns ) {
    struct timespec nanos;
    nanos.tv_sec = 0;
    nanos.tv_nsec = ns;
    nanosleep( &nanos, NULL );
}

// Returns [0..15] if the given character is a valid hex digit, -1 otherwise.
extern int hex2int( char c ) {
    if( !isxdigit( c ) ) return -1;
    return ((c >= '0') && (c <= '9')) ? (c - '0') : (0x0F & (c + 9));
}
