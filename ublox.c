//
// Created by Tom Dilatush on 10/20/17.
//

// these defines allow compiling on both an OS X development machine and the target Raspberry Pi.  If different
// development environments or target machines are needed, these will likely need to be tweaked.
#define _XOPEN_SOURCE 700
#ifdef Macintosh
#define _DARWIN_C_SOURCE
#endif /* Macintosh */
#define _POSIX_C_SOURCE 199309L

#include "ublox.h"
