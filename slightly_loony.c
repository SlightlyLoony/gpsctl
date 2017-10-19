//
// Created by Tom Dilatush on 10/19/17.
//

#include <memory.h>
#include <stdlib.h>
#include "slightly_loony.h"

char* concat( const char *s1, const char *s2 ) {
    const size_t len1 = strlen(s1);
    const size_t len2 = strlen(s2);
    char *result = malloc( len1 + len2 + 1 );  //+1 for the null-terminator
    memcpy(result, s1, len1);
    memcpy(result + len1, s2, len2 + 1 ); //+1 to copy the null-terminator
    return result;
}