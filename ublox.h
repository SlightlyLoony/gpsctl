//
// Created by Tom Dilatush on 10/20/17.
//

#ifndef GPSCTL_UBLOX_H
#define GPSCTL_UBLOX_H

#include "sl_return.h"

typedef enum ubxState    { Valid, NotValid, NotPresent } ubxState;

typedef struct ubxChecksum {
    byte ck_a;
    byte ck_b;
} ubxChecksum;

typedef struct ubxType {
    byte class;
    byte id;
} ubxType;

typedef struct {
    int          year;
    int          month;
    int          day;
    int          hour;
    int          minute;
    double       second;
    int          nanoseconds_correction;
    unsigned int time_accuracy_ns;
    bool         time_resolved;
    bool         date_valid;
    bool         time_valid;
    bool         fix_is_3d;
    bool         fix_valid;
    unsigned int number_of_satellites_used;
    double       longitude_deg;
    double       latitude_deg;
    int          height_above_ellipsoid_mm;
    int          height_above_sea_level_mm;
    unsigned int height_accuracy_mm;
    unsigned int horizontal_accuracy_mm;
    int          ground_speed_mm_s;
    unsigned int ground_speed_accuracy_mm_s;
    double       heading_deg;
    double       heading_accuracy_deg;

} ubxFix;

typedef struct {
    char *software;
    char *hardware;
    char **extensions;  // terminated by NULL entry...
    int number_of_extensions;
} ubxVersion;

typedef struct {

} ubxConfig;

slReturn getVersion( int fdPort, int verbosity, ubxVersion *version );
slReturn getFix( int fdPort, int verbosity, ubxFix *fix );
slReturn getConfig( int fdPort, int verbosity, ubxConfig *config );
slReturn changeBaudRate( int fdPort, unsigned int newBaudRate, int verbosity );
slReturn setNMEAData( int fdPort, int verbosity, bool nmeaOn );
slReturn ubxSynchronize( const int, const int );

#endif //GPSCTL_UBLOX_H
