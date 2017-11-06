//
// Created by Tom Dilatush on 10/20/17.
//

#ifndef GPSCTL_UBLOX_H
#define GPSCTL_UBLOX_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/termios.h>
#include "sl_general.h"
#include "sl_bits.h"
#include "sl_buffer.h"
#include "sl_return.h"
#include "sl_serial.h"

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

typedef enum { GPS, SBAS, Galileo, BeiDou, IMES, QZSS, GLONASS } gnssID;
typedef enum { Portable, Stationary = 2, Pedestrian, Automotive, Sea, Air1G, Air2G, Air4G, Watch } dynModel;
typedef enum { Only2D = 1, Only3D, Auto2D3D } fixMode;
typedef enum { AutoUTC, USNO_UTC = 3, GLONASS_UTC = 6, BEIDOU_UTC } utcType;
typedef struct {
    gnssID id;
    int minChnnls;
    int maxChnnls;
    bool enabled;
    int sigConfig;
} ubxGNSSConfig;

typedef struct {
    bool           antPwr;                  // true if antenna power is enabled
    bool           antShrtDet;              // true if antenna short circuit detection is enabled
    bool           antOpenDet;              // true if antenna open circuit detection is enabled
    bool           antPwrDwnOnShrt;         // true if power down antenna on short detected
    bool           antAutoRec;              // true if automatically recover from antenna short circuit
    int            trkChnnls;               // number of hardware tracking channels
    int            gnssRecs;                // number of GNSS records in gnss
    ubxGNSSConfig* gnss;                    // configurations for each GNSS
    dynModel       model;                   // navigation engine dynamic model
    fixMode        mode;                    // navigation engine fix mode
    double         fixedAltM;               // fixed altitude for 2D fix mode
    double         fixedAltVarM2;           // fixed altitude variance for 2D mode
    int            minElevDeg;              // minimum elevation for a GNSS satellite to be used in a fix
    double         pDoP;                    // position DoP mask
    double         tDoP;                    // time DoP mask
    uint16_t       pAccM;                   // position accuracy mask
    uint16_t       tAccM;                   // time accuracy mask
    uint8_t        staticHoldThreshCmS;     // static hold threshold
    uint8_t        dgnssTimeoutS;           // DGNSS timeout
    uint8_t        cnoThreshNumSVs;         // number of satellites required to have C/NO above cnoThresh for a fix to be attempted
    uint8_t        cnoThreshDbHz;           // C/NO threshold for deciding whether to attempt a fix
    uint16_t       staticHoldMaxDistM;      // static hold distance threshold (before quitting static hold)
    utcType        utcStandard;             // UTC standard used
} ubxConfig;

slReturn ubxGetVersion( int fdPort, int verbosity, ubxVersion* version );
slReturn ubxSaveConfig( int fdPort, int verbosity );
slReturn ubxGetFix( int fdPort, int verbosity, ubxFix* fix );
slReturn ubxGetConfig( int fdPort, int verbosity, ubxConfig* config );
slReturn ubxChangeBaudRate( int fdPort, unsigned int newBaudRate, int verbosity );
slReturn ubxSetNMEAData( int fdPort, int verbosity, bool nmeaOn );
slReturn ubxSynchronize( const int, const int );

#endif //GPSCTL_UBLOX_H
