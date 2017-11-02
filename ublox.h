//
// Created by Tom Dilatush on 10/20/17.
//

#ifndef GPSCTL_UBLOX_H
#define GPSCTL_UBLOX_H

typedef enum ubxState    { Valid, NotValid, NotPresent } ubxState;

typedef enum ubxResponse { Ok, TxFail, InvalidPollResponse, NoPollResponse, UnexpectedPollResponse,
    NAK, UnexpectedAck, InvalidAck, NoAck, None } ubxResponse;

typedef enum changeBaudResponse { ChangeBaudOk, ChangeBaudFailure } changeBaudResponse;

typedef enum reportResponse { reportOK, reportError } reportResponse;

typedef enum configResponse { configOK, configError } configResponse;

typedef struct ubxChecksum {
    byte ck_a;
    byte ck_b;
} ubxChecksum;

typedef struct ubxType {
    byte class;
    byte id;
} ubxType;
#define UBX_ACK 0x05
#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00
#define UBX_CFG 0x06
#define UBX_CFG_PRT 0x00
#define UBX_MON 0x0A
#define UBX_MON_VER 0x04
#define UBX_NAV 0x01
#define UBX_NAV_PVT 0x07
#define UBX_CFG_PRT_UART_ID = 0x01

typedef struct pvt_fix {
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

} pvt_fix;

typedef struct ubxVersion {
    char *software;
    char *hardware;
    char **extensions;  // terminated by NULL entry...
    int number_of_extensions;
} ubxVersion;

reportResponse getVersion(int fdPort, int verbosity, ubxVersion *version );

changeBaudResponse changeBaudRate( int fdPort, unsigned int newBaudRate, bool verbose );

reportResponse getFix( int fdPort, int verbosity, pvt_fix *fix );

configResponse setNMEAData(int fdPort, int verbosity, bool nmeaOn);

bool ubxSynchronize( const int, const int );

#endif //GPSCTL_UBLOX_H
