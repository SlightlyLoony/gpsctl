//
// Created by Tom Dilatush on 10/20/17.
//

#ifndef GPSCTL_UBLOX_H
#define GPSCTL_UBLOX_H

typedef enum ubxState    { Valid, NotValid, NotPresent } ubxState;
typedef enum ubxResponse { Ok, TxFail, InvalidPollResponse, NoPollResponse, UnexpectedPollResponse,
    NAK, UnexpectedAck, InvalidAck, NoAck, None } ubxResponse;
typedef enum changeBaudResponse { ChangeBaudOk, ChangeBaudFailure } changeBaudResponse;

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
void test( int fdPort );

changeBaudResponse changeBaudRate( int fdPort, unsigned int newBaudRate, bool verbose );

typedef struct pvt_fix {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    double second;
    int nanoseconds_correction;
    int time_accuracy_ns;
    bool time_resolved;
    bool date_valid;
    bool time_valid;
    int magnetic_declination_deg;
    int magnetic_declination_accuracy_deg;
    bool magnetic_declination_valid;
    bool fix_is_3d;
    bool fix_valid;
    int number_of_satellites_used;
    double longitude_deg;
    double latitude_deg;
    int height_above_ellipsoid_mm;
    int height_above_sea_level_mm;
    int height_accuracy_mm;
    int ground_speed_mm_s;
    int ground_speed_accuracy_mm_s;
    double heading_deg;
    double heading_accuracy_deg;

} pvt_fix;

int getFix( int fdPort, bool verbose, pvt_fix *fix );
#endif //GPSCTL_UBLOX_H
