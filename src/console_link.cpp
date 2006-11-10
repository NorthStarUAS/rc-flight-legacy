#include <stdint.h>
#include <unistd.h>

#include "console_link.h"
#include "globaldefs.h"
#include "serial.h"
#include "util.h"

// global variables

bool console_link_on = false;    // link to ground station via console port
static int sPort0;


// open up the console port
void console_link_init() {
    sPort0 = open_serial( SERIAL_PORT0, BAUDRATE_57600 );
}


static short console_write( const void *buf, short size ) {
    return write(sPort0, buf, size);
}


void console_link_gps( struct gps *gpspacket ) {
    uint8_t buf[3];
    uint8_t size;
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1; buf[2] = 0;
    console_write( buf, 2 );

    // packet id (1 byte)
    buf[0] = GPS_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct gps);
    buf[0] = size + 2; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( gpspacket, size );

    // check sum (2 bytes)
    ugear_cksum( (uint8_t *)gpspacket, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_imu( struct imu *imupacket ) {
    uint8_t buf[3];
    uint8_t size;
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1; buf[2] = 0;
    console_write( buf, 2 );

    // packet id (1 byte)
    buf[0] = IMU_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct imu);
    buf[0] = size + 2; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( imupacket, size );

    // check sum (2 bytes)
    ugear_cksum( (uint8_t *)imupacket, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_nav( struct nav *navpacket ) {
    uint8_t buf[3];
    uint8_t size;
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1; buf[2] = 0;
    console_write( buf, 2 );

    // packet id (1 byte)
    buf[0] = NAV_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct nav);
    buf[0] = size + 2; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( navpacket, size );

    // check sum (2 bytes)
    ugear_cksum( (uint8_t *)navpacket, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_servo( struct servo *servopacket ) {
    uint8_t buf[3];
    uint8_t size;
    uint8_t cksum0, cksum1;

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1; buf[2] = 0;
    console_write( buf, 2 );

    // packet id (1 byte)
    buf[0] = SERVO_PACKET; buf[1] = 0;
    console_write( buf, 1 );

    // packet size (1 byte)
    size = sizeof(struct servo);
    buf[0] = size + 2; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( servopacket, size );

    // check sum (2 bytes)
    ugear_cksum( (uint8_t *)servopacket, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}
