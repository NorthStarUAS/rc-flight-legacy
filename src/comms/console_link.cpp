#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "checksum.h"
#include "globaldefs.h"
#include "serial.h"

#include "console_link.h"

// global variables

bool console_link_on = false;    // link to ground station via console port
static int sPort0;


// open up the console port
void console_link_init() {
    sPort0 = open_serial( SERIAL_PORT0, BAUDRATE_115200, true );
}


static short console_write( uint8_t *buf, short size ) {
  for ( int i = 0; i < size; ++i ) {
    // printf("%d ", (uint8_t)buf[i]);
    write( sPort0, buf+i, 1 );
  }
  // printf("\n");
  return size;
}


void console_link_gps( struct gps *gpspacket ) {
    static const uint8_t skip_count = 2;
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

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
    buf[0] = size; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( (uint8_t *)gpspacket, size );

    // check sum (2 bytes)
    ugear_cksum( GPS_PACKET, size, (uint8_t *)gpspacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_imu( struct imu *imupacket ) {
    static const uint8_t skip_count = 25;
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

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
    buf[0] = size; buf[1] = 0;
    // printf("imu size = %d\n", size);
    console_write( buf, 1 );

    // packet data
    uint8_t bytes = console_write( (uint8_t *)imupacket, size );
    if ( bytes != size ) {
      printf("Only wrote %d imu bytes out of %d\n", bytes, size);
    }

    // check sum (2 bytes)
    ugear_cksum( IMU_PACKET, size, (uint8_t *)imupacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_nav( struct nav *navpacket ) {
    static const uint8_t skip_count = 25;
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

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
    buf[0] = size; buf[1] = 0;
    console_write( buf, 1 );

    // packet data
    console_write( (uint8_t *)navpacket, size );

    // check sum (2 bytes)
    ugear_cksum( NAV_PACKET, size, (uint8_t *)navpacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}


void console_link_servo( struct servo *servopacket ) {
    static const uint8_t skip_count = 25;
    static uint8_t skip = skip_count;

    if ( skip > 0 ) {
        --skip;
        return;
    } else {
        skip = skip_count;
    }

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
    buf[0] = size; buf[1] = 0;
    // printf("servo size = %d\n", size);
    console_write( buf, 1 );

    // packet data
    uint8_t bytes = console_write( (uint8_t *)servopacket, size );
    // uint8_t *tmp = (uint8_t *)servopacket;
    // printf("%d %d %d %d\n", tmp[0], tmp[1], tmp[2], tmp[3] );

    if ( bytes != size ) {
      printf("Only wrote %d servo bytes out of %d\n", bytes, size);
    }

    // check sum (2 bytes)
    ugear_cksum( SERVO_PACKET, size, (uint8_t *)servopacket, size,
		 &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    console_write( buf, 2 );
}
