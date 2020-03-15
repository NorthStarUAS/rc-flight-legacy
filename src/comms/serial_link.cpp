#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()
#include <sys/ioctl.h>          // FIONREAD

#include "serial_link.h"

SerialLink::SerialLink() {
}

SerialLink::~SerialLink() {
}

int SerialLink::encode_baud( int baud ) {
    if ( baud == 115200 ) {
	return B115200;
    } else if ( baud == 500000 ) {
	return B500000;
    } else {
	printf("unsupported baud rate = %d\n", baud);
        return B115200;
    }
}

void SerialLink::checksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 )
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += hdr1;
    c1 += c0;

    c0 += hdr2;
    c1 += c0;

    for ( uint8_t i = 0; i < size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}

bool SerialLink::open( int baud, const char *device_name ) {
    // fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    fd = ::open( device_name, O_RDWR | O_NOCTTY );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name, strerror(errno) );
	return false;
    }

    struct termios config;	// Old Serial Port Settings

    memset(&config, 0, sizeof(config));

    int baud_bits = encode_baud( baud );

    // Configure New Serial Port Settings
    config.c_cflag     = baud_bits | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 1;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name, strerror(errno) );
	return false;
    }

    return true;
}

bool SerialLink::update() {
    int len;
    uint8_t input[2];
    int giveup_counter = 0;

    // printf("update() entry state = %d\n", state);

    bool new_data = false;

    if ( state == 0 ) {
        counter = 0;
        len = read( fd, input, 1 );
        giveup_counter = 0;
        while ( len > 0 && input[0] != START_OF_MSG0 && giveup_counter < 100 ) {
            // printf("state0: len = %d val = %2X (%c)\n", len, input[0] , input[0]);
            len = read( fd, input, 1 );
            giveup_counter++;
            // fprintf( stderr, "giveup_counter = %d\n", giveup_counter);
        }
        if ( len > 0 && input[0] == START_OF_MSG0 ) {
            // fprintf( stderr, "read START_OF_MSG0\n");
            state++;
        }
    }
    if ( state == 1 ) {
        len = read( fd, input, 1 );
        if ( len > 0 ) {
            if ( input[0] == START_OF_MSG1 ) {
                //fprintf( stderr, "read START_OF_MSG1\n");
                state++;
            } else if ( input[0] == START_OF_MSG0 ) {
                //fprintf( stderr, "read START_OF_MSG0\n");
            } else {
                parse_errors++;
                state = 0;
            }
        }
    }
    if ( state == 2 ) {
        len = read( fd, input, 1 );
        if ( len > 0 ) {
            pkt_id = input[0];
            //fprintf( stderr, "pkt_id = %d\n", pkt_id );
            state++;
        }
    }
    if ( state == 3 ) {
        len = read( fd, input, 1 );
        if ( len > 0 ) {
            pkt_len = input[0];
            if ( pkt_len < 256 ) {
                //fprintf( stderr, "pkt_len = %d\n", pkt_len );
                state++;
            } else {
                parse_errors++;
                state = 0;
            }
        }
    }
    if ( state == 4 ) {
        len = read( fd, input, 1 );
        while ( len > 0 ) {
            payload[counter++] = input[0];
            // fprintf( stderr, "%02X ", input[0] );
            if ( counter >= pkt_len ) {
                break;
            }
            len = read( fd, input, 1 );
        }

        if ( counter >= pkt_len ) {
            state++;
            // fprintf( stderr, "\n" );
        }
    }
    if ( state == 5 ) {
        len = read( fd, input, 1 );
        if ( len > 0 ) {
            cksum_lo = input[0];
            state++;
        }
    }
    if ( state == 6 ) {
        len = read( fd, input, 1 );
        if ( len > 0 ) {
            cksum_hi = input[0];
            uint8_t cksum0, cksum1;
            checksum( pkt_id, pkt_len, payload, pkt_len, &cksum0, &cksum1 );
            if ( cksum0 == cksum_lo && cksum1 == cksum_hi ) {
                // printf( "checksum passes (%d)\n", pkt_id );
                new_data = true;
            } else {
                parse_errors++;
                //if ( display_on ) {
                    // printf("checksum failed %d %d (computed) != %d %d (message)\n",
                    //        cksum_A, cksum_B, cksum_lo, cksum_hi );
                //}
            }
            // This Is the end of a record, reset state to 0 to start
            // looking for next record
            state = 0;
        }
    }

    return new_data;
}

int SerialLink::bytes_available() {
    int avail = 0;
    ioctl(fd, FIONREAD, &avail);
    return avail;
}

bool SerialLink::write_packet(uint8_t packet_id, uint8_t *payload, uint8_t len) {
    uint8_t buf[2];
    uint8_t cksum0, cksum1;
    
    // start of message sync (2) bytes
    buf[0] = START_OF_MSG0;
    write( fd, buf, 1 );
    buf[0] = START_OF_MSG1;
    write( fd, buf, 1 );

    // packet id (1 byte)
    buf[0] = packet_id;
    write( fd, buf, 1 );
    
    // packet length (1 byte)
    buf[0] = len;
    write( fd, buf, 1 );

    // write payload
    if ( len > 0 ) {
        write( fd, payload, len );
    }
    
    // check sum (2 bytes)
    checksum( packet_id, len, payload, len, &cksum0, &cksum1 );
    buf[0] = cksum0;
    write( fd, buf, 1 );
    buf[0] = cksum1;
    write( fd, buf, 1 );

    return true;
}

bool SerialLink::close() {
    int result = ::close(fd);
    if ( result < 0 ) {
        fprintf( stderr, "unable to close serial: %s\n", strerror(errno) );
	return false;
    }
    fd = -1;
    return true;
}
