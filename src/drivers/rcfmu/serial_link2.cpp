#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <stdlib.h>             // realloc()
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()
#include <sys/ioctl.h>          // FIONREAD

#include "serial_link2.h"

SerialLink2::SerialLink2() {
}

SerialLink2::~SerialLink2() {
}

int SerialLink2::encode_baud( int baud ) {
    if ( baud == 115200 ) {
	return B115200;
    } else if ( baud == 500000 ) {
	return B500000;
    } else {
	printf("unsupported baud rate = %d\n", baud);
        return B115200;
    }
}

void SerialLink2::checksum( uint8_t id, uint8_t len_lo, uint8_t len_hi,
                            uint8_t *buf, uint16_t buf_size,
                            uint8_t *cksum0, uint8_t *cksum1 )
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += id;
    c1 += c0;

    c0 += len_lo;
    c1 += c0;

    c0 += len_hi;
    c1 += c0;

    for ( uint16_t i = 0; i < buf_size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}

bool SerialLink2::open( int baud, const char *device_name ) {
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

bool SerialLink2::update() {
    // 0 = looking for SOM0
    // 1 = looking for SOM1
    // 2 = looking for packet id
    // 3 = looking for packet len (note 2 bytes)
    // 4 = looking for packet data
    // 5 = looking for checksum_lo
    // 6 = looking for checksum_l=hi
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
                // fprintf( stderr, "read START_OF_MSG1\n");
                state++;
            } else if ( input[0] == START_OF_MSG0 ) {
                // fprintf( stderr, "read START_OF_MSG0\n");
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
            // fprintf( stderr, "pkt_id = %d\n", pkt_id );
            state++;
        }
    }
    if ( state == 3 ) {
        if ( bytes_available() >= 2 ) {
            len = read( fd, input, 2 );
            if ( len == 2 ) {
                pkt_len_lo = input[0];
                pkt_len_hi = input[1];
                // fprintf( stderr, "packet lo: %d hi: %d\n", pkt_len_lo, pkt_len_hi);
                pkt_len = pkt_len_hi << 8 | pkt_len_lo;
                if ( pkt_len > 4096 ) {
                    fprintf( stderr, "nonsense packet size (%d) for id: %d, skipping.\n", pkt_len, pkt_id);
                    parse_errors++;
                    state = 0;
                } else {
                    // fprintf( stderr, "packet len: %d\n", pkt_len);
                    state++;
                }
            } else {
                parse_errors++;
                state = 0;
            }
        }
    }
    if ( state == 4 ) {
        if ( pkt_len > payload_len ) {
            // reallocate buffer if new payload is larger than
            // anything we've read so far.
            void *newbuf = realloc(payload, pkt_len);
            if ( newbuf == nullptr ) {
                printf("payload heap allocation failed.\n");
                state = 0;
                return false;
            } else {
                payload = (uint8_t *)newbuf;
                payload_len = pkt_len;
            }
        }
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
            checksum( pkt_id, pkt_len_lo, pkt_len_hi, payload, pkt_len,
                      &cksum0, &cksum1 );
            if ( cksum0 == cksum_lo && cksum1 == cksum_hi ) {
                // printf( "checksum passes (%d)\n", pkt_id );
                new_data = true;
            } else {
                parse_errors++;
                // printf( "checksum failed\n");
                // if ( display_on ) {
                //     printf("checksum failed %d %d (computed) != %d %d (message)\n",
                //            cksum_A, cksum_B, cksum_lo, cksum_hi );
                // }
            }
            // This Is the end of a record, reset state to 0 to start
            // looking for next record
            state = 0;
        }
    }

    return new_data;
}

int SerialLink2::bytes_available() {
    int avail = 0;
    ioctl(fd, FIONREAD, &avail);
    return avail;
}

bool SerialLink2::write_packet(uint8_t packet_id, uint8_t *payload, uint16_t payload_size) {
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
    
    // packet length (2 bytes)
    uint8_t len_lo = payload_size & 0xFF;
    uint8_t len_hi = payload_size >> 8;
    buf[0] = len_lo;
    buf[1] = len_hi;
    write( fd, buf, 2 );

    // write payload
    if ( payload_size > 0 ) {
        write( fd, payload, payload_size );
    }
    
    // check sum (2 bytes)
    checksum( packet_id, len_lo, len_hi, payload, payload_size, &cksum0, &cksum1 );
    buf[0] = cksum0;
    buf[1] = cksum1;
    write( fd, buf, 2 );

    return true;
}

bool SerialLink2::close() {
    int result = ::close(fd);
    if ( result < 0 ) {
        fprintf( stderr, "unable to close serial: %s\n", strerror(errno) );
	return false;
    }
    fd = -1;
    return true;
}

