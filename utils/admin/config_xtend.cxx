#include <errno.h>              // errno
#include <sys/types.h>          // open()
#include <sys/stat.h>           // open()
#include <fcntl.h>              // open()
#include <stdio.h>              // printf() et. al.
#include <termios.h>            // tcgetattr() et. al.
#include <unistd.h>             // tcgetattr() et. al.
#include <string.h>             // memset()
#include <string>

using std::string;


static int fd = -1;
static string device_name = "/dev/ttyUSB1";


// send our configured init strings to configure gpsd the way we prefer
static bool xtend_open(tcflag_t baud) {
    fd = open( device_name.c_str(), O_RDWR /* | O_NOCTTY */ );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config; 	// Serial port settings
    // memset(&config, 0, sizeof(config));

    // Fetch current serial port settings
    tcgetattr(fd, &config); 
    cfsetspeed( &config, baud );
    cfmakeraw( &config );

#if 0
    // Configure New Serial Port Settings
    config.c_cflag     = baud | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 0;	   // block 'read' from returning until at
                                   // least 1 character is received

#endif

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    return true;
}


bool xtend_send( const char *payload ) {
    unsigned int size = strlen( payload );
    unsigned int len = write( fd, payload, size );
    if ( len != size ) {
	printf("wrote %d of %d bytes -- try again\n", len, size);
	return false;
    } else {
	printf("wrote %d bytes: %s\n", len, payload);
	fsync(fd);
	return true;
    }

}


static bool xtend_read_line( char *payload ) {
    static int counter = 0;
    int len;
    char input[8];

    bool new_message = false;

    counter = 0;
    len = read( fd, input, 1 );
    while ( true ) {
	// printf("loopingread\n");
	if ( input[0] == '\r' ) {
	    payload[counter] = '\0';
	    break;
	}
	if ( len > 0 ) {
	    payload[counter++] = input[0];
	    // printf("xtend read: %c\n", input[0]);
	}

	len = read( fd, input, 1 );
    }

    //  printf("message = %s\n", payload);

    return new_message;
}


int main(int argc, char **argv ) {
    bool query_only = false;
    string power_level = "2";
    string stream_limit = "400";

    if ( argc == 1 ) {
	query_only = true;
    } else if ( argc == 3 ) {
	power_level = argv[1];
	stream_limit = argv[2];
    } else {
	printf("usage: %s <power_code> <stream_limit_bytes_hex>\n", argv[0]);
	printf("  power code: 0=1mw 1=10mw 2=100mw 3=500mw 4=1000mw\n");
	printf("  stream_limit_bytes: sender inserts a pause after this many bytes to\n");
	printf("      emulate full duplex operations.\n");
	printf("\n");
	printf("Example: %s 2 400\n", argv[0]);
	return(-1);
    }
    

    char reply[256];

    xtend_open( B115200 );
    sleep(1);
    xtend_send( "ATCN\r" );
    sleep(2);

    xtend_send( "+++" );
    xtend_read_line(reply);
    printf("reply = %s\n", reply);

    printf("\n");
    printf("Querying current settings...\n");

    xtend_send( "ATBD\r" );
    xtend_read_line(reply);
    printf("reply = %s: ", reply);
    if ( reply[0] == '3' ) {
	printf("baud = 9600\n");
    } else if ( reply[0] == '7' ) {
	printf("baud = 115,200\n");
    }

    xtend_send( "ATBR\r" );
    xtend_read_line(reply);
    printf("reply = %s: ", reply);
    if ( reply[0] == '0' ) {
	printf("RF baud = 9600\n");
    } else if ( reply[0] == '1' ) {
	printf("RF baud = 115,200\n");
    }

    xtend_send( "ATID\r" );
    xtend_read_line(reply);
    printf("reply = %s\n", reply);

    xtend_send( "ATPL\r" );
    xtend_read_line(reply);
    printf("reply = %s: ", reply);
    if ( reply[0] == '0' ) {
	printf("1 mW\n");
    } else if ( reply[0] == '1' ) {
	printf("10 mW\n");
    } else if ( reply[0] == '2' ) {
	printf("100 mW\n");
    } else if ( reply[0] == '3' ) {
	printf("500 mW\n");
    } else if ( reply[0] == '4' ) {
	printf("1000 mW\n");
    }

    xtend_send( "ATSH\r" );
    xtend_read_line(reply);
    printf("reply = %s\n", reply);

    xtend_send( "ATSL\r" );
    xtend_read_line(reply);
    printf("reply = %s\n", reply);

    xtend_send( "ATTT\r" );
    xtend_read_line(reply);
    printf("reply = 0x%s\n", reply);

    if ( !query_only ) {
	printf("\n");
	printf("Setting parameters...\n");

	xtend_send( "ATBD7\r" );
	xtend_read_line(reply);
	printf("reply = %s\n", reply);

	xtend_send( "ATBR1\r" );
	xtend_read_line(reply);
	printf("reply = %s\n", reply);

	xtend_send( "ATPL" );
	xtend_send( power_level.c_str() );
	xtend_send( "\r" );
	xtend_read_line(reply);
	printf("reply = %s\n", reply);

	xtend_send( "ATTT400\r" );
	xtend_read_line(reply);
	printf("reply = %s\n", reply);

	printf("\n");
	printf("Writing settings to non-volatile memory\n");

	xtend_send( "ATWR\r" );
	xtend_read_line(reply);
	printf("reply = %s\n", reply);

	xtend_send( "ATCN\r" );
	xtend_read_line(reply);
	printf("reply = %s\n", reply);
    }

    close(fd);

    return 0;
}
