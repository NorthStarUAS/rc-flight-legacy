// read the specified serial port and blindly log the data to a file

#include <stdio.h>
#include <stdlib.h>             // exit()
#include <string.h>
#include <string>

#include <sys/types.h>          // open()
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>             // read()


using std::string;

void usage() {
    printf("\nUsage: uartlogger --option1 arg1 --option2 arg2 ...\n");
    printf("--device dev_path (uart device)\n");
    printf("--baud n (uart baud)\n");
    printf("--file file.bin (file name for logging uart traffic)\n");
    exit(0);
}

int main( int argc, char **argv) {
    printf("simple uart logger\n");

    string device = "/dev/ttyS0";
    int baud = 115200;
    string file = "uart-log.bin";

    // Parse the command line
    for ( int iarg = 1; iarg < argc; iarg++ ) {
        if ( !strcmp(argv[iarg], "--device" )  ) {
            ++iarg;
            if ( iarg < argc ) {
                device = argv[iarg];
            } else {
                usage();
            }
        } else if ( !strcmp(argv[iarg], "--baud" )  ) {
            ++iarg;
            if ( iarg < argc ) {
                baud = atoi( argv[iarg] );
                if ( baud < 300 || baud > 500000 ) {
                    printf("Baud must be >= 300 and <= 500000\n");
                    usage();
                }
            } else {
                usage();
            }
        } else if ( !strcmp(argv[iarg],"--file") ) {
            ++iarg;
            if ( iarg < argc ) {
                file = argv[iarg];
            } else {
                usage();
            }
	} else {
	    usage();
	}
    }

    printf("before opening %s\n", device.c_str() );
    printf("baud = %d\n", baud);
    printf("log file = %s\n", file.c_str());

    int baud_bits = B500000;
    if ( baud == 9600 ) {
	baud_bits = B9600;
    } else if ( baud == 115200 ) {
	baud_bits = B115200;
    } else if ( baud == 500000 ) {
	baud_bits = B500000;
     } else {
	printf("unsupported baud rate = %d\n", baud);
    }
    

    // Open and configure the serial port
    
    int serial_fd = -1;
    serial_fd = open( device.c_str(), O_RDWR | O_NOCTTY );
    if ( serial_fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device.c_str(), strerror(errno) );
	return false;
    }

    struct termios config;	// Old Serial Port Settings

    memset(&config, 0, sizeof(config));

    // Save Current Serial Port Settings
    // tcgetattr(serial_fd,&oldTio); 

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
    tcflush(serial_fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( serial_fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device.c_str(), strerror(errno) );
	return false;
    }

    // Open the log file
    FILE *fp = fopen( file.c_str(), "w");
    if ( fp == NULL ) {
        fprintf( stderr, "error opening file for writing: %s", file.c_str() );
    }

    const int MAX_BUF = 1024;
    char serial_buf[MAX_BUF];

    int byte_counter = 0;
    int increment = 4096;
    int message_threshold = 0;
    
    while ( true ) {
        // Read available bytes from the serial port
        int rlen = read( serial_fd, serial_buf, MAX_BUF );
        if ( rlen > 0 ) {
            byte_counter += rlen;
            if ( byte_counter > message_threshold ) {
                printf("total bytes read: %d\n", byte_counter);
                message_threshold += increment;
            }
                
            int wlen = fwrite( serial_buf, 1, rlen, fp );
            if ( rlen != wlen ) {
                fprintf( stderr, "bytes in (%d) != bytes written (%d)\n",
                         rlen, wlen);
            }
        }
    }


    // we'll never get here, but just to pretend we are being tidy ...
    fclose(fp);
    close(serial_fd);

    return 0;
}
