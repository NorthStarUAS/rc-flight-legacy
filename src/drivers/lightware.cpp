#include <fcntl.h>		// open()
//#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.

#include "util/strutils.h"
#include "lightware.h"

bool lightware_t::open( const char *device_name ) {
    fd = ::open( device_name, O_RDWR | O_NOCTTY | O_NONBLOCK );
    // fd = ::open( device_name, O_RDWR | O_NOCTTY );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name, strerror(errno) );
	return false;
    }

#if 0
    struct termios config;	// Old Serial Port Settings

    memset(&config, 0, sizeof(config));

    // Configure New Serial Port Settings
    config.c_cflag     = B115200 | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 0;	   // block 'read' from returning until at
                                   // least 0 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name, strerror(errno) );
	return false;
    }

    // Enable non-blocking IO (one more time for good measure)
    fcntl(fd, F_SETFL, O_NONBLOCK);
#endif
    
    return true;
}

void lightware_t::init( pyPropertyNode *config ) {
    pos_node = pyGetNode("/position", true);
    if ( config->hasChild("device") ) {
        string device = config->getString("device");
        if ( open(device.c_str()) ) {
            printf("lightware device opened: %s\n", device.c_str());
        } else {
            printf("unable to open lightware device: %s\n", device.c_str());
        }
    } else {
        printf("no lightware device specified\n");
    }
}

float lightware_t::read() {
    char buf[128];
    int len;
    //buf[0] = 27;
    //::write(fd, buf, 1);
    while ( (len = ::read(fd, buf, sizeof(buf)-1)) > 0 ) {
        vector<string> tokens = split(buf);
        if ( tokens.size() == 5 ) {
            // printf("lightware: %d %s\n", tokens.size(), tokens[0].c_str());
            pos_node.setDouble("lightware_agl_m", atof(tokens[0].c_str()));
        }
    }
    return 0.0;
}

void lightware_t::close() {
    ::close(fd);
}
