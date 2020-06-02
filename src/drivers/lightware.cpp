#include <fcntl.h>		// open()
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
