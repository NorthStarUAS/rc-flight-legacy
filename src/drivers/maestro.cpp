#include <fcntl.h>		// open()
#include <termios.h>		// tcgetattr() et. al.

#include "util/strutils.h"
#include "maestro.h"

bool maestro_t::open( const char *device_name ) {
    fd = ::open( device_name, O_RDWR | O_NOCTTY | O_NONBLOCK );
    // fd = ::open( device_name, O_RDWR | O_NOCTTY );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name, strerror(errno) );
	return false;
    }

    return true;
}

void maestro_t::init( pyPropertyNode *config ) {
    act_node = pyGetNode("/actuators", true);
    ap_node = pyGetNode("/autopilot", true);
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    if ( config->hasChild("device") ) {
        string device = config->getString("device");
        if ( open(device.c_str()) ) {
            printf("maestro device opened: %s\n", device.c_str());
        } else {
            printf("unable to open maestro device: %s\n", device.c_str());
        }
    } else {
        printf("no maestro device specified\n");
    }
    if ( config->hasChild("gains") and config->getLen("gains") == 6 ) {
        for ( int i = 0; i < maestro_channels; i++ ) {
            gains[i] = config->getDouble("gains", i);
            printf("maestro: ch[%d] gain = %.2f\n", i, gains[i]);
        }
    }
}

void maestro_t::write_channel(int ch, float norm, bool symmetrical) {
    // honor gain (i.e. for servo reversing)
    norm *= gains[ch];
    
    // target value is 1/4 us, so center (1500) would have a value of
    // 6000 for a symmetrical channel
    int target = 1500.0 * 4;
    if ( symmetrical ) {
        // rudder, etc.
        target = (1500 + 500 * norm) * 4;
    } else {
        // throttle
        target = (1000 + 1000 * norm) * 4;
    }
    uint8_t command[] = {0x84, ch, target & 0x7F, target >> 7 & 0x7F};
    if ( ::write(fd, command, sizeof(command)) == -1) {
        perror("maestro error writing");
    }
}

void maestro_t::write() {
    float throttle = 0.0;
    if ( pilot_node.getDouble("throttle_safety") < -0.3 and !pilot_node.getBool("fail_safe") ) {
        if ( ap_node.getBool("master_switch") ) {
	    throttle = act_node.getDouble("throttle");
	} else {
            throttle = pilot_node.getDouble("throttle");
	}
    }
    // hardcoded hack: tie throttle gain to channel[4] switch position
    gains[0] = pilot_node.getDouble("channel", 4);
    write_channel(0, throttle, true);
    write_channel(1, act_node.getDouble("aileron"), true);
    write_channel(2, act_node.getDouble("elevator"), true);
    write_channel(3, act_node.getDouble("rudder"), true);
    write_channel(4, act_node.getDouble("flaps"), true);
    write_channel(5, act_node.getDouble("gear"), true);
}

void maestro_t::close() {
    ::close(fd);
}
