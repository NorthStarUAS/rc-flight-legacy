#include <pyprops.h>

#include <stdio.h>
#include <stdlib.h>

#include "comms/display.h"

#include "loadavg.h"


static FILE *fload;

static pyPropertyNode system_node;

bool loadavg_init() {
    system_node = pyGetNode("/status", true);
    return true;
}

bool loadavg_update() {
    char buf[5];
    int result = 0;

    if ( (fload = fopen("/proc/loadavg", "r")) != NULL ) {
        result = fread( buf, 4, 1, fload );
        buf[4] = 0;
        if ( result == 1 ) {
            float load = atof(buf);
            system_node.setDouble( "system_load_avg", load );
        } else {
	    printf("fread() failed\n");
            fclose( fload );
            return false;
        }
        fclose( fload );
    } else {
        if ( display_on ) {
            printf("Cannot open /proc/loadavg\n");
        }
        return false;
    }

    return true;
}
