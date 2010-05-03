#include <stdio.h>
#include <stdlib.h>

#include "comms/logging.h"
#include "props/props.hxx"

#include "loadavg.h"


static FILE *fload;

static SGPropertyNode *system_load_avg = NULL;


bool loadavg_init() {
    system_load_avg = fgGetNode("/status/system-load-avg", true);

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
            system_load_avg->setFloatValue(load);
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
