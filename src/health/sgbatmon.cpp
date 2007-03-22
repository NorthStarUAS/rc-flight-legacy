#include <stdio.h>
#include <stdlib.h>

#include "comms/logging.h"
#include "util/timing.h"

#include "sgbatmon.h"


static FILE *fbat;
static float start_volts = 0.0;
static float volt_cutoff = 3.40;
static double start_time = 0.0;
static int batmon_failed;


bool sgbatmon_init() {
    batmon_failed = 0;
    return true;
}


bool sgbatmon_update() {
    char buf[5];
    int result = 0;

    if ( batmon_failed > 5 ) {
        return false;
    }

    // NOTE: on the stargate, the actual battery monitor device name
    // is /dev/platx/batmon, but it is brutally slow to open and read.
    // So I created a "niced" background script that copies the value
    // of /dev/platx/batmon to /tmp/batmon every 10 seconds.  Thus the
    // real time app can simply open a file and read the value quickly
    // from there.

    if ( (fbat = fopen("/tmp/batmon", "r")) != NULL ) {
        result = fread( buf, 4, 1, fbat );
        buf[4] = 0;
        if ( result == 1 ) {
            float v = atof(buf);
            if ( v <= 0 ) {
                // nonsense value keep previous reading.
                v = healthpacket.volts;
            }
            if ( start_volts < 0.1 ) {
                healthpacket.volts = v;
                start_volts = v;
		start_time = get_Time();
            }
            // printf("buf = %s, raw volts = %.2f\n", buf, v);
            healthpacket.volts_raw = v;
            healthpacket.volts = healthpacket.volts*0.99 + v*0.01;
                
            float v_used = start_volts - healthpacket.volts;
            float t_elapsed = healthpacket.time - start_time;
            if ( t_elapsed > 0 ) {
                double rate = v_used / t_elapsed;
                // printf("volts per sec = %.10f\n", rate);
                if ( rate > 0.0000001 ) {
                    float est = (healthpacket.volts - volt_cutoff) / rate;
                    if ( est < 0.0 ) est = 0.0;
                    healthpacket.est_seconds = (uint16_t)est;
                }
            }

	    batmon_failed = 0;
        } else {
	    printf("fread() failed\n");
            fclose( fbat );
            batmon_failed++;
            return false;
        }
        fclose( fbat );
    } else {
        batmon_failed++;
        if ( display_on ) {
            printf("Cannot open battery monitor device\n");
        }
        return false;
    }

    return true;
}
