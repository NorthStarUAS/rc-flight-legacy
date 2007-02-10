#include <stdio.h>
#include <stdlib.h>

#include "comms/logging.h"
#include "util/timing.h"

#include "sgbatmon.h"


static FILE *fbat;
static float start_volts = 0.0;
static float volt_cutoff = 3.40;
static double start_time = 0.0;
static bool batmon_exists;


bool sgbatmon_init() {
    batmon_exists = true;
    return true;
}


bool sgbatmon_update() {
    char buf[5];
    int result = 0;

    if ( !batmon_exists ) {
        return false;
    }

    if ( (fbat = fopen("/dev/platx/batmon", "r")) != NULL ) {
        result = fread( buf, 4, 1, fbat );
        buf[4] = 0;
        if ( result == 1 ) {
            float v = atof(buf);
            if ( v < 0 ) {
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
            healthpacket.time = get_Time();
                
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
        } else {
	    printf("fread() failed\n");
            fclose( fbat );
            batmon_exists = false;
            return false;
        }
        fclose( fbat );
    } else {
        batmon_exists = false;
        if ( display_on ) {
            printf("Cannot open battery monitor device\n");
        }
        return false;
    }

    return true;
}
