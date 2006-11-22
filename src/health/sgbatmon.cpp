#include <stdio.h>
#include <stdlib.h>

#include "comms/logging.h"
#include "util/timing.h"

#include "sgbatmon.h"


struct sgbatmon batmonpacket;


static FILE *fbat;
static float start_volts = 0.0;
static float volt_cutoff = 3.30;
static double start_time = 0.0;


bool sgbatmon_init() {
    return true;
}


bool sgbatmon_update() {
    char buf[5];
    int result = 0;

    if ( (fbat = fopen("/dev/platx/batmon", "r")) != NULL ) {
        result = fread( buf, 4, 1, fbat );
        buf[4] = 0;
        if ( result == 1 ) {
            float v = atof(buf);
            if ( v < 0 ) {
                // nonsense value keep previous reading.
                v = batmonpacket.volts;
            }
            if ( start_volts < 0.1 ) {
                batmonpacket.volts = v;
                start_volts = v;
                    start_time = get_Time();
            }
            printf("buf = %s, raw volts = %.2f\n", buf, v);
            batmonpacket.volts_raw = v;
            batmonpacket.volts = (batmonpacket.volts*99.0 + v) / 100.0;
            batmonpacket.time = get_Time();
                
            float v_used = start_volts - batmonpacket.volts;
            float t_elapsed = batmonpacket.time - start_time;
            if ( t_elapsed > 0 ) {
                double rate = v_used / t_elapsed;
                printf("volts per sec = %.10f\n", rate);
                if ( rate > 0.0000001 ) {
                    float est = (batmonpacket.volts - volt_cutoff) / rate;
                    if ( est < 0.0 ) est = 0.0;
                    batmonpacket.est_seconds = (uint16_t)est;
                }
            }
        } else {
            printf("fread() failed\n");
            fclose( fbat );
            return false;
        }
        fclose( fbat );
    } else {
        if ( display_on ) {
            printf("Cannot open battery monitor device\n");
        }
        return false;
    }

    return true;
}
