#include <stdio.h>

#include "logging.h"


FILE *fimu = NULL;
FILE *fgps = NULL;
FILE *fnav = NULL;


bool logging_init() {
    // open files

    if ( (fimu = fopen("/mnt/cf1/imu.dat","w+b")) == NULL ) {
        printf("imu.dat cannot be created in /mnt/cf1 directory...error!\n");
        return false;
    }

    if ( (fgps = fopen("/mnt/cf1/gps.dat","w+b")) == NULL ) {
        printf("gps.dat cannot be created in /mnt/cf1 directory...error!\n");
        return false;
    }

    if ( (fnav = fopen("/mnt/cf1/nav.dat","w+b")) == NULL ) {
        printf("nav.dat cannot be created in /mnt/cf1 directory...error!\n");
        return false;
    }

    return true;
}


bool logging_close() {
    // close files

    fclose(fimu);
    fclose(fgps);
    fclose(fnav);

    return true;
}


void log_gps( struct gps *gpspacket ) {
    fwrite( gpspacket, sizeof(struct gps), 1, fgps );
}


void log_imu( struct imu *imupacket ) {
    fwrite( imupacket, sizeof(struct imu), 1, fimu );
}


void log_nav( struct nav *navpacket ) {
    fwrite( navpacket, sizeof(struct nav), 1, fnav );
}

