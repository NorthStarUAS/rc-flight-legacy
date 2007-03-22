#include <stdio.h>

#include "logging.h"
#include "navigation/ahrs.h"


// global variables for data file logging

static FILE *fimu = NULL;
static FILE *fgps = NULL;
static FILE *fnav = NULL;
static FILE *fservo = NULL;
static FILE *fhealth = NULL;

bool log_to_file = false;       // log to file is enabled/disabled
bool display_on = false;        // dump summary to display periodically


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

    if ( (fservo = fopen("/mnt/cf1/servo.dat","w+b")) == NULL ) {
        printf("servo.dat cannot be created in /mnt/cf1 directory...error!\n");
        return false;
    }

    if ( (fhealth = fopen("/mnt/cf1/health.dat","w+b")) == NULL ) {
        printf("health.dat cannot be created in /mnt/cf1 directory...error!\n");
        return false;
    }

    return true;
}


bool logging_close() {
    // close files

    fclose(fimu);
    fclose(fgps);
    fclose(fnav);
    fclose(fservo);
    fclose(fhealth);

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


void log_servo( struct servo *servopacket ) {
    fwrite( servopacket, sizeof(struct servo), 1, fservo );
}


void log_health( struct health *healthpacket ) {
    fwrite( healthpacket, sizeof(struct health), 1, fhealth );
}


// periodic console summary of attitude/location estimate
void display_message( struct imu *data, struct gps *gdata, struct nav *ndata,
                      struct servo *sdata, struct health *hdata )
{
    printf("[m/s^2]:ax  = %6.3f ay  = %6.3f az  = %6.3f \n",data->ax,data->ay,data->az);
    printf("[deg/s]:p   = %6.3f q   = %6.3f r   = %6.3f \n",data->p*57.3, data->q*57.3, data->r*57.3);
    printf("[deg  ]:phi = %6.2f the = %6.2f psi = %6.2f \n",data->phi*57.3,data->the*57.3,data->psi*57.3);
    printf("[Gauss]:hx  = %6.3f hy  = %6.3f hz  = %6.3f \n",data->hx,data->hy,data->hz);
    printf("[     ]:Ps  = %6.3f Pt  = %6.3f             \n",data->Ps,data->Pt);
    printf("[deg/s]:bp  = %6.3f,bq  = %6.3f,br  = %6.3f \n",xs[4]*57.3,xs[5]*57.3,xs[6]*57.3);
    if ( gdata->err_type == no_error ) {
        printf("[GPS  ]:ITOW= %5d[ms], lon = %f[deg], lat = %f[deg], alt = %f[m]\n",gdata->ITOW,gdata->lon,gdata->lat,gdata->alt);
    }
    if ( ndata->err_type == no_error ) {
        printf("[nav  ]:                 lon = %f[deg], lat = %f[deg], alt = %f[m]\n",            ndata->lon,ndata->lat,ndata->alt);	
    }
    printf("[Servo]: %d %d %d %d %d %d\n", sdata->chn[0], sdata->chn[1],
           sdata->chn[2], sdata->chn[3], sdata->chn[4], sdata->chn[5]);
    printf("[health]: v = %.2f sec = %d loadavg = %.2f\n",
           hdata->volts, hdata->est_seconds,
           (float)hdata->loadavg / 100.0);
    printf("\n");

}

