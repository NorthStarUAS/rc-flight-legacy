#include <stdio.h>

#include "navigation/ahrs.h"

#include "logging.h"

// global variables for data file logging

static FILE *fimu = NULL;
static FILE *fgps = NULL;
static FILE *fnav = NULL;
static FILE *fservo = NULL;
static FILE *fhealth = NULL;

bool log_to_file = false;       // log to file is enabled/disabled
SGPath log_path;                // base log path
bool display_on = false;        // dump summary to display periodically


bool logging_init() {
    SGPath file;

    // open files

    file = log_path; file.append( "imu.dat" );
    if ( (fimu = fopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = log_path; file.append( "gps.dat" );
    if ( (fgps = fopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = log_path; file.append( "nav.dat" );
    if ( (fnav = fopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = log_path; file.append( "servo.dat" );
    if ( (fservo = fopen( file.c_str(),"w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = log_path; file.append( "health.dat" );
    if ( (fhealth = fopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
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
        double tmp = gdata->ITOW;
        int days = tmp / (24 * 60 * 60);
        tmp -= days * 24 * 60 * 60;
        int hours = tmp / (60 * 60);
        tmp -= hours * 60 * 60;
        int min = tmp / 60;
        tmp -= min * 60;
        double sec = tmp;
        printf("[GPS  ]:ITOW= %.3f[sec]  %dd %02d:%02d:%06.3f\n", gdata->ITOW, days, hours, min, sec);
        printf("[GPS  ]:lon = %f[deg], lat = %f[deg], alt = %f[m]\n",gdata->lon,gdata->lat,gdata->alt);
    }
    if ( ndata->err_type == no_error ) {
        printf("[nav  ]:lon = %f[deg], lat = %f[deg], alt = %f[m]\n",            ndata->lon,ndata->lat,ndata->alt);	
    }
    printf("[Servo]: %d %d %d %d %d %d\n", sdata->chn[0], sdata->chn[1],
           sdata->chn[2], sdata->chn[3], sdata->chn[4], sdata->chn[5]);
    printf("[health]: v = %.2f sec = %d loadavg = %.2f\n",
           hdata->volts, hdata->est_seconds,
           (float)hdata->loadavg / 100.0);
    printf("\n");

    // printf("imu size = %d\n", sizeof( struct imu ) );
}
