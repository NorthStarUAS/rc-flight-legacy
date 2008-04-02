#include <dirent.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <zlib.h>

#include "navigation/ahrs.h"

#include "logging.h"

// global variables for data file logging

static gzFile fimu = NULL;
static gzFile fgps = NULL;
static gzFile fnav = NULL;
static gzFile fservo = NULL;
static gzFile fhealth = NULL;

bool log_to_file = false;       // log to file is enabled/disabled
SGPath log_path;                // base log path
bool display_on = false;        // dump summary to display periodically


// scan the base path for fltNNNN directories.  Return the biggest
// flight number
int max_flight_num() {
    int max = -1;

    DIR *d = opendir( log_path.c_str() );
    if ( d == NULL ) {
        printf( "Cannot open %s for log writing\n", log_path.c_str() );
        return max;
    }

    struct dirent *file;
    while ( ( file = readdir(d) ) != NULL  ) {
        if ( strncmp( file->d_name, "flt", 3 ) == 0 ) {
            int num;
            sscanf( file->d_name, "flt%d", &num );
            if ( num > max ) {
                max = num;
            }
            printf("file = %s  num = %d  max = %d\n", file->d_name, num, max);
        }
    }

    if ( closedir( d ) != 0 ) {
        printf("Error: cannot close log directory\n");
        return -1;
    }

    return max;
}


bool logging_init() {
    // find the biggest flight number logged so far
    int max = max_flight_num();
    printf("Max log dir is flt%05d\n", max);


    // make the new logging directory
    char new_dir[256];
    snprintf( new_dir, 256, "%s/flt%05d", log_path.c_str(), max+1 );
    printf("Creating log dir: %s\n", new_dir);
    int result = mkdir( new_dir, 01777 );
    if ( result != 0 ) {
        printf("Error: creating %s\n", new_dir);
    }

    // open all the logging files

    SGPath file;

    file = new_dir; file.append( "imu.dat.gz" );
    if ( (fimu = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "gps.dat.gz" );
    if ( (fgps = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "nav.dat.gz" );
    if ( (fnav = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "servo.dat.gz" );
    if ( (fservo = gzopen( file.c_str(),"w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    file = new_dir; file.append( "health.dat.gz" );
    if ( (fhealth = gzopen( file.c_str(), "w+b" )) == NULL ) {
        printf("Cannont open %s\n", file.c_str());
        return false;
    }

    return true;
}


bool logging_close() {
    // close files

    gzclose(fimu);
    gzclose(fgps);
    gzclose(fnav);
    gzclose(fservo);
    gzclose(fhealth);

    return true;
}


void log_gps( struct gps *gpspacket ) {
    gzwrite( fgps, gpspacket, sizeof(struct gps) );
}


void log_imu( struct imu *imupacket ) {
    gzwrite( fimu, imupacket, sizeof(struct imu) );
}


void log_nav( struct nav *navpacket ) {
    gzwrite( fnav, navpacket, sizeof(struct nav) );
}


void log_servo( struct servo *servopacket ) {
    gzwrite( fservo, servopacket, sizeof(struct servo) );
}


void log_health( struct health *healthpacket ) {
    gzwrite( fhealth, healthpacket, sizeof(struct health) );
}


void flush_gps() {
    // printf("flush gps\n");
    gzflush( fgps, Z_SYNC_FLUSH );
}


void flush_imu() {
    // printf("flush imu\n");
    gzflush( fimu, Z_SYNC_FLUSH );
}


void flush_nav() {
    // printf("flush nav\n");
    gzflush( fnav, Z_SYNC_FLUSH );
}


void flush_servo() {
    // printf("flush servo\n");
    gzflush( fservo, Z_SYNC_FLUSH );
}


void flush_health() {
    // printf("flush health\n");
    gzflush( fhealth, Z_SYNC_FLUSH );
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
        int days = (int)(tmp / (24 * 60 * 60));
        tmp -= days * 24 * 60 * 60;
        int hours = (int)(tmp / (60 * 60));
        tmp -= hours * 60 * 60;
        int min = (int)(tmp / 60);
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
    printf("[health]: cmdseq = %d  tgtwp = %d  loadavg = %.2f\n",
           (int)hdata->command_sequence, (int)hdata->target_waypoint,
           (float)hdata->loadavg / 100.0);
    printf("\n");

    printf("imu size = %d\n", sizeof( struct imu ) );
}
