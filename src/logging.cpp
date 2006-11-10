#include <stdio.h>

#include "ahrs.h"
#include "logging.h"


static FILE *fimu = NULL;
static FILE *fgps = NULL;
static FILE *fnav = NULL;
static FILE *fservo = NULL;

#ifdef NCURSE_DISPLAY_OPTION
#include <ncurses/ncurses.h>
WINDOW *win;
#endif // NCURSE_DISPLAY_OPTION


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

    return true;
}


bool logging_close() {
    // close files

    fclose(fimu);
    fclose(fgps);
    fclose(fnav);
    fclose(fservo);

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


// ncurses related functions
#ifdef NCURSE_DISPLAY_OPTION
static void set_colors(int pair, int foreground, int background)
{
    if (has_colors()) {
   	if (pair > COLOR_PAIRS)
   	    pair = COLOR_PAIRS;
   	init_pair(pair, foreground,background);
   	wattrset(win, COLOR_PAIR(pair));
    }		    
}

static chtype use_colors(int pairs, chtype attrs)
{
    if (has_colors()) {
   	if (pairs > COLOR_PAIRS)
   	    pairs = COLOR_PAIRS;
   	attrs |= COLOR_PAIR(pairs);
    }
    wattrset(win,attrs);
    return attrs;	    	
}		
#endif


// periodic console summary of attitude/location estimate
void display_message(struct imu *data, struct gps *gdata, struct nav *ndata, int disptime)
{
    static int count=0;
    // static double r2d = 57.3;
    // char buf[100];

#ifdef NCURSE_DISPLAY_OPTION   
    if(++count == disptime) {
   	set_colors(1,COLOR_RED,COLOR_BLACK);
	box(win, ACS_VLINE, ACS_HLINE);
	set_colors(2,COLOR_WHITE,COLOR_BLACK);
        mvwaddstr(win, 0, 4," MNAV-100CA AHRS and INS Information ");
        set_colors(3,COLOR_GREEN,COLOR_BLACK);
        mvwaddstr(win, 2, 4,">> Attitude Heading Reference System (AHRS):");    	
        set_colors(2,COLOR_WHITE,COLOR_BLACK);
	sprintf(buf,"[deg  ]:phi = %6.2f    the = %6.2f    psi = %6.1f",data->phi*r2d,data->the*r2d,data->psi*r2d);
   	mvwaddstr(win, 3, 4,buf);    	
   	sprintf(buf,"[deg/s]:p   = %6.3f    q   = %6.3f    r   = %6.3f",data->p*r2d,data->q*r2d,data->r*r2d);
   	mvwaddstr(win, 4, 4,buf);      	
        sprintf(buf,"[m/s^2]:ax  = %6.3f    ay  = %6.3f    az  = %6.3f",data->ax,data->ay,data->az);
        mvwaddstr(win, 5, 4,buf);   	
	sprintf(buf,"[Gauss]:hx  = %6.3f    hy  = %6.3f    hz  = %6.3f",data->hx,data->hy,data->hz);
        mvwaddstr(win, 6, 4,buf);
        sprintf(buf,"[Press]:Pt  = %6.1f    Pv  = %6.2f",data->Ps,data->Pt);
        mvwaddstr(win, 7, 4,buf);
        sprintf(buf,"[bias ]:bp  = %6.3f    bq  = %6.3f    br  = %6.3f ",xs[4]*r2d,xs[5]*r2d,xs[6]*r2d);
        mvwaddstr(win, 8, 4,buf);
        
	if ( ndata->err_type == no_error ) {
            set_colors(3,COLOR_GREEN,COLOR_BLACK);
            mvwaddstr(win, 10, 4,">> Strapdown Inertial Navigation System (S-INS):");   
            set_colors(2,COLOR_WHITE,COLOR_BLACK); 	
            sprintf(buf,"[GPS  ]:lon = %f  lat = %f  alt = %6.2f",gdata->lon,gdata->lat,gdata->alt);
            mvwaddstr(win, 11, 4,buf);
            sprintf(buf,"[nav  ]:lon = %f  lat = %f  alt = %6.2f",ndata->lon,ndata->lat,ndata->alt);
            mvwaddstr(win, 12, 4,buf);
            sprintf(buf,"[bias ]:bax = %6.3f       bay = %6.3f     baz = %6.3f",nxs[6][0],nxs[7][0],nxs[8][0]);
            mvwaddstr(win, 13, 4,buf);

            sprintf(buf,"[MNAV-NAV]:The cycles in %5.2f (Hz):%6.2f (ms)\t\t",1/exe_rate[1], exe_rate[1]*1000);
            mvwaddstr(win, 16, 4,buf);
        } else {
            set_colors(3,COLOR_GREEN,COLOR_BLACK);
            mvwaddstr(win, 10, 4,">> Strapdown Inertial Navigation System (S-INS):"); 
          
            if (gps_init_count !=0) { 
                sprintf(buf,"GPS Acquisition for NAV Initialization (%d)    ",20-gps_init_count);
                mvwaddstr(win,11,4,buf);	
            } else {	
                set_colors(1,COLOR_RED,COLOR_BLACK);
                mvwaddstr(win, 11, 4,"GPS is not available...");   	
            }   
        }
        set_colors(2,COLOR_WHITE,COLOR_BLACK);
        sprintf(buf,"[MNAV-ACQ]:The cycles in %5.2f (Hz):%6.2f (ms)\t\t",1/exe_rate[0], exe_rate[0]*1000);
        mvwaddstr(win, 14, 4,buf);
        sprintf(buf,"[MNAV-TCP]:The cycles in %5.2f (Hz):%6.2f (ms)\t\t",1/exe_rate[2], exe_rate[2]*1000);
        mvwaddstr(win, 15, 4,buf);

        set_colors(4,COLOR_YELLOW,COLOR_BLACK);
        mvwaddstr(win, 18, 4,uplinkstr);

        set_colors(2,COLOR_WHITE,COLOR_BLACK);
        sprintf(buf,"[GND-BASE]:I.P. Address >> %s",HOST_IP_ADDR);
        mvwaddstr(win, 19, 4,buf);
        set_colors(4,COLOR_YELLOW,COLOR_BLACK);
        sprintf(buf,"[uNAV    ]:%s      ",buf_err); 
        mvwaddstr(win, 20, 4,buf);
        
        set_colors(2,COLOR_WHITE,COLOR_BLACK);
        sprintf(buf,"[CONTROL ]:%s",cnt_status);
        mvwaddstr(win, 21, 4,buf);

        mvwaddstr(win, 22, 50," Ctrl+C to Quit ");
        wrefresh(win); 
        nodelay(win, true);
        count=0;
    }
#else  	
    if (++count == disptime) {
        printf("[m/s^2]:ax  = %6.3f ay  = %6.3f az  = %6.3f \n",data->ax,data->ay,data->az);
        printf("[deg/s]:p   = %6.3f q   = %6.3f r   = %6.3f \n",data->p*57.3, data->q*57.3, data->r*57.3);
        printf("[deg  ]:phi = %6.2f the = %6.2f psi = %6.2f \n",data->phi*57.3,data->the*57.3,data->psi*57.3);
        printf("[Gauss]:hx  = %6.3f hy  = %6.3f hz  = %6.3f \n",data->hx,data->hy,data->hz);
        printf("[     ]:Ps  = %6.3f Pt  = %6.3f             \n",data->Ps,data->Pt);
        printf("[deg/s]:bp  = %6.3f,bq  = %6.3f,br  = %6.3f \n\n",xs[4]*57.3,xs[5]*57.3,xs[6]*57.3);
        if ( ndata->err_type == no_error ) {
            printf("[GPS  ]:ITOW= %5d[ms], lon = %f[deg], lat = %f[deg], alt = %f[m]\n",gdata->ITOW,gdata->lon,gdata->lat,gdata->alt);	
            printf("[nav  ]:                 lon = %f[deg], lat = %f[deg], alt = %f[m]\n",            ndata->lon,ndata->lat,ndata->alt);	
        }

        count = 0;
    }	
#endif

}
