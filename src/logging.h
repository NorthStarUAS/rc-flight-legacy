#ifndef _UGEAR_LOGGING_H
#define _UGEAR_LOGGING_H


#include "globaldefs.h"

// global variables

#ifdef NCURSE_DISPLAY_OPTION
#include <ncurses/ncurses.h>
extern WINDOW *win;
#endif // NCURSE_DISPLAY_OPTION

extern bool log_to_file;


// global functions

bool logging_init();
bool logging_close();

void log_gps( struct gps *gpspacket );
void log_imu( struct imu *imupacket );
void log_nav( struct nav *navpacket );
void log_servo( struct servo *servopacket );

void display_message( struct imu *data, struct gps *gdata,
                      struct nav *ndata, int disptime );

#endif // _UGEAR_LOGGING_H
