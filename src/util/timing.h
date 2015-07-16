#ifndef _UGEAR_TIMING_H
#define _UGEAR_TIMING_H


extern void snap_time_interval( const char *threadname,
				int displaytime, short id );
extern double get_time_interval( short id );
extern double get_Time();
extern double get_RealTime();

#endif // _UGEAR_TIMING_H
