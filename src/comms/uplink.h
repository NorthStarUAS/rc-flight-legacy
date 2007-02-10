#ifndef _UGEAR_UPLINK_H
#define _UGEAR_UPLINK_H


// constants
#define maxwaypoints    8

// global variables
extern short  whichmode[6];
extern double pitch_gain[3],roll_gain[3];
extern double heading_gain[3],alt_gain[3],pos_gain[3];
extern int    numofwaypoints;     
extern double waypoints[maxwaypoints][2];
extern char   uplinkstr[80];
extern bool   retvalsock;         //socket status

// global functions
void uplink_acq();


#endif // _UGEAR_UPLINK_H
