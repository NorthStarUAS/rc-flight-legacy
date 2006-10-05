/*******************************************************************************
 * FILE: ground_station.h
 * DESCRIPTION: routines to do ethernet based communication with the ground
 *              station.
 *
 * SOURCE: 
 * REVISED: 9/02/05 Jung Soon Jang
 * REVISED: 4/07/06 Jung Soon Jang
 ******************************************************************************/

#ifndef _UNAV_GROUNDSTATION_H
#define _UNAV_GROUNDSTATION_H


//
// Ground station client communication routines
//
short open_client( void );
short send_client( void );
void close_client( void );


#endif // _UNAV_GROUNDSTATION_H
