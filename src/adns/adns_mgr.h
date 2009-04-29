/**
 * \file: adns_mgr.h
 *
 * Front end management interface for executing the available ADNS codes.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: adns_mgr.h,v 1.1 2009/04/29 01:46:14 curt Exp $
 */


#ifndef _UGEAR_ADNS_MGR_H
#define _UGEAR_ADNS_MGR_H


void ADNS_init();
bool ADNS_update( bool fresh_imu_data );
void ADNS_close();


#endif // _UGEAR_ADNS_MGR_H
