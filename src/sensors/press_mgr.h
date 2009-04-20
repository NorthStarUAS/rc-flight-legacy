/**
 * \file: press_mgr.h
 *
 * Front end management interface for reading pressure data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: press_mgr.h,v 1.1 2009/04/20 01:53:02 curt Exp $
 */


#ifndef _UGEAR_PRESS_MGR_H
#define _UGEAR_PRESS_MGR_H


extern struct imu imupacket;

enum pressure_source_t {
    pressNone,
    pressMNAV
};

void Pressure_init();
bool Pressure_update();
void Pressure_close();


#endif // _UGEAR_PRESS_MGR_H
