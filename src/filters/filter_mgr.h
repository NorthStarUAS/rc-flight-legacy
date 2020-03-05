/**
 * \file: filter_mgr.h
 *
 * Front end management interface for executing the available filter codes.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson <at> gmail <dot> com
 *
 * $Id: adns_mgr.h,v 1.1 2009/04/29 01:46:14 curt Exp $
 */


#pragma once

void Filter_init();
bool Filter_update();
void Filter_close();
