#include <stdio.h>

#include "timing.h"

#include "myprof.h"

myprofile::myprofile() {
  count = 0;
  total_time = 0.0;
}

myprofile::~myprofile() {
}

void myprofile::start() {
  start_time = get_Time();
  count++;
}

void myprofile::stop() {
  last_interval = get_Time() - start_time;
  total_time += last_interval;
}

void myprofile::stats( const char *header ) {
  printf("%s: avg = %.4f  count = %d  total = %.4f  (last = %.4f)\n",
	 header, total_time / (double)count, count, total_time, last_interval );
}


// global profiling structures
myprofile mnav_prof;
myprofile ahrs_prof;
myprofile nav_prof;
myprofile filter_prof;
myprofile control_prof;
myprofile route_mgr_prof;
myprofile health_prof;
myprofile main_prof;
