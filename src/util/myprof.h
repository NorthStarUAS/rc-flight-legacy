#ifndef _UGEAR_MYPROF_H
#define _UGEAR_MYPROF_H


class myprofile {

 private:

  int count;
  double start_time;
  double end_time;
  double last_interval;
  double total_time;

 public:

  myprofile();
  ~myprofile();

  void start();
  void stop();
  void stats( const char *header );
};


// global profiling structures
extern myprofile filter_prof;
extern myprofile mnav_prof;
extern myprofile ahrs_prof;
extern myprofile nav_prof;
extern myprofile control_prof;
extern myprofile route_mgr_prof;
extern myprofile health_prof;
extern myprofile main_prof;


#endif // _UGEAR_MYPROF_H
