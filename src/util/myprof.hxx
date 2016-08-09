#ifndef _AURA_MYPROF_H
#define _AURA_MYPROF_H

#include <string>

using std::string;


class myprofile {

 private:

  int count;
  double init_time;
  double start_time;
  double end_time;
  double min_interval;
  double max_interval;
  double last_interval;
  double sum_time;
  string name;

 public:

  myprofile();
  ~myprofile();

  void set_name( const string _name );
  void start();
  void stop();
  void stats();
  inline double get_last_interval() { return last_interval; }
};


// global profiling structures
extern myprofile imu_prof;
extern myprofile gps_prof;
extern myprofile air_prof;
extern myprofile pilot_prof;
extern myprofile filter_prof;
extern myprofile mission_prof;
extern myprofile control_prof;
extern myprofile route_mgr_prof;
extern myprofile health_prof;
extern myprofile datalog_prof;
extern myprofile main_prof;
extern myprofile sync_prof;


#endif // _AURA_MYPROF_H
