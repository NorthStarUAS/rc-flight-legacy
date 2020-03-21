#pragma once

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
    bool enabled;

public:

    myprofile();
    ~myprofile();

    void set_name( const string _name );
    void start();
    void stop();
    void stats();
    inline double get_last_interval() { return last_interval; }
    inline void enable() { enabled = true; }
    inline void disable() { enabled = false; }
};


// global profiling structures
extern myprofile air_prof;
extern myprofile driver_prof;
extern myprofile filter_prof;
extern myprofile mission_prof;
extern myprofile control_prof;
extern myprofile health_prof;
extern myprofile datalog_prof;
extern myprofile main_prof;
extern myprofile sync_prof;
