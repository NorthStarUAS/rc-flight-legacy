# Simple profiling assistant

from comms import events
from util import timer

class Profile():
    init_time = None
    count = 0
    sum_time = 0.0
    max_interval = 0.0
    min_interval = 1000.0
    enabled = True
    
    def __init__(self, name):
        self.name = name

        
    def start(self):
        if not self.enabled:
            return
        
        if self.init_time is None:
            self.init_time = timer.get_pytime()

        self.start_time = timer.get_pytime()
        self.count += 1

    def stop(self):
        if not self.enabled:
            return
        
        stop_time = timer.get_pytime()
        last_interval = stop_time - self.start_time
        self.sum_time += last_interval
        
        # log situations where a module took longer that 0.10 sec to execute
        if last_interval > 0.10:
            msg = "t1 = %.3f t2 = %.3f int = %.3f" % (start_time, stop_time, last_interval)
            events.log(self.name, msg )

        if last_interval < self.min_interval:
            self.min_interval = last_interval
        if last_interval > self.max_interval:
            self.max_interval = last_interval

    def stats(self):
        if not self.enabled:
            return
    
        total_time = timer.get_pytime() - self.init_time
        avg_hz = 0.0
        if total_time > 0.0:
            avg_hz = self.count / total_time
        print("%s avg: %.2f(ms) num: %d tot: %.4f(s) (range: %.2f-%.2f) hz: %.1f" % (self.name, 1000.0 * self.sum_time / self.count, self.count, self.sum_time, 1000.0 * self.min_interval, 1000.0 * self.max_interval, avg_hz) )

    def enable(self):
        self.enabled = True
        
# global profiling structures
airdata_prof = Profile("airdata")
driver_prof = Profile("drivers")
filter_prof = Profile("filter")
mission_prof = Profile("mission")
control_prof = Profile("control")
health_prof = Profile("health")
datalog_prof = Profile("logger")
main_prof = Profile("main")
sync_prof = Profile("sync")
