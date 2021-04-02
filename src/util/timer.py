# Simple time measuring and stamping routines

import time

# void print_Time_Resolution()
# {
#     struct timespec res;
#     clock_getres(CLOCK_MONOTONIC, &res);
#     printf("CLOCK_MONOTONIC resolution = %ld sec, %ld nanosec\n", res.tv_sec,
# 	   res.tv_nsec);
# }

tstart = None
def get_pytime():
    global tstart
    tnow = time.clock_gettime(time.CLOCK_MONOTONIC)
    if tstart is None:
        tstart = tnow
    return tnow - tstart

def get_realtime():
    return time.clock_gettime(time.CLOCK_REALTIME)
