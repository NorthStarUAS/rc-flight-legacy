#include "timing.h"
#include "ratelimiter.h"

RateLimiter::RateLimiter() {
    dt_millis = 1000;
}

RateLimiter::RateLimiter( float hz ) {
    dt_millis = 1000.0 / hz;
}

bool RateLimiter::update() {
    uint32_t millis = get_Time()*1000;
    if ( timer == 0 ) {
        timer = millis;
    }
    if ( millis >= timer + dt_millis ) {
        if ( millis > timer + dt_millis ) {
            misses++;                 // oops
            timer = millis;           // catchup
        } else {
            timer += dt_millis;       // all good
        }
        return true;
    } else {
        return false;
    }
}
