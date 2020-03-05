#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "butter.h"
#include "lowpass.h"

// cutoff freq of 1
static float pitot_butter_filt(float input) {
    const int NZEROS = 2;
    const int NPOLES = 2;
    const float GAIN = 1.058546241e+03;
    static float xv[NZEROS+1], yv[NPOLES+1];
    
    xv[0] = xv[1]; xv[1] = xv[2]; 
    xv[2] = input / GAIN;
    yv[0] = yv[1]; yv[1] = yv[2]; 
    yv[2] = (xv[0] + xv[2]) + 2 * xv[1]
        + ( -0.9149758348  * yv[0]) + (  1.9111970674 * yv[1]);
    return yv[2];
}

int main() {
    ButterworthFilter bf(2, 100, 1.5);
    LowPassFilter lp(0.5);

    float dt = 0.01; // 100hz
    float time = 0.0;
    while (time < 20.0) {
        double signal = sin(time) + 0.5 * cos(time*time) - 0.25 * sin(time*time*time);
        signal += drand48() * 1.0 - 0.5;
        double pbf = pitot_butter_filt(signal);
        double gbf = bf.update(signal);
        double lpf = lp.update(signal, dt);
        
        printf("%.4f %.4f %.4f %.4f\n", signal, pbf, gbf, lpf);
        time += dt;
    }
}
