# Estimate wind vector given indicated airspeed, aircraft heading
# (true), and gps ground velocity vector.  This function is designed
# to be called repeatedly to update the wind estimate in real time.

import math
from tqdm import tqdm

from rcUAS_flightdata import flight_interp

import lowpass

# useful constants
d2r = math.pi / 180.0
r2d = 180.0 / math.pi
mps2kt = 1.94384
kt2mps = 1 / mps2kt

class Wind():
    def __init__(self):
        self.pitot_time_factor = 240
        self.psi_error_time_factor = 60
        self.wind_tf_long = 60
        self.wind_tf_short = 1
        self.filt_ps = lowpass.LowPassFilter(self.pitot_time_factor, 1.0)
        self.filt_psi_error = lowpass.LowPassFilter(self.psi_error_time_factor, 1.0)
        self.filt_long_wn = lowpass.LowPassFilter(self.wind_tf_long, 0.0)
        self.filt_long_we = lowpass.LowPassFilter(self.wind_tf_long, 0.0)
        self.filt_short_wn = lowpass.LowPassFilter(self.wind_tf_short, 0.0)
        self.filt_short_we = lowpass.LowPassFilter(self.wind_tf_short, 0.0)
        self.ps = 1

    def update(self, time, airspeed_kt, yaw_rad, vn, ve):
        dt = 0.0
        if self.last_time > 0:
            dt = time - self.last_time
        self.last_time = time

        if dt > 0.0 and airspeed_kt >= 10.0:
            # update values if 'flying' and time has elapsed
            psi = 0.5*math.pi - yaw_rad
            psi += self.filt_psi_error.value

            # estimate body velocity
            ue = math.cos(psi) * (airspeed_kt * self.filt_ps.value * kt2mps)
            un = math.sin(psi) * (airspeed_kt * self.filt_ps.value * kt2mps)
            # print("yaw_deg: %0f psi_deg: %.2f" % (yaw_rad*r2d, psi*r2d),
            #       "ue: %.1f un: %.1f" % (ue, un),
            #       "ve: %.1f vn: %.1f" % (ve, vn))
            # instantaneous wind velocity
            we = ve - ue
            wn = vn - un

            # filtered wind velocity
            self.filt_long_wn.update(wn, dt)
            self.filt_long_we.update(we, dt)
            self.filt_short_wn.update(wn, dt)
            self.filt_short_we.update(we, dt)

            # estimate true airspeed
            true_e = ve - self.filt_long_we.value
            true_n = vn - self.filt_long_wn.value

            # estimate aircraft 'true' heading
            true_psi = math.atan2(true_n, true_e)
            psi_error = true_psi - psi
            if psi_error < -math.pi: psi_error += 2*math.pi
            if psi_error > math.pi: psi_error -= 2*math.pi
            self.filt_psi_error.update(psi_error, dt)
            #print(self.filt_psi_error.value*r2d, psi_error*r2d)

            # estimate pitot tube bias
            true_speed_kt = math.sqrt( true_e*true_e + true_n*true_n ) * mps2kt
            self.ps = true_speed_kt / airspeed_kt
            #print("asi: %.1f  true(est): %.1f true(act): %.1f scale: %.2f" % (airspeed_kt, airspeed_kt * self.filt_ps.value, true_speed_kt, self.filt_ps.value))
            # don't let the scale factor exceed some reasonable limits
            if self.ps < 0.75: self.ps = 0.75
            if self.ps > 1.25: self.ps = 1.25
            self.filt_ps.update(self.ps, dt)

    # run a quick wind estimate and pitot calibration based on nav
    # estimate + air data
    def estimate(self, data, wind_tf_long):
        print("Estimating winds aloft:")
        if wind_tf_long:
            self.wind_tf_long = wind_tf_long
        self.last_time = 0.0
        winds = []
        airspeed = 0
        psi = 0
        vn = 0
        ve = 0
        wind_deg = 0
        wind_kt = 0
        ps = 1.0
        interp = flight_interp.InterpolationGroup(data)
        iter = flight_interp.IterateGroup(data)
        for i in tqdm(range(iter.size())):
            record = iter.next()
            if len(record):
                t = record['imu']['time']
                if 'air' in record:
                    airspeed = record['air']['airspeed']
                filt = interp.query(t, 'filter')
                if not len(filt):
                    continue
                phi = filt['phi']
                psi = filt['psi']
                vn = filt['vn']
                ve = filt['ve']
                #print("Phi:", phi)
                if airspeed > 15.0: # and abs(phi) > 0.2 and abs(phi) < 0.3:
                    self.update(t, airspeed, psi, vn, ve)
                    wn = self.filt_long_wn.value
                    we = self.filt_long_we.value
                    psi_error = self.filt_psi_error.value
                    #print wn, we, math.atan2(wn, we), math.atan2(wn, we)*r2d
                    wind_deg = 90 - math.atan2(wn, we) * r2d
                    if wind_deg < 0: wind_deg += 360.0
                    wind_kt = math.sqrt( we*we + wn*wn ) * mps2kt
                    #print wn, we, ps, wind_deg, wind_kt
                    # make sure we log one record per each imu record
                    winds.append( { 'time': t,
                                    'wind_deg': wind_deg,
                                    'wind_kt': wind_kt,
                                    'pitot_scale': self.filt_ps.value,
                                    'long_wn': self.filt_long_wn.value,
                                    'long_we': self.filt_long_we.value,
                                    'short_wn': self.filt_short_wn.value,
                                    'short_we': self.filt_short_we.value,
                                    'psi_error': self.filt_psi_error.value,
                                    'phi': phi,
                                    'ps': self.ps
                                   } )
        return winds
