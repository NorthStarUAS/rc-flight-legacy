# from http://www.edwilliams.org/avform.htm (aviation formulas)

import math

from props import getNode

d2r = math.pi / 180.0
r2d = 180.0 / math.pi
k2pi = math.pi * 2

orient_node = getNode('/orientation', True)
wind_node = getNode('/filters/wind', True)


# Given a wind speed estimate, true airspeed estimate, wind direction
# estimate, and a desired course to fly, compute a true heading to fly
# to achieve the desired course in the given wind.  Also compute the
# estimated ground speed.  Returns estimated aircraft true heading and
# estimated ground speed.

def wind_course( ws_kt, tas_kt, wd_deg, crs_deg ):
    wd = wd_deg * d2r
    crs = crs_deg * d2r
    hd = 0.0
    hd_deg = 0.0
    gs_kt = 0.0
    if tas_kt > 0.1:
        # printf("ws=%.1f tas=%.1f wd=%.1f crs=%.1f\n", ws, tas, wd, crs)
        swc = (ws_kt/tas_kt) * math.sin(wd-crs)
        # printf("swc=%.2f\n", swc)
        if abs(swc) > 1.0:
            # course cannot be flown, wind too strong
            return None, None
        else:
            hd = crs + math.asin(swc)
            if hd < 0.0: hd += k2pi
            if hd > k2pi: hd -= k2pi
            gs_kt = tas_kt * math.sqrt(1-swc*swc) - ws_kt * math.cos(wd - crs)
            if gs_kt < 0.0:
                # course cannot be flown, wind too strong
                return None, None
        hd_deg = hd * r2d
    # if display_on:
    #   printf("af: hd=%.1f gs=%.1f\n", hd * SGD_RADIANS_TO_DEGREES, gs)
    return hd_deg, gs_kt


