# From: http://williams.best.vwh.net/avform.htm#GCF

import math

EPS = 0.0001

d2r = math.pi / 180.0
r2d = 180.0 / math.pi
rad2nm = (180.0 * 60.0) / math.pi
nm2meter = 1852

# p1 = (lat1, lon1)
# p2 = (lat2, lon2)
def course_and_dist(p1, p2):
    # this formulations uses postive lon = W (opposite of usual, so we
    # invert the longitude.)
    lat1 = p1[0] * d2r
    lon1 = -p1[1] * d2r
    lat2 = p2[0] * d2r
    lon2 = -p2[1] * d2r

    dist_rad = 2.0 * math.asin(math.sqrt((math.sin((lat1-lat2)/2.0))**2.0 + math.cos(lat1)*math.cos(lat2)*(math.sin((lon1-lon2)/2.0))**2))

    # if starting point is on a pole
    if math.cos(lat1) < EPS:
        # EPS a small number ~ machine precision
        if (lat1 > 0.0):
            # starting from N pole
            tc1_rad = math.pi
        else:
            # starting from S pole
            tc1_rad = 2.0 * math.pi

    # For starting points other than the poles:
    if math.sin(lon2-lon1) < 0.0:
        tc1_rad = math.acos((math.sin(lat2)-math.sin(lat1)*math.cos(dist_rad))/(math.sin(dist_rad)*math.cos(lat1)))    
    else:
        tc1_rad = 2.0 * math.pi - math.acos((math.sin(lat2)-math.sin(lat1)*math.cos(dist_rad))/(math.sin(dist_rad)*math.cos(lat1)))    

    dist_nm = dist_rad * rad2nm
    dist_m = dist_nm * nm2meter

    tc1_deg = tc1_rad * r2d

    return (tc1_deg, dist_m)
