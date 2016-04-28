import math

from props import root, getNode

filter_node = getNode('/filters/filter', True)

r2d = 180.0 / math.pi
mps2kt = 1.9438444924406046432

def compute_derived_data():
    # compute ground track heading/speed
    vn = filter_node.getFloat("vn_ms")
    ve = filter_node.getFloat("ve_ms")
    vd = filter_node.getFloat("vd_ms")
    hdg = (math.pi * 0.5 - math.atan2(vn, ve)) * r2d
    speed = math.sqrt( vn*vn + ve*ve + vd*vd ) * mps2kt
    filter_node.setFloat("track_deg", hdg)
    filter_node.setFloat("speed_kt", speed)
