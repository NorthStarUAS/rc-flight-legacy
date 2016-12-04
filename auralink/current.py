import math

from props import root, getNode

airdata_node = getNode('/sensors/airdata', True)
filter_node = getNode('/filters/filter', True)
pilot_node = getNode('/sensors/pilot_input', True)
status_node = getNode('/status', True)
vel_node = getNode("/velocity", True)

r2d = 180.0 / math.pi
mps2kt = 1.9438444924406046432
kt2mps = 0.5144444444444444444

last_time = 0.0

def compute_derived_data():
    global last_time
    
    # compute ground track heading/speed
    vn = filter_node.getFloat("vn_ms")
    ve = filter_node.getFloat("ve_ms")
    vd = filter_node.getFloat("vd_ms")
    hdg = (math.pi * 0.5 - math.atan2(vn, ve)) * r2d
    vel_ms = math.sqrt( vn*vn + ve*ve + vd*vd )
    filter_node.setFloat("groundtrack_deg", hdg)
    filter_node.setFloat("speed_ms", vel_ms)

    # local 'airborne' helper (not official)
    if vel_node.getFloat('airspeed_smoothed_kt') > 15:
        in_flight = True
    else:
        in_flight = False
    
    # compute frame dt
    current_time = filter_node.getFloat('timestamp')
    dt = current_time - last_time
    last_time = current_time

    # local flight timer
    if in_flight:
        timer = status_node.getFloat('local_flight_timer')
        timer += dt
        status_node.setFloat('local_flight_timer', timer)

    # local autopilot timer
    ap_enabled = False
    if pilot_node.getFloatEnum("channel", 7) > 0:
        ap_enabled = True
        
    if in_flight and ap_enabled:
        timer = status_node.getFloat('local_autopilot_timer')
        timer += dt
        status_node.setFloat('local_autopilot_timer', timer)

    # estimate distance traveled from filter velocity and dt
    if in_flight:
        od = status_node.getFloat('flight_odometer')
        od += vel_ms * dt
        status_node.setFloat('flight_odometer', od)
