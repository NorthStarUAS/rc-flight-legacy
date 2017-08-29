import math

from props import root, getNode

airdata_node = getNode('/sensors/airdata', True)
filter_node = getNode('/filters/filter[0]', True)
pilot_node = getNode('/sensors/pilot_input', True)
status_node = getNode('/status', True)
pos_node = getNode("/position", True)
vel_node = getNode("/velocity", True)
targets_node = getNode("/autopilot/targets", True)
tecs_node = getNode("/autopilot/tecs", True)
apm2_node = getNode("/sensors/APM2", True)
tecs_config_node = getNode("/config/autopilot/TECS", True)

r2d = 180.0 / math.pi
mps2kt = 1.9438444924406046432
kt2mps = 0.5144444444444444444
ft2m = 0.3048
g = 9.81

last_time = 0.0

def compute_tecs():
    mass_kg = tecs_config_node.getFloat("mass_kg")
    if mass_kg < 0.01: mass_kg = 3.0
    wt = tecs_config_node.getFloat("weight_tot")
    wb = tecs_config_node.getFloat("weight_bal")
    alt_m = filter_node.getFloat("altitude_m")
    vel_mps = vel_node.getFloat("airspeed_smoothed_kt") * kt2mps
    target_alt_m = targets_node.getFloat("altitude_msl_ft") * ft2m
    target_vel_mps = targets_node.getFloat("airspeed_kt") * kt2mps
    
    energy_pot = mass_kg * g * alt_m
    energy_kin = 0.5 * mass_kg * vel_mps * vel_mps

    target_pot = mass_kg * g * target_alt_m
    target_kin = 0.5 * mass_kg * target_vel_mps * target_vel_mps

    error_pot = target_pot - energy_pot
    error_kin = target_kin - energy_kin
    error_total = wt * error_pot + (2.0 - wt) * error_kin
    error_bal =  (2.0 - wb) * error_kin - wb * error_pot

    tecs_node.setFloat("energy_total", energy_pot + energy_kin )
    tecs_node.setFloat("target_total", target_pot + target_kin )
    tecs_node.setFloat("error_diff", error_bal)

def compute_derived_data():
    global last_time
    
    # compute ground track heading/speed
    vn = filter_node.getFloat("vn_ms")
    ve = filter_node.getFloat("ve_ms")
    vd = filter_node.getFloat("vd_ms")
    hdg = (math.pi * 0.5 - math.atan2(vn, ve)) * r2d
    vel_ms = math.sqrt( vn*vn + ve*ve + vd*vd )
    filter_node.setFloat("groundtrack_deg", hdg)
    filter_node.setFloat("groundspeed_ms", vel_ms)
    filter_node.setFloat("groundspeed_kt", vel_ms * mps2kt)

    # compute frame dt
    current_time = filter_node.getFloat('timestamp')
    dt = current_time - last_time
    last_time = current_time

    # local 'airborne' helper (not official)
    if vel_node.getFloat('airspeed_smoothed_kt') > 15:
        in_flight = True
    else:
        in_flight = False
    
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

    # autopilot error metrics
    roll_error = targets_node.getFloat('roll_deg') - filter_node.getFloat('roll_deg')
    #print 'error %.4f,%.1f' % (filter_node.getFloat('timestamp'), roll_error)
                        
    volts = apm2_node.getFloat("extern_volts")
    amps = apm2_node.getFloat("extern_amps")
    watts = volts * amps
    apm2_node.setFloat("extern_watts", watts)

    # TECS
    compute_tecs()
