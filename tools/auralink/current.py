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
power_node = getNode("/sensors/power", True)
tecs_config_node = getNode("/config/autopilot/TECS", True)

r2d = 180.0 / math.pi
mps2kt = 1.9438444924406046432
kt2mps = 0.5144444444444444444
ft2m = 0.3048
g = 9.81

last_time = 0.0

# crude battery % interpolation model
# 100 - 4.2
# 83% - 3.8
# 27% - 3.65
# 0%  - 3.5
batv = [ 3.3, 3.50, 3.65, 3.80, 4.20 ]
batp = [ 0.0, 0.05, 0.27, 0.83, 1.00 ]
from scipy.interpolate import interp1d
batf = interp1d(batv, batp)
filt_perc = 1.0

def compute_tecs():
    if filter_node.getFloat('timestamp') < 0.01:
        # do nothing if filter not inited
        return
    
    mass_kg = tecs_config_node.getFloat("mass_kg")
    if mass_kg < 0.01:
        mass_kg = 3.0
    if tecs_config_node.hasChild("weight_bal"):
        wb = tecs_config_node.getFloat("weight_bal")
    else:
        wb = 1.0
    # fixem:
    wb = 0.0
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
    # print(filter_node.getFloat('timestamp'), 'target_alt:', target_alt_m, 'tgt_pot:', target_pot, 'E_pot:', energy_pot, 'Err_kin:', error_kin, 'Err_pot:', error_pot)
    error_total = error_pot + error_kin
    error_bal =  (2.0 - wb) * error_kin - wb * error_pot

    tecs_node.setFloat("energy_total", energy_pot + energy_kin )
    tecs_node.setFloat("target_total", target_pot + target_kin )
    tecs_node.setFloat("error_total", error_total)
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
    if vel_node.getFloat('airspeed_smoothed_kt') >= 15:
        in_flight = True
    else:
        in_flight = False
    status_node.setBool("in_flight", in_flight)
    
    # local autopilot timer
    ap_enabled = False
    if pilot_node.getFloatEnum("channel", 0) > 0:
        ap_enabled = True
        
    if in_flight and ap_enabled:
        timer = status_node.getFloat('local_autopilot_timer')
        timer += dt
        status_node.setFloat('local_autopilot_timer', timer)

    # estimate distance traveled from filter velocity and dt
    if in_flight:
        if not status_node.getBool('onboard_flight_time'):
            ft = status_node.getFloat('flight_timer')
            ft += dt
            status_node.setFloat('flight_timer', ft)
        od = status_node.getFloat('flight_odometer')
        od += vel_ms * dt
        status_node.setFloat('flight_odometer', od)

    throttle_timer = status_node.getFloat("throttle_timer")
    if pilot_node.getFloatEnum("channel", 2) > 0.1:
        throttle_timer += dt
    status_node.setFloat("throttle_timer", throttle_timer)
        
    # autopilot error metrics
    roll_error = targets_node.getFloat('roll_deg') - filter_node.getFloat('roll_deg')
    #print 'error %.4f,%.1f' % (filter_node.getFloat('timestamp'), roll_error)
                        
    volts = power_node.getFloat("main_vcc")
    amps = power_node.getFloat("main_amps")
    watts = volts * amps
    power_node.setFloat("main_watts", watts)

    cell_volts = power_node.getFloat("cell_vcc")
    if cell_volts < 3.3: cell_volts = 3.3
    if cell_volts > 4.2: cell_volts = 4.2
    batt_perc = batf(cell_volts)
    global filt_perc
    if filt_perc is None:
        filt_perc = batt_perc
    else:
        filt_perc = 0.9995 * filt_perc + 0.0005 * batt_perc
    power_node.setFloat("battery_perc", filt_perc)
    
    # TECS
    compute_tecs()
