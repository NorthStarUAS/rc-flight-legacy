import math

from props import getNode

r2d = 180.0 / math.pi

# initialize property nodes
comms_node = getNode('/comms', True)
imu_node = getNode('/sensors/imu', True)
gps_node = getNode('/sensors/gps', True)
velocity_node = getNode('/velocity', True)
filter_node = getNode('/filters/filter', True)
orient_node = getNode('/orientation', True)
pos_pressure_node = getNode('/position/pressure', True)
act_node = getNode('/actuators', True)
remote_link_node = getNode('/comms/remote_link', True)
route_node = getNode('/task/route', True)
status_node = getNode('/status', True)
power_node = getNode('/sensors/power', True)

# make the C++ interface happy
def init():
    pass

# show a display message if display is enabled
def show(message):
    if comms_node.getBool('display_on'):
        print(message)
        
# periodic console summary of attitude/location estimate
def status_summary():
    if comms_node.getBool('display_on'):
        print('[imu  ]:gyro = %.3f %.3f %.3f [deg/s]' % \
              (imu_node.getFloat('p_rad_sec') * r2d,
               imu_node.getFloat('q_rad_sec') * r2d,
               imu_node.getFloat('r_rad_sec') * r2d),
              'accel = %.3f %.3f %.3f [m/s^2]' % \
              (imu_node.getFloat('ax_mps_sec'),
               imu_node.getFloat('ay_mps_sec'),
               imu_node.getFloat('az_mps_sec')))
        print('[mag  ]:%.3f %.3f %.3f' % \
              (imu_node.getFloat('hx'),
               imu_node.getFloat('hy'),
               imu_node.getFloat('hz')))
        print('[air  ]:Palt = %5.2f[m] Pspd = %4.1f[kt]' % \
              (pos_pressure_node.getFloat('altitude_m'),
               velocity_node.getFloat('airspeed_kt')))

        if gps_node.getFloat('data_age') < 10.0:
            print('[gps  ]:date = %04d/%02d/%02d %02d:%02d:%02d' % \
                  (gps_node.getInt('year'),
                   gps_node.getInt('month'),
                   gps_node.getInt('day'),
                   gps_node.getInt('hour'),
                   gps_node.getInt('min'),
                   gps_node.getInt('sec')))
            print('[gps  ]:lon = %.6f lat = %.6f alt = %.1f[m] sats = %ld, age = %.2f' % \
                  (gps_node.getFloat('longitude_deg'),
                   gps_node.getFloat('latitude_deg'),
                   gps_node.getFloat('altitude_m'),
                   gps_node.getInt('satellites'),
                   gps_node.getFloat('data_age')))
        else:
            print('[gps  ]: age =', gps_node.getFloat('data_age'))

        filter_status = filter_node.getString("navigation")
        print("[filt ]:[%s] " % filter_status, end='')
        print("lon = %.6f lat = %.6f alt = %.1f[m]" % \
              (filter_node.getFloat("longitude_deg"),
               filter_node.getFloat("latitude_deg"),
               filter_node.getFloat("altitude_m"), end = ''))
        print("phi = %4.1f the = %4.1f psi = %5.1f [deg]" % \
              (orient_node.getFloat("roll_deg"),
               orient_node.getFloat("pitch_deg"),
               orient_node.getFloat("heading_deg")))

        print('[act  ]:%.2f %.2f %.2f %.2f %.2f' % \
              (act_node.getFloat('aileron'),
               act_node.getFloat('elevator'),
               act_node.getFloat('throttle'),
               act_node.getFloat('rudder'),
               act_node.getFloat('flaps')))
        print('[hlth ]:cmdseq = %ld  tgtwp = %ld  loadavg = %.2f  vcc = %.2f' % \
              (remote_link_node.getInt('sequence_num'),
               route_node.getInt('target_waypoint_idx'),
               status_node.getFloat('system_load_avg'),
               power_node.getFloat('avionics_vcc')))
        print()
