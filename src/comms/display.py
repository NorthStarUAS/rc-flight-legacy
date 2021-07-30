import math

from PropertyTree import PropertyNode

r2d = 180.0 / math.pi

# initialize property nodes
comms_node = PropertyNode("/comms")
imu_node = PropertyNode("/sensors/imu/0")
gps_node = PropertyNode("/sensors/gps/0")
velocity_node = PropertyNode("/velocity")
filter_node = PropertyNode("/filters/filter/0")
orient_node = PropertyNode("/orientation")
pos_pressure_node = PropertyNode("/position/pressure")
act_node = PropertyNode("/actuators")
remote_link_node = PropertyNode("/comms/remote_link")
route_node = PropertyNode("/task/route")
status_node = PropertyNode("/status")
power_node = PropertyNode("/sensors/power")

# make the C++ interface happy
def init():
    pass

# show a display message if display is enabled
def show(message):
    if comms_node.getBool("display_on"):
        print(message)
        
# periodic console summary of attitude/location estimate
def status_summary():
    print("[imu  ]:gyro = %.3f %.3f %.3f [deg/s]" % \
          (imu_node.getDouble("p_rps") * r2d,
           imu_node.getDouble("q_rps") * r2d,
           imu_node.getDouble("r_rps") * r2d),
          "accel = %.3f %.3f %.3f [m/s^2]" % \
          (imu_node.getDouble("ax_mps2"),
           imu_node.getDouble("ay_mps2"),
           imu_node.getDouble("az_mps2")))
    print("[mag  ]:%.3f %.3f %.3f" % \
          (imu_node.getDouble("hx"),
           imu_node.getDouble("hy"),
           imu_node.getDouble("hz")))
    print("[air  ]:Palt = %5.2f[m] Pspd = %4.1f[kt]" % \
          (pos_pressure_node.getDouble("altitude_m"),
           velocity_node.getDouble("airspeed_kt")))

    if gps_node.getDouble("data_age") < 10.0:
        print("[gps  ]:date = %04d/%02d/%02d %02d:%02d:%02d" % \
              (gps_node.getInt("year"),
               gps_node.getInt("month"),
               gps_node.getInt("day"),
               gps_node.getInt("hour"),
               gps_node.getInt("min"),
               gps_node.getInt("sec")))
        print("[gps  ]: pos = %.6f %.6f %.1fm sats = %ld, age = %.2f" % \
              (gps_node.getDouble("longitude_deg"),
               gps_node.getDouble("latitude_deg"),
               gps_node.getDouble("altitude_m"),
               gps_node.getInt("satellites"),
               gps_node.getDouble("data_age")))
    else:
        print("[gps  ]: age =", gps_node.getDouble("data_age"))

    filter_status = status_node.getString("navigation")
    print("[filt ]:[%s] " % filter_status, end="")
    print("pos = %.6f %.6f %.1f " % \
          (filter_node.getDouble("longitude_deg"),
           filter_node.getDouble("latitude_deg"),
           filter_node.getDouble("altitude_m")), end = "")
    print("RPY = %4.1f %4.1f %5.1f (deg)" % \
          (orient_node.getDouble("roll_deg"),
           orient_node.getDouble("pitch_deg"),
           orient_node.getDouble("heading_deg")))

    print("[act  ]:%.2f %.2f %.2f %.2f %.2f" % \
          (act_node.getDouble("aileron"),
           act_node.getDouble("elevator"),
           act_node.getDouble("throttle"),
           act_node.getDouble("rudder"),
           act_node.getDouble("flaps")))
    print("[hlth ]:cmdseq = %ld  tgtwp = %ld  loadavg = %.2f  vcc = %.2f" % \
          (remote_link_node.getInt("sequence_num"),
           route_node.getInt("target_waypoint_idx"),
           status_node.getDouble("system_load_avg"),
           power_node.getDouble("avionics_vcc")))
    print()
