import struct

# Message id constants
command_ack_id = 10
config_airdata_id = 11
config_board_id = 12
config_ekf_id = 13
config_imu_id = 14
config_mixer_id = 15
config_mixer_matrix_id = 16
config_power_id = 17
config_pwm_id = 18
config_stability_damping_id = 19
command_inceptors_id = 20
command_zero_gyros_id = 21
command_reset_ekf_id = 22
command_cycle_inceptors_id = 23
pilot_id = 24
imu_id = 25
aura_nav_pvt_id = 26
airdata_id = 27
power_id = 28
status_id = 29
ekf_id = 30

# Constants
pwm_channels = 8  # number of pwm output channels
sbus_channels = 16  # number of sbus channels
ap_channels = 6  # number of sbus channels
mix_matrix_size = 64  # 8 x 8 mix matrix

# Enums
enum_nav_none = 0  # None
enum_nav_nav15 = 1  # None
enum_nav_nav15_mag = 2  # None

# Message: command_ack
# Id: 10
class command_ack():
    id = 10
    _pack_string = "<BB"

    def __init__(self, msg=None):
        # public fields
        self.command_id = 0
        self.subcommand_id = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.command_id,
                          self.subcommand_id)
        return msg

    def unpack(self, msg):
        (self.command_id,
         self.subcommand_id) = struct.unpack(self._pack_string, msg)

# Message: config_airdata
# Id: 11
class config_airdata():
    id = 11
    _pack_string = "<BBBB"

    def __init__(self, msg=None):
        # public fields
        self.barometer = 0
        self.pitot = 0
        self.swift_baro_addr = 0
        self.swift_pitot_addr = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.barometer,
                          self.pitot,
                          self.swift_baro_addr,
                          self.swift_pitot_addr)
        return msg

    def unpack(self, msg):
        (self.barometer,
         self.pitot,
         self.swift_baro_addr,
         self.swift_pitot_addr) = struct.unpack(self._pack_string, msg)

# Message: config_board
# Id: 12
class config_board():
    id = 12
    _pack_string = "<BB"

    def __init__(self, msg=None):
        # public fields
        self.board = 0
        self.led_pin = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.board,
                          self.led_pin)
        return msg

    def unpack(self, msg):
        (self.board,
         self.led_pin) = struct.unpack(self._pack_string, msg)

# Message: config_ekf
# Id: 13
class config_ekf():
    id = 13
    _pack_string = "<Bfffffffffff"

    def __init__(self, msg=None):
        # public fields
        self.select = 0
        self.sig_w_accel = 0.0
        self.sig_w_gyro = 0.0
        self.sig_a_d = 0.0
        self.tau_a = 0.0
        self.sig_g_d = 0.0
        self.tau_g = 0.0
        self.sig_gps_p_ne = 0.0
        self.sig_gps_p_d = 0.0
        self.sig_gps_v_ne = 0.0
        self.sig_gps_v_d = 0.0
        self.sig_mag = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.select,
                          self.sig_w_accel,
                          self.sig_w_gyro,
                          self.sig_a_d,
                          self.tau_a,
                          self.sig_g_d,
                          self.tau_g,
                          self.sig_gps_p_ne,
                          self.sig_gps_p_d,
                          self.sig_gps_v_ne,
                          self.sig_gps_v_d,
                          self.sig_mag)
        return msg

    def unpack(self, msg):
        (self.select,
         self.sig_w_accel,
         self.sig_w_gyro,
         self.sig_a_d,
         self.tau_a,
         self.sig_g_d,
         self.tau_g,
         self.sig_gps_p_ne,
         self.sig_gps_p_d,
         self.sig_gps_v_ne,
         self.sig_gps_v_d,
         self.sig_mag) = struct.unpack(self._pack_string, msg)

# Message: config_imu
# Id: 14
class config_imu():
    id = 14
    _pack_string = "<BBfffffffffffffffffffffffffffffff"

    def __init__(self, msg=None):
        # public fields
        self.interface = 0
        self.pin_or_address = 0
        self.strapdown_calib = [0.0] * 9
        self.accel_scale = [0.0] * 3
        self.accel_translate = [0.0] * 3
        self.mag_affine = [0.0] * 16
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.interface,
                          self.pin_or_address,
                          self.strapdown_calib[0],
                          self.strapdown_calib[1],
                          self.strapdown_calib[2],
                          self.strapdown_calib[3],
                          self.strapdown_calib[4],
                          self.strapdown_calib[5],
                          self.strapdown_calib[6],
                          self.strapdown_calib[7],
                          self.strapdown_calib[8],
                          self.accel_scale[0],
                          self.accel_scale[1],
                          self.accel_scale[2],
                          self.accel_translate[0],
                          self.accel_translate[1],
                          self.accel_translate[2],
                          self.mag_affine[0],
                          self.mag_affine[1],
                          self.mag_affine[2],
                          self.mag_affine[3],
                          self.mag_affine[4],
                          self.mag_affine[5],
                          self.mag_affine[6],
                          self.mag_affine[7],
                          self.mag_affine[8],
                          self.mag_affine[9],
                          self.mag_affine[10],
                          self.mag_affine[11],
                          self.mag_affine[12],
                          self.mag_affine[13],
                          self.mag_affine[14],
                          self.mag_affine[15])
        return msg

    def unpack(self, msg):
        (self.interface,
         self.pin_or_address,
         self.strapdown_calib[0],
         self.strapdown_calib[1],
         self.strapdown_calib[2],
         self.strapdown_calib[3],
         self.strapdown_calib[4],
         self.strapdown_calib[5],
         self.strapdown_calib[6],
         self.strapdown_calib[7],
         self.strapdown_calib[8],
         self.accel_scale[0],
         self.accel_scale[1],
         self.accel_scale[2],
         self.accel_translate[0],
         self.accel_translate[1],
         self.accel_translate[2],
         self.mag_affine[0],
         self.mag_affine[1],
         self.mag_affine[2],
         self.mag_affine[3],
         self.mag_affine[4],
         self.mag_affine[5],
         self.mag_affine[6],
         self.mag_affine[7],
         self.mag_affine[8],
         self.mag_affine[9],
         self.mag_affine[10],
         self.mag_affine[11],
         self.mag_affine[12],
         self.mag_affine[13],
         self.mag_affine[14],
         self.mag_affine[15]) = struct.unpack(self._pack_string, msg)

# Message: config_mixer
# Id: 15
class config_mixer():
    id = 15
    _pack_string = "<BBBBBBBfffffffffff"

    def __init__(self, msg=None):
        # public fields
        self.mix_autocoord = False
        self.mix_throttle_trim = False
        self.mix_flap_trim = False
        self.mix_elevon = False
        self.mix_flaperon = False
        self.mix_vtail = False
        self.mix_diff_thrust = False
        self.mix_Gac = 0.0
        self.mix_Get = 0.0
        self.mix_Gef = 0.0
        self.mix_Gea = 0.0
        self.mix_Gee = 0.0
        self.mix_Gfa = 0.0
        self.mix_Gff = 0.0
        self.mix_Gve = 0.0
        self.mix_Gvr = 0.0
        self.mix_Gtt = 0.0
        self.mix_Gtr = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.mix_autocoord,
                          self.mix_throttle_trim,
                          self.mix_flap_trim,
                          self.mix_elevon,
                          self.mix_flaperon,
                          self.mix_vtail,
                          self.mix_diff_thrust,
                          self.mix_Gac,
                          self.mix_Get,
                          self.mix_Gef,
                          self.mix_Gea,
                          self.mix_Gee,
                          self.mix_Gfa,
                          self.mix_Gff,
                          self.mix_Gve,
                          self.mix_Gvr,
                          self.mix_Gtt,
                          self.mix_Gtr)
        return msg

    def unpack(self, msg):
        (self.mix_autocoord,
         self.mix_throttle_trim,
         self.mix_flap_trim,
         self.mix_elevon,
         self.mix_flaperon,
         self.mix_vtail,
         self.mix_diff_thrust,
         self.mix_Gac,
         self.mix_Get,
         self.mix_Gef,
         self.mix_Gea,
         self.mix_Gee,
         self.mix_Gfa,
         self.mix_Gff,
         self.mix_Gve,
         self.mix_Gvr,
         self.mix_Gtt,
         self.mix_Gtr) = struct.unpack(self._pack_string, msg)

# Message: config_mixer_matrix
# Id: 16
class config_mixer_matrix():
    id = 16
    _pack_string = "<hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh"

    def __init__(self, msg=None):
        # public fields
        self.matrix = [0.0] * mix_matrix_size
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          int(round(self.matrix[0] * 16384)),
                          int(round(self.matrix[1] * 16384)),
                          int(round(self.matrix[2] * 16384)),
                          int(round(self.matrix[3] * 16384)),
                          int(round(self.matrix[4] * 16384)),
                          int(round(self.matrix[5] * 16384)),
                          int(round(self.matrix[6] * 16384)),
                          int(round(self.matrix[7] * 16384)),
                          int(round(self.matrix[8] * 16384)),
                          int(round(self.matrix[9] * 16384)),
                          int(round(self.matrix[10] * 16384)),
                          int(round(self.matrix[11] * 16384)),
                          int(round(self.matrix[12] * 16384)),
                          int(round(self.matrix[13] * 16384)),
                          int(round(self.matrix[14] * 16384)),
                          int(round(self.matrix[15] * 16384)),
                          int(round(self.matrix[16] * 16384)),
                          int(round(self.matrix[17] * 16384)),
                          int(round(self.matrix[18] * 16384)),
                          int(round(self.matrix[19] * 16384)),
                          int(round(self.matrix[20] * 16384)),
                          int(round(self.matrix[21] * 16384)),
                          int(round(self.matrix[22] * 16384)),
                          int(round(self.matrix[23] * 16384)),
                          int(round(self.matrix[24] * 16384)),
                          int(round(self.matrix[25] * 16384)),
                          int(round(self.matrix[26] * 16384)),
                          int(round(self.matrix[27] * 16384)),
                          int(round(self.matrix[28] * 16384)),
                          int(round(self.matrix[29] * 16384)),
                          int(round(self.matrix[30] * 16384)),
                          int(round(self.matrix[31] * 16384)),
                          int(round(self.matrix[32] * 16384)),
                          int(round(self.matrix[33] * 16384)),
                          int(round(self.matrix[34] * 16384)),
                          int(round(self.matrix[35] * 16384)),
                          int(round(self.matrix[36] * 16384)),
                          int(round(self.matrix[37] * 16384)),
                          int(round(self.matrix[38] * 16384)),
                          int(round(self.matrix[39] * 16384)),
                          int(round(self.matrix[40] * 16384)),
                          int(round(self.matrix[41] * 16384)),
                          int(round(self.matrix[42] * 16384)),
                          int(round(self.matrix[43] * 16384)),
                          int(round(self.matrix[44] * 16384)),
                          int(round(self.matrix[45] * 16384)),
                          int(round(self.matrix[46] * 16384)),
                          int(round(self.matrix[47] * 16384)),
                          int(round(self.matrix[48] * 16384)),
                          int(round(self.matrix[49] * 16384)),
                          int(round(self.matrix[50] * 16384)),
                          int(round(self.matrix[51] * 16384)),
                          int(round(self.matrix[52] * 16384)),
                          int(round(self.matrix[53] * 16384)),
                          int(round(self.matrix[54] * 16384)),
                          int(round(self.matrix[55] * 16384)),
                          int(round(self.matrix[56] * 16384)),
                          int(round(self.matrix[57] * 16384)),
                          int(round(self.matrix[58] * 16384)),
                          int(round(self.matrix[59] * 16384)),
                          int(round(self.matrix[60] * 16384)),
                          int(round(self.matrix[61] * 16384)),
                          int(round(self.matrix[62] * 16384)),
                          int(round(self.matrix[63] * 16384)))
        return msg

    def unpack(self, msg):
        (self.matrix[0],
         self.matrix[1],
         self.matrix[2],
         self.matrix[3],
         self.matrix[4],
         self.matrix[5],
         self.matrix[6],
         self.matrix[7],
         self.matrix[8],
         self.matrix[9],
         self.matrix[10],
         self.matrix[11],
         self.matrix[12],
         self.matrix[13],
         self.matrix[14],
         self.matrix[15],
         self.matrix[16],
         self.matrix[17],
         self.matrix[18],
         self.matrix[19],
         self.matrix[20],
         self.matrix[21],
         self.matrix[22],
         self.matrix[23],
         self.matrix[24],
         self.matrix[25],
         self.matrix[26],
         self.matrix[27],
         self.matrix[28],
         self.matrix[29],
         self.matrix[30],
         self.matrix[31],
         self.matrix[32],
         self.matrix[33],
         self.matrix[34],
         self.matrix[35],
         self.matrix[36],
         self.matrix[37],
         self.matrix[38],
         self.matrix[39],
         self.matrix[40],
         self.matrix[41],
         self.matrix[42],
         self.matrix[43],
         self.matrix[44],
         self.matrix[45],
         self.matrix[46],
         self.matrix[47],
         self.matrix[48],
         self.matrix[49],
         self.matrix[50],
         self.matrix[51],
         self.matrix[52],
         self.matrix[53],
         self.matrix[54],
         self.matrix[55],
         self.matrix[56],
         self.matrix[57],
         self.matrix[58],
         self.matrix[59],
         self.matrix[60],
         self.matrix[61],
         self.matrix[62],
         self.matrix[63],) = struct.unpack(self._pack_string, msg)
        self.matrix[0] /= 16384
        self.matrix[1] /= 16384
        self.matrix[2] /= 16384
        self.matrix[3] /= 16384
        self.matrix[4] /= 16384
        self.matrix[5] /= 16384
        self.matrix[6] /= 16384
        self.matrix[7] /= 16384
        self.matrix[8] /= 16384
        self.matrix[9] /= 16384
        self.matrix[10] /= 16384
        self.matrix[11] /= 16384
        self.matrix[12] /= 16384
        self.matrix[13] /= 16384
        self.matrix[14] /= 16384
        self.matrix[15] /= 16384
        self.matrix[16] /= 16384
        self.matrix[17] /= 16384
        self.matrix[18] /= 16384
        self.matrix[19] /= 16384
        self.matrix[20] /= 16384
        self.matrix[21] /= 16384
        self.matrix[22] /= 16384
        self.matrix[23] /= 16384
        self.matrix[24] /= 16384
        self.matrix[25] /= 16384
        self.matrix[26] /= 16384
        self.matrix[27] /= 16384
        self.matrix[28] /= 16384
        self.matrix[29] /= 16384
        self.matrix[30] /= 16384
        self.matrix[31] /= 16384
        self.matrix[32] /= 16384
        self.matrix[33] /= 16384
        self.matrix[34] /= 16384
        self.matrix[35] /= 16384
        self.matrix[36] /= 16384
        self.matrix[37] /= 16384
        self.matrix[38] /= 16384
        self.matrix[39] /= 16384
        self.matrix[40] /= 16384
        self.matrix[41] /= 16384
        self.matrix[42] /= 16384
        self.matrix[43] /= 16384
        self.matrix[44] /= 16384
        self.matrix[45] /= 16384
        self.matrix[46] /= 16384
        self.matrix[47] /= 16384
        self.matrix[48] /= 16384
        self.matrix[49] /= 16384
        self.matrix[50] /= 16384
        self.matrix[51] /= 16384
        self.matrix[52] /= 16384
        self.matrix[53] /= 16384
        self.matrix[54] /= 16384
        self.matrix[55] /= 16384
        self.matrix[56] /= 16384
        self.matrix[57] /= 16384
        self.matrix[58] /= 16384
        self.matrix[59] /= 16384
        self.matrix[60] /= 16384
        self.matrix[61] /= 16384
        self.matrix[62] /= 16384
        self.matrix[63] /= 16384

# Message: config_power
# Id: 17
class config_power():
    id = 17
    _pack_string = "<B"

    def __init__(self, msg=None):
        # public fields
        self.have_attopilot = False
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.have_attopilot)
        return msg

    def unpack(self, msg):
        (self.have_attopilot,) = struct.unpack(self._pack_string, msg)

# Message: config_pwm
# Id: 18
class config_pwm():
    id = 18
    _pack_string = "<Hffffffff"

    def __init__(self, msg=None):
        # public fields
        self.pwm_hz = 0
        self.act_gain = [0.0] * pwm_channels
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.pwm_hz,
                          self.act_gain[0],
                          self.act_gain[1],
                          self.act_gain[2],
                          self.act_gain[3],
                          self.act_gain[4],
                          self.act_gain[5],
                          self.act_gain[6],
                          self.act_gain[7])
        return msg

    def unpack(self, msg):
        (self.pwm_hz,
         self.act_gain[0],
         self.act_gain[1],
         self.act_gain[2],
         self.act_gain[3],
         self.act_gain[4],
         self.act_gain[5],
         self.act_gain[6],
         self.act_gain[7]) = struct.unpack(self._pack_string, msg)

# Message: config_stability_damping
# Id: 19
class config_stability_damping():
    id = 19
    _pack_string = "<BBBBffff"

    def __init__(self, msg=None):
        # public fields
        self.sas_rollaxis = False
        self.sas_pitchaxis = False
        self.sas_yawaxis = False
        self.sas_tune = False
        self.sas_rollgain = 0.0
        self.sas_pitchgain = 0.0
        self.sas_yawgain = 0.0
        self.sas_max_gain = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.sas_rollaxis,
                          self.sas_pitchaxis,
                          self.sas_yawaxis,
                          self.sas_tune,
                          self.sas_rollgain,
                          self.sas_pitchgain,
                          self.sas_yawgain,
                          self.sas_max_gain)
        return msg

    def unpack(self, msg):
        (self.sas_rollaxis,
         self.sas_pitchaxis,
         self.sas_yawaxis,
         self.sas_tune,
         self.sas_rollgain,
         self.sas_pitchgain,
         self.sas_yawgain,
         self.sas_max_gain) = struct.unpack(self._pack_string, msg)

# Message: command_inceptors
# Id: 20
class command_inceptors():
    id = 20
    _pack_string = "<hhhhhh"

    def __init__(self, msg=None):
        # public fields
        self.channel = [0.0] * ap_channels
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          int(round(self.channel[0] * 16384)),
                          int(round(self.channel[1] * 16384)),
                          int(round(self.channel[2] * 16384)),
                          int(round(self.channel[3] * 16384)),
                          int(round(self.channel[4] * 16384)),
                          int(round(self.channel[5] * 16384)))
        return msg

    def unpack(self, msg):
        (self.channel[0],
         self.channel[1],
         self.channel[2],
         self.channel[3],
         self.channel[4],
         self.channel[5],) = struct.unpack(self._pack_string, msg)
        self.channel[0] /= 16384
        self.channel[1] /= 16384
        self.channel[2] /= 16384
        self.channel[3] /= 16384
        self.channel[4] /= 16384
        self.channel[5] /= 16384

# Message: command_zero_gyros
# Id: 21
class command_zero_gyros():
    id = 21
    _pack_string = "<"

    def __init__(self, msg=None):
        # public fields
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
        return msg

    def unpack(self, msg):

# Message: command_reset_ekf
# Id: 22
class command_reset_ekf():
    id = 22
    _pack_string = "<"

    def __init__(self, msg=None):
        # public fields
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
        return msg

    def unpack(self, msg):

# Message: command_cycle_inceptors
# Id: 23
class command_cycle_inceptors():
    id = 23
    _pack_string = "<"

    def __init__(self, msg=None):
        # public fields
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
        return msg

    def unpack(self, msg):

# Message: pilot
# Id: 24
class pilot():
    id = 24
    _pack_string = "<hhhhhhhhhhhhhhhhB"

    def __init__(self, msg=None):
        # public fields
        self.channel = [0.0] * sbus_channels
        self.flags = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          int(round(self.channel[0] * 16384)),
                          int(round(self.channel[1] * 16384)),
                          int(round(self.channel[2] * 16384)),
                          int(round(self.channel[3] * 16384)),
                          int(round(self.channel[4] * 16384)),
                          int(round(self.channel[5] * 16384)),
                          int(round(self.channel[6] * 16384)),
                          int(round(self.channel[7] * 16384)),
                          int(round(self.channel[8] * 16384)),
                          int(round(self.channel[9] * 16384)),
                          int(round(self.channel[10] * 16384)),
                          int(round(self.channel[11] * 16384)),
                          int(round(self.channel[12] * 16384)),
                          int(round(self.channel[13] * 16384)),
                          int(round(self.channel[14] * 16384)),
                          int(round(self.channel[15] * 16384)),
                          self.flags)
        return msg

    def unpack(self, msg):
        (self.channel[0],
         self.channel[1],
         self.channel[2],
         self.channel[3],
         self.channel[4],
         self.channel[5],
         self.channel[6],
         self.channel[7],
         self.channel[8],
         self.channel[9],
         self.channel[10],
         self.channel[11],
         self.channel[12],
         self.channel[13],
         self.channel[14],
         self.channel[15],
         self.flags) = struct.unpack(self._pack_string, msg)
        self.channel[0] /= 16384
        self.channel[1] /= 16384
        self.channel[2] /= 16384
        self.channel[3] /= 16384
        self.channel[4] /= 16384
        self.channel[5] /= 16384
        self.channel[6] /= 16384
        self.channel[7] /= 16384
        self.channel[8] /= 16384
        self.channel[9] /= 16384
        self.channel[10] /= 16384
        self.channel[11] /= 16384
        self.channel[12] /= 16384
        self.channel[13] /= 16384
        self.channel[14] /= 16384
        self.channel[15] /= 16384

# Message: imu
# Id: 25
class imu():
    id = 25
    _pack_string = "<Lhhhhhhhhhhhhhhhh"

    def __init__(self, msg=None):
        # public fields
        self.millis = 0
        self.raw = [0] * 6
        self.cal = [0] * 10
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.millis,
                          self.raw[0],
                          self.raw[1],
                          self.raw[2],
                          self.raw[3],
                          self.raw[4],
                          self.raw[5],
                          self.cal[0],
                          self.cal[1],
                          self.cal[2],
                          self.cal[3],
                          self.cal[4],
                          self.cal[5],
                          self.cal[6],
                          self.cal[7],
                          self.cal[8],
                          self.cal[9])
        return msg

    def unpack(self, msg):
        (self.millis,
         self.raw[0],
         self.raw[1],
         self.raw[2],
         self.raw[3],
         self.raw[4],
         self.raw[5],
         self.cal[0],
         self.cal[1],
         self.cal[2],
         self.cal[3],
         self.cal[4],
         self.cal[5],
         self.cal[6],
         self.cal[7],
         self.cal[8],
         self.cal[9]) = struct.unpack(self._pack_string, msg)

# Message: aura_nav_pvt
# Id: 26
class aura_nav_pvt():
    id = 26
    _pack_string = "<LhBBBBBBLlBBBBllllLLlllLlLLHBBBBBBlhH"

    def __init__(self, msg=None):
        # public fields
        self.iTOW = 0
        self.year = 0
        self.month = 0
        self.day = 0
        self.hour = 0
        self.min = 0
        self.sec = 0
        self.valid = 0
        self.tAcc = 0
        self.nano = 0
        self.fixType = 0
        self.flags = 0
        self.flags2 = 0
        self.numSV = 0
        self.lon = 0
        self.lat = 0
        self.height = 0
        self.hMSL = 0
        self.hAcc = 0
        self.vAcc = 0
        self.velN = 0
        self.velE = 0
        self.velD = 0
        self.gSpeed = 0
        self.heading = 0
        self.sAcc = 0
        self.headingAcc = 0
        self.pDOP = 0
        self.reserved = [0] * 6
        self.headVeh = 0
        self.magDec = 0
        self.magAcc = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.iTOW,
                          self.year,
                          self.month,
                          self.day,
                          self.hour,
                          self.min,
                          self.sec,
                          self.valid,
                          self.tAcc,
                          self.nano,
                          self.fixType,
                          self.flags,
                          self.flags2,
                          self.numSV,
                          self.lon,
                          self.lat,
                          self.height,
                          self.hMSL,
                          self.hAcc,
                          self.vAcc,
                          self.velN,
                          self.velE,
                          self.velD,
                          self.gSpeed,
                          self.heading,
                          self.sAcc,
                          self.headingAcc,
                          self.pDOP,
                          self.reserved[0],
                          self.reserved[1],
                          self.reserved[2],
                          self.reserved[3],
                          self.reserved[4],
                          self.reserved[5],
                          self.headVeh,
                          self.magDec,
                          self.magAcc)
        return msg

    def unpack(self, msg):
        (self.iTOW,
         self.year,
         self.month,
         self.day,
         self.hour,
         self.min,
         self.sec,
         self.valid,
         self.tAcc,
         self.nano,
         self.fixType,
         self.flags,
         self.flags2,
         self.numSV,
         self.lon,
         self.lat,
         self.height,
         self.hMSL,
         self.hAcc,
         self.vAcc,
         self.velN,
         self.velE,
         self.velD,
         self.gSpeed,
         self.heading,
         self.sAcc,
         self.headingAcc,
         self.pDOP,
         self.reserved[0],
         self.reserved[1],
         self.reserved[2],
         self.reserved[3],
         self.reserved[4],
         self.reserved[5],
         self.headVeh,
         self.magDec,
         self.magAcc) = struct.unpack(self._pack_string, msg)

# Message: airdata
# Id: 27
class airdata():
    id = 27
    _pack_string = "<ffffffH"

    def __init__(self, msg=None):
        # public fields
        self.baro_press_pa = 0.0
        self.baro_temp_C = 0.0
        self.baro_hum = 0.0
        self.ext_diff_press_pa = 0.0
        self.ext_static_press_pa = 0.0
        self.ext_temp_C = 0.0
        self.error_count = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.baro_press_pa,
                          self.baro_temp_C,
                          self.baro_hum,
                          self.ext_diff_press_pa,
                          self.ext_static_press_pa,
                          self.ext_temp_C,
                          self.error_count)
        return msg

    def unpack(self, msg):
        (self.baro_press_pa,
         self.baro_temp_C,
         self.baro_hum,
         self.ext_diff_press_pa,
         self.ext_static_press_pa,
         self.ext_temp_C,
         self.error_count) = struct.unpack(self._pack_string, msg)

# Message: power
# Id: 28
class power():
    id = 28
    _pack_string = "<HHHH"

    def __init__(self, msg=None):
        # public fields
        self.int_main_v = 0.0
        self.avionics_v = 0.0
        self.ext_main_v = 0.0
        self.ext_main_amp = 0.0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          int(round(self.int_main_v * 100)),
                          int(round(self.avionics_v * 100)),
                          int(round(self.ext_main_v * 100)),
                          int(round(self.ext_main_amp * 100)))
        return msg

    def unpack(self, msg):
        (self.int_main_v,
         self.avionics_v,
         self.ext_main_v,
         self.ext_main_amp) = struct.unpack(self._pack_string, msg)
        self.int_main_v /= 100
        self.avionics_v /= 100
        self.ext_main_v /= 100
        self.ext_main_amp /= 100

# Message: status
# Id: 29
class status():
    id = 29
    _pack_string = "<HHHLHH"

    def __init__(self, msg=None):
        # public fields
        self.serial_number = 0
        self.firmware_rev = 0
        self.master_hz = 0
        self.baud = 0
        self.byte_rate = 0
        self.timer_misses = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.serial_number,
                          self.firmware_rev,
                          self.master_hz,
                          self.baud,
                          self.byte_rate,
                          self.timer_misses)
        return msg

    def unpack(self, msg):
        (self.serial_number,
         self.firmware_rev,
         self.master_hz,
         self.baud,
         self.byte_rate,
         self.timer_misses) = struct.unpack(self._pack_string, msg)

# Message: ekf
# Id: 30
class ekf():
    id = 30
    _pack_string = "<LddfffffffffffffHHHB"

    def __init__(self, msg=None):
        # public fields
        self.millis = 0
        self.lat_rad = 0.0
        self.lon_rad = 0.0
        self.altitude_m = 0.0
        self.vn_ms = 0.0
        self.ve_ms = 0.0
        self.vd_ms = 0.0
        self.phi_rad = 0.0
        self.the_rad = 0.0
        self.psi_rad = 0.0
        self.p_bias = 0.0
        self.q_bias = 0.0
        self.r_bias = 0.0
        self.ax_bias = 0.0
        self.ay_bias = 0.0
        self.az_bias = 0.0
        self.max_pos_cov = 0.0
        self.max_vel_cov = 0.0
        self.max_att_cov = 0.0
        self.status = 0
        # unpack if requested
        if msg: self.unpack(msg)

    def pack(self):
        msg = struct.pack(self._pack_string,
                          self.millis,
                          self.lat_rad,
                          self.lon_rad,
                          self.altitude_m,
                          self.vn_ms,
                          self.ve_ms,
                          self.vd_ms,
                          self.phi_rad,
                          self.the_rad,
                          self.psi_rad,
                          self.p_bias,
                          self.q_bias,
                          self.r_bias,
                          self.ax_bias,
                          self.ay_bias,
                          self.az_bias,
                          int(round(self.max_pos_cov * 100)),
                          int(round(self.max_vel_cov * 1000)),
                          int(round(self.max_att_cov * 10000)),
                          self.status)
        return msg

    def unpack(self, msg):
        (self.millis,
         self.lat_rad,
         self.lon_rad,
         self.altitude_m,
         self.vn_ms,
         self.ve_ms,
         self.vd_ms,
         self.phi_rad,
         self.the_rad,
         self.psi_rad,
         self.p_bias,
         self.q_bias,
         self.r_bias,
         self.ax_bias,
         self.ay_bias,
         self.az_bias,
         self.max_pos_cov,
         self.max_vel_cov,
         self.max_att_cov,
         self.status) = struct.unpack(self._pack_string, msg)
        self.max_pos_cov /= 100
        self.max_vel_cov /= 1000
        self.max_att_cov /= 10000

