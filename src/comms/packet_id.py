# shared packet id definitions

# note: this id is encoded as a single byte in the binary packet
# format so the max id number we can assign is 255.  As long as we
# have numbers available, it is best to add new packet id's instead of
# changing or reusing numbers to maximize backwards compatibility with
# older binary log files.

GPS_PACKET_V1 = 0
GPS_PACKET_V2 = 16
GPS_PACKET_V3 = 26

IMU_PACKET_V1 = 1
IMU_PACKET_V2 = 15
IMU_PACKET_V3 = 17

FILTER_PACKET_V1 = 2
FILTER_PACKET_V2 = 22
FILTER_PACKET_V3 = 31

ACTUATOR_PACKET_V1 = 3
ACTUATOR_PACKET_V2 = 21

PILOT_INPUT_PACKET_V1 = 4
PILOT_INPUT_PACKET_V2 = 20

AP_STATUS_PACKET_V1 = 5
AP_STATUS_PACKET_V2 = 10
AP_STATUS_PACKET_V3 = 24
AP_STATUS_PACKET_V4 = 30
AP_STATUS_PACKET_V5 = 32        # last id assigned

AIRDATA_PACKET_V3 = 9
AIRDATA_PACKET_V4 = 13
AIRDATA_PACKET_V5 = 18

SYSTEM_HEALTH_PACKET_V2 = 11
SYSTEM_HEALTH_PACKET_V3 = 14
SYSTEM_HEALTH_PACKET_V4 = 19

PAYLOAD_PACKET_V1 = 12
PAYLOAD_PACKET_V2 = 23

EVENT_PACKET_V1 = 27

COMMAND_PACKET_V1 = 28

RAVEN_PACKET_V1 = 25
REMOTE_JOYSTICK_V1 = 29
