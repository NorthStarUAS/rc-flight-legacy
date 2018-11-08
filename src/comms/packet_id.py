# shared packet id definitions

# note: this id is encoded as a single byte in the binary packet
# format so the max id number we can assign is 255.  As long as we
# have numbers available, it is best to add new packet id's instead of
# changing or reusing numbers to maximize backwards compatibility with
# older binary log files.

GPS_PACKET_V2 = 16
GPS_PACKET_V3 = 26
GPS_PACKET_V4 = 34

IMU_PACKET_V3 = 17
IMU_PACKET_V4 = 35

FILTER_PACKET_V2 = 22
FILTER_PACKET_V3 = 31
FILTER_PACKET_V4 = 36

ACTUATOR_PACKET_V2 = 21
ACTUATOR_PACKET_V3 = 37

PILOT_INPUT_PACKET_V2 = 20
PILOT_INPUT_PACKET_V3 = 38

AP_STATUS_PACKET_V4 = 30
AP_STATUS_PACKET_V5 = 32
AP_STATUS_PACKET_V6 = 33
AP_STATUS_PACKET_V7 = 39

AIRDATA_PACKET_V5 = 18
AIRDATA_PACKET_V6 = 40

SYSTEM_HEALTH_PACKET_V4 = 19
SYSTEM_HEALTH_PACKET_V5 = 41

PAYLOAD_PACKET_V2 = 23
PAYLOAD_PACKET_V3 = 42          # last id assigned

EVENT_PACKET_V1 = 27

COMMAND_PACKET_V1 = 28

REMOTE_JOYSTICK_V1 = 29
