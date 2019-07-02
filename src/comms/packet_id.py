# shared packet id definitions

# note: this id is encoded as a single byte in the binary packet
# format so the max id number we can assign is 255.  As long as we
# have numbers available, it is best to add new packet id's instead of
# changing or reusing numbers to maximize backwards compatibility with
# older binary log files.

REMOTE_JOYSTICK_V1 = 29
