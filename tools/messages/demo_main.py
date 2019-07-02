import messages

# quick message test
st = messages.simple_test()
print('st:', st.__dict__)
st.a = 1234
msg = st.pack()
print("msg len:", len(msg))

st_recv = messages.simple_test(msg)
#st_recv.unpack(msg)
print("received:", st_recv.a)

# gps v4 test
gps = messages.gps_v4()
gps.index = 0
gps.latitude_deg = 43.241
gps.longitude_deg = -93.520
gps.altitude_m = 278.5
gps.vn_ms = 1.5
gps.ve_ms = -2.7
gps.vd_ms = -0.02
gps.satellites = 9
msg = gps.pack()
print("msg len:", len(msg))

# in steps (to reuse the instance)
gps_recv = messages.gps_v4()
gps_recv.unpack(msg)
print(gps_recv.__dict__)

# directly
gps_recv = messages.gps_v4(msg)
print(gps_recv.__dict__)

# array test
at = messages.array_test()
at.time = 14.5
for i in range(4):
    at.flags[i] = 10 - i
for i in range(9):
    at.orientation[i] = 2 * i - 7.0
at.something = 16777
msg = at.pack()
at_recv = messages.array_test()
at_recv.unpack(msg)
print(at_recv.__dict__)

# variable length string test
vs = messages.dynamic_string_test()
vs.time = 42.987654321
vs.event = "hello, this is a test"
vs.counter = 4567
vs.args[0] = "a1"
vs.args[1] = "b2"
vs.args[2] = "c3"
vs.args[3] = "d4"
msg = vs.pack()
print("len(msg):", len(msg))
vs_recv = messages.dynamic_string_test()
vs_recv.unpack(msg)
print("vs_recv:", vs_recv.__dict__)
