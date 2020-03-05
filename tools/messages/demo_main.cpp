#include <stdio.h>

#include "messages.h"

int main() {
    // create a test message
    message::simple_test_t st;
    st.a = 1234;
    st.pack();
    printf("packed length = %d %d\n", st.len, (int)sizeof(st));
    message::simple_test_t st_recv;
    st_recv.unpack(st.payload, st.len);
    printf("result = %d\n", st_recv.a);
    printf("\n");

    // create a gps message
    message::gps_v4_t gps;
    gps.latitude_deg = 43.241;
    gps.longitude_deg = -93.520;
    gps.altitude_m = 278.5;
    gps.vn_ms = 1.5;
    gps.ve_ms = -2.7;
    gps.vd_ms = -0.02;
    gps.satellites = 9;

    // pack it
    if ( !gps.pack() ) {
        printf("gps pack failed\n");
    } else {
        printf("msg id = %d, sizeof = %d  packed length = %d\n", gps.id, sizeof(gps), gps.len);

        // pretend the serialized message got sent somewhere and now we
        // received it and deserialized it on the other side
        message::gps_v4_t gps_recv;
        gps_recv.unpack(gps.payload, gps.len);

        // let's see what we got
        printf("unpack lat: %f\n", gps_recv.latitude_deg);
        printf("unpack lon: %f\n", gps_recv.longitude_deg);
        printf("unpack alt: %f\n", gps_recv.altitude_m);
        printf("unpack vn: %f\n", gps_recv.vn_ms);
        printf("unpack ve: %f\n", gps_recv.ve_ms);
        printf("unpack vd: %f\n", gps_recv.vd_ms);
        printf("unpack vd: %d\n", gps_recv.satellites);
    }
    
    message::array_test_t at;
    printf("array test size: %d %d\n", at.len, sizeof(at));
    for (int i = 0; i < 9; i++ ) {
        at.orientation[i] = i * 10.0;
    }
    at.pack();
    message::array_test_t at_recv;
    at_recv.unpack(at.payload, at.len);
    for (int i = 0; i < 9; i++) {
        printf("orientation[%d] = %.2f\n", i, at_recv.orientation[i]);
    }

    // variable length string test
    message::dynamic_string_test_t vs;
    vs.time = 42.987654321;
    vs.event = "hello, this is a test                                                                          dsd                             dsd                                                                                                     ";
    vs.counter = 4567;
    vs.args[0] = "a1";
    vs.args[1] = "b2";
    vs.args[2] = "c3";
    vs.args[3] = "d4";
    if ( !vs.pack() ) {
        printf("variable length pack failed (too big)\n");
    } else {
        printf("len(vs): %d\n", vs.len);
        message::dynamic_string_test_t vs_recv;
        vs_recv.unpack(vs.payload, vs.len);
        printf("vs_recv:\n");
        printf("  time: %f\n", vs_recv.time);
        printf("  event: %s\n", vs_recv.event.c_str());
        printf("  counter: %d\n", vs_recv.counter);
        printf("  args[0]: %s\n", vs_recv.args[0].c_str());
        printf("  args[1]: %s\n", vs_recv.args[1].c_str());
        printf("  args[2]: %s\n", vs_recv.args[2].c_str());
        printf("  args[3]: %s\n", vs_recv.args[3].c_str());
    }
}
