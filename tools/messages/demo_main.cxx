#include <stdio.h>

#include "messages.h"

int main() {
    // create a test message
    message_simple_test_t st;
    st.a = 1234;
    uint8_t *msg = st.pack();
    printf("packed length = %d %d\n", st.len, (int)sizeof(st));
    message_simple_test_t st_recv;
    st_recv.unpack(msg);
    printf("result = %d\n", st_recv.a);
    printf("\n");

    // create a gps message
    message_gps_v4_t gps;
    gps.latitude_deg = 43.241;
    gps.longitude_deg = -93.520;
    gps.altitude_m = 278.5;
    gps.vn_ms = 1.5;
    gps.ve_ms = -2.7;
    gps.vd_ms = -0.02;
    gps.satellites = 9;

    // pack it
    msg = gps.pack();
    printf("msg id = %d, sizeof = %d  packed length = %d\n", gps.id, sizeof(gps), gps.len);

    // pretend the serialized message got sent somewhere and now we
    // received it and deserialized it on the other side
    message_gps_v4_t gps_recv;
    gps_recv.unpack(msg);

    // let's see what we got
    printf("unpack lat: %f\n", gps_recv.latitude_deg);
    printf("unpack lon: %f\n", gps_recv.longitude_deg);
    printf("unpack alt: %f\n", gps_recv.altitude_m);
    printf("unpack vn: %f\n", gps_recv.vn_ms);
    printf("unpack ve: %f\n", gps_recv.ve_ms);
    printf("unpack vd: %f\n", gps_recv.vd_ms);
    printf("unpack vd: %d\n", gps_recv.satellites);

    message_array_test_t at;
    printf("array test size: %d %d\n", at.len, sizeof(at));
    for (int i = 0; i < 9; i++ ) {
        at.orientation[i] = i * 10.0;
    }
    msg = at.pack();
    message_array_test_t at_recv;
    at_recv.unpack(msg);
    for (int i = 0; i < 9; i++) {
        printf("orientation[%d] = %.2f\n", i, at_recv.orientation[i]);
    }
}
