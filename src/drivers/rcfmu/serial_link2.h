#pragma once

#include <stdint.h>             // uint8_t, et. al.

class SerialLink2 {

private:

    // port
    int fd = -1;

    // parser
    int state = 0;
    int counter = 0;
    uint8_t cksum_lo = 0, cksum_hi = 0;

    static const uint8_t START_OF_MSG0 = 147;
    static const uint8_t START_OF_MSG1 = 224;

    int encode_baud( int baud );
    void checksum( uint8_t id, uint8_t len_lo, uint8_t len_hi,
                   uint8_t *buf, uint16_t buf_size,
                   uint8_t *cksum0, uint8_t *cksum1 );

public:

    uint8_t pkt_id = 0;
    uint8_t pkt_len_lo = 0;
    uint8_t pkt_len_hi = 0;
    uint16_t pkt_len = 0;
    uint8_t *payload = nullptr;
    uint16_t payload_len = 0;

    uint32_t parse_errors = 0;

    SerialLink2();
    ~SerialLink2();

    bool open( int baud, const char *device_name );
    bool update();
    int bytes_available();
    bool write_packet(uint8_t packet_id, uint8_t *payload, uint16_t payload_size);
    bool close();
    bool is_open() { return fd >= 0; }
};
