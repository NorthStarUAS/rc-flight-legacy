#ifndef _UGEAR_CHECKSUM_H
#define _UGEAR_CHECKSUM_H


#include <stdint.h>

void ugear_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size,
                  uint8_t *cksum0, uint8_t *cksum1 );


#endif // _UGEAR_CHECKSUM_H
