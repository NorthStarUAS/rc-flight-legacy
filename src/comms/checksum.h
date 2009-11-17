#ifndef _UGEAR_CHECKSUM_H
#define _UGEAR_CHECKSUM_H


#include <stdint.h>

void ugear_cksum( const uint8_t hdr1, const uint8_t hdr2,
		  const uint8_t *buf, const uint8_t size,
                  uint8_t *cksum0, uint8_t *cksum1 );


#endif // _UGEAR_CHECKSUM_H
