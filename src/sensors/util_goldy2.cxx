// goldy2 utilities

#include <stdint.h>

#include "util_goldy2.hxx"


uint16_t utilCRC16( const void* data_p, uint16_t data_len, uint16_t crc_start )
{
    uint8_t* data_u8_p;
    uint16_t data_idx;
    uint16_t crc;
        
    // Typecast input for processing.  Note: typecase to 8-bit type does not
    // yield data alignment issues.
    data_u8_p = (uint8_t*) data_p;
        
    // Start CRC calculation value as that supplied.
    crc = crc_start;
        
    for( data_idx = 0; data_idx < data_len; data_idx++ ) {
	crc = (uint8_t)(crc >> 8) | (crc << 8);
	crc ^= data_u8_p[ data_idx ];
	crc ^= (uint8_t)(crc & 0xff) >> 4;
	crc ^= crc << 12;
	crc ^= (crc & 0x00ff) << 5;
    } 
        
    return crc;
}
