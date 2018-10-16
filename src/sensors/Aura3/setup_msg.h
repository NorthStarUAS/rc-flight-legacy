#ifndef AURA_CONFIG_MSG_H_INCLUDED
#define AURA_CONFIG_MSG_H_INCLUDED


const uint8_t START_OF_MSG0 = 147;
const uint8_t START_OF_MSG1 = 224;

const uint8_t CONFIG_ACK_PACKET_ID = 20;
const uint8_t CONFIG_MASTER_PACKET_ID = 21;
const uint8_t CONFIG_IMU_PACKET_ID = 22;
const uint8_t CONFIG_ACTUATORS_PACKET_ID = 23;
const uint8_t CONFIG_LED_PACKET_ID = 24;

const uint8_t WRITE_EEPROM_PACKET_ID = 29;

const uint8_t FLIGHT_COMMAND_PACKET_ID = 30;

const uint8_t PILOT_PACKET_ID = 50;
const uint8_t IMU_PACKET_ID = 51;
const uint8_t GPS_PACKET_ID = 52;
const uint8_t AIRDATA_PACKET_ID = 53;
const uint8_t POWER_PACKET_ID = 54;
const uint8_t STATUS_INFO_PACKET_ID = 55;


#endif // AURA_CONFIG_MSG_H_INCLUDED
