#ifndef _UGEAR_CONSOLE_LINK_H
#define _UGEAR_CONSOLE_LINK_H


#define START_OF_MSG0 147
#define START_OF_MSG1 224

enum ugPacketType {
    GPS_PACKET = 0,
    IMU_PACKET = 1,
    NAV_PACKET = 2,
    SERVO_PACKET = 3
};

extern bool console_link_on;


#endif // _UGEAR_CONSOLE_LINK_H
