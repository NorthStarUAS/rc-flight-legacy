/***************************************************************************
 * Serial communication interface (RS232) in Stargate
 * header file for serial.c
 * Jung Soon Jang, May 2005
 ***************************************************************************/

#ifndef _UGEAR_SERIAL_H
#define _UGEAR_SERIAL_H


#include <termios.h>


/* Serial port information for talking to components */
#define SERIAL_PORT0 "/dev/tts/0" 	
#define SERIAL_PORT1 "/dev/tts/1" 	
#define SERIAL_PORT2 "/dev/tts/2" 	
#define SERIAL_PORT3 "/dev/tts/3"
#define SERIAL_PORT4 "/dev/tts/4"
#define sPORT_ST2    "/dev/ttyS0"

#define SERIAL_PORT2 "/dev/ttyS2"

/* Baud rate */
#define	BAUDRATE_9600	B9600
#define BAUDRATE_19200	B19200
#define BAUDRATE_38400	B38400
#define BAUDRATE_57600  B57600
#define BAUDRATE_76800	B76800
#define	BAUDRATE_115200	B115200

/* Serial data structure */
#define s8N1	0		/* 8 data bits,   no parity, 1 stop bit */
#define s8E1	1		/* 8 data bits, even parity, 1 stop bit */	
#define	s8O1	2		/* 8 data bits,  odd parity, 1 stop bit */

/* function prototypes */
int  open_serial(char* serial_port, int baudrate, bool raw_mode);


#endif // _UGEAR_SERIAL_H
