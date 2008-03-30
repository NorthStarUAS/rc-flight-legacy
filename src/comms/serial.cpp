/******************************************************************************
 * FILE: serial.c
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 5/11/05 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "serial.h"

/***************************************************************************
 * Open and configure serial port
 ***************************************************************************/
int open_serial(char* serial_port, int baudrate, bool raw_mode, bool nonblock )
{
    int fd;
    struct termios tio_serial;

    /* open serial port */
    fd = open(serial_port, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        fprintf(stderr,"open serial: unable to open %s - %s\n",
                serial_port,strerror(errno));
        _exit(-1);
    }

    if ( nonblock ) {
        /* Enable non-blocking IO */
        fcntl(fd, F_SETFL, O_NONBLOCK);
    }
      
    /* Serial port setting */
    bzero(&tio_serial, sizeof(tio_serial));
    tio_serial.c_cflag = CS8 | CLOCAL | CREAD;
    tio_serial.c_iflag = IGNBRK | IGNPAR;
    tio_serial.c_oflag = 0;
    //tio_serial.c_lflag = 0;

    if ( raw_mode ) {
      tio_serial.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    }
   
    /***************************************************************************
     * smode = 0, read will be satisfied immediately. The number of characters
     * currently available, or the number of characters requested will be
     * returned. smode > 0, smode sets the number of characters to receive
     *  before the read is satisfied.
     **************************************************************************/
    tio_serial.c_cc[VMIN] = 1;

    cfsetispeed(&tio_serial, baudrate);
    cfsetospeed(&tio_serial, baudrate);

    /* Flush buffer; parameters take effect immediately */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &tio_serial);

    return fd;
}
