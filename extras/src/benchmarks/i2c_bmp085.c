#include <stdio.h>		/* sprintf() */
#include <sys/types.h>		/* open() */
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>		/* exit() */
#include <sys/ioctl.h>		/* ioctl() */
#include <linux/i2c-dev.h>
#include <unistd.h>		/* read() / write() */
#include <errno.h>
#include <math.h>		/* pow() */

#define I2C_DEV "/dev/i2c-3"
#define DEV_ADDR 0x77
#define OVERSAMPLING 3		/* 0 - 3, 3 = highest resolution */

int fd;

int main() {
    int result;
    int channel;
    int gain;
    double filter;
    int filter_init;
    double v;
    unsigned char buf[10];
    double psi, kpa, alt_m;

    fd = open( I2C_DEV, O_RDWR );
    if ( fd < 0 ) {
        perror(I2C_DEV);
        exit(1);
    }
    
    result = ioctl( fd, I2C_SLAVE, DEV_ADDR );
    if ( result < 0 ) {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }

    filter = 0.0;
    filter_init = 0;

    while ( 1 ) {
	/* write command register */
	buf[0] = 0xF4;
	buf[1] = 0x34+(OVERSAMPLING << 6); /* command: read pressure */
	result = write( fd, buf, 2);
	if ( result != 2 ) {
	    /* ERROR HANDLING: i2c transaction failed */
	    perror("Failed to write to the i2c bus");
	}
	usleep(4000);

	/* read response */
	result = read( fd, buf, 3);
	if ( result != 3 ) {
	    perror("Failed to read from the i2c bus");
	}

	/* display result */
	result = buf[0] << 8 | buf[1];
  	
	if ( result >= 32768) {
	    result = 65536 - result;
	}
  	
	v = (double)result * 2.048 / 32768.0;

	if ( filter_init ) {
	    filter = 0.999 * filter + 0.001 * v;
	} else {
	    filter = v;
	    filter_init = 1;
	}

	kpa = 97482 * v / 0.5031;
	psi = kpa / 6894.75729;

	alt_m = 44330.8 - 4946.54 * pow(kpa, 0.1902632);

	printf("reg: %0x ch: %d gain: %d  %0X %0X %0X %.4fv (%d) %.4f\n", reg, channel, gain, buf[0], buf[1], buf[2], v, result, alt_m );
	//printf("ch: %d gain: %d volt = %.4f\n", channel, gain, v);

	usleep(90000);
    }
}
