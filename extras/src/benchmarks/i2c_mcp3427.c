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
#define DEV_ADDR 0x68

static const unsigned char BIT_RDY = 7; //data ready
static const unsigned char BIT_C1 = 6; //channel select
static const unsigned char BIT_C0 = 5; //channel select
static const unsigned char BIT_OC = 4; //conversion mode (one shot/continuous)
static const unsigned char BIT_S1 = 3; //sample rate
static const unsigned char BIT_S0 = 2; //sample rate
static const unsigned char BIT_G1 = 1; //gain
static const unsigned char BIT_G0 = 0; //gain


int fd;


/* issue a general call reset to the device */
void dev_reset() {
    int result;
    unsigned char buf[10];

    buf[0] = 0x00;
    buf[1] = 0x06;
    result = write( fd, buf, 2);
    if ( result != 2 ) {
	/* ERROR HANDLING: i2c transaction failed */
	perror("(reset) Failed to write to the i2c bus");
    }
}


int main() {
    int result;
    int channel;
    int gain;
    double filter;
    int filter_init;
    unsigned char reg;
    double v;
    unsigned char buf[10];
    double psi, pa, alt_m;

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

    dev_reset();

    filter = 0.0;
    filter_init = 0;

    while ( 1 ) {
	for ( gain = 0; gain < 1; gain++ ) {
	    for ( channel = 0; channel < 1; channel++ ) {
		/* compute register command */
		reg = 1 << BIT_RDY |
		    channel << BIT_C0 |
		    1 << BIT_OC |
		    1 << BIT_S1 |
		    gain;

		/* write command register */
		buf[0] = reg;
		result = write( fd, buf, 1);
		if ( result != 1 ) {
		    /* ERROR HANDLING: i2c transaction failed */
		    perror("Failed to write to the i2c bus");
		    dev_reset();
		    sleep(1);
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

		pa = 97482 * v / 0.5031;
		psi = pa / 6894.75729;

		alt_m = 44330.8 - 4946.54 * pow(pa, 0.1902632);

		printf("reg: %0x ch: %d gain: %d  %0X %0X %0X %.4fv (%d) %.0f %.4f\n", reg, channel, gain, buf[0], buf[1], buf[2], v, result, pa, alt_m );
		//printf("ch: %d gain: %d volt = %.4f\n", channel, gain, v);

		usleep(90000);
	    }
	}
    }
}
