#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <string.h>


void usage() {
    printf("usage: spiread loops\n");
    exit(-1);
}

int main(int argc, char **argv) {
    char buff[256];
    int count = 0;
    int i = 0, j = 0;
    int result;
    uint32_t *iptr;
    float *fptr;

    if ( argc != 2 ) {
	usage();
    }

    count = atoi(argv[1]);

    int fdev = open("/dev/spike", O_RDWR);
    if ( fdev == -1 ) {
	printf("Cannot open device\n");
	exit(-1);
    }

    printf("Starting the spi driver (then sleeping 1 second)...\n");
    sprintf(buff, "start\n");
    write( fdev, buff, strlen(buff) );
    sleep(1);

    for( i = 0; i < count; i++ ) {
	result = read( fdev, buff, 56 );
	if ( result != 56 ) {
	    printf("read error, got %d bytes\n", result);
	} else {
	    /* fwrite( buff, 56, 1, stdout ); */
	    iptr = (uint32_t *)&buff[0]; /* running */
	    printf("%u ", *iptr);
	    iptr = (uint32_t *)&buff[4]; /* call back counter */
	    printf("%u ", *iptr);
	    iptr = (uint32_t *)&buff[8]; /* busy counter */
	    printf("%u ", *iptr);

	    for ( j = 0; j < 10; j++ ) {
		fptr = (float *)&buff[16+4*j];
		printf("%.3f ", *fptr);
	    }
	    printf("\n");
	}
    }

    return 0;
}
