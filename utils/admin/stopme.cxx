#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "util/timing.h"

const char *magic = "stopme";
int counter = 0;

int kbhit(void)
{
    fd_set rfds;
    struct timeval tv;
    int retval;
    struct termios term, oterm;
    int fd = 0;
    tcgetattr( fd, &oterm );
    memcpy( &term, &oterm, sizeof(term) );
    cfmakeraw(&term);
    tcsetattr( fd, TCSANOW, &term );
    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    /* Wait up to one seconds. */
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    retval = select(1, &rfds, NULL, NULL, &tv);
    /* Don't rely on the value of tv now! */
    tcsetattr( fd, TCSANOW, &oterm );
    return(retval);
}

int mygetch( ) {
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

int main(int argc, char **argv) {
    int timer = 5;

    if ( argc > 1 ) {
	int tmp = atoi(argv[1]);
	if ( tmp > 0 ) {
	    timer = tmp;
	}
    }

    double end_time = get_Time() + timer;
    int magic_len = strlen(magic);

    printf("Type '%s' withing %d secs to quit: ", magic, timer);
    fflush(stdout);

    while ( get_Time() < end_time ) {
	/* printf("%d ", timer); */
	/* fflush(stdout); */

	if ( kbhit() ) {
	    int result = mygetch();
	    if ( result == magic[counter] ) {
		printf("%c", result);
		fflush(stdout);
		counter++;
	    } else {
		counter = 0;
		printf("\nrestart: ");
		fflush(stdout);
	    }
	    if ( counter >= magic_len ) {
		printf("\nInteractive stop request granted.\n");
		return 1;
	    }
	}
    }

    printf("\ncontinuing...\n");

    return 0;
}
