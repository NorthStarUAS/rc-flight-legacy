#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

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
    int count = 5;

    if ( argc > 1 ) {
	int tmp = atoi(argv[1]);
	if ( tmp > 0 ) {
	    count = tmp;
	}
    }

    printf("Press any key ... ");
    fflush(stdout);

    while ( count >= 0 ) {
	printf("%d ", count);
	fflush(stdout);

	//int result = mygetch();
	if ( kbhit() ) {
	    return 1;
	}
	//sleep(1);
	count--;
    }

    return 0;
}
