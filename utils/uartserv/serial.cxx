// serial.cxx -- Unix serial I/O support
//
// Written by Curtis Olson, started November 1998.
//
// Copyright (C) 1998  Curtis L. Olson - http://www.flightgear.org/~curt
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id: serial.cxx,v 1.12 2008/07/25 18:35:42 ehofman Exp $


#include <stdio.h>
#include <errno.h>

#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "serial.hxx"


SGSerialPort::SGSerialPort()
    : dev_open(false)
{
    // empty
}

SGSerialPort::SGSerialPort(const string& device, int baud, bool nonblock_mode )
{
    open_port(device, nonblock_mode);
    
    if ( dev_open ) {
	set_baud(baud);
    }
}

SGSerialPort::~SGSerialPort() {
    // closing the port here screws us up because if we would even so
    // much as make a copy of an SGSerialPort object and then delete it,
    // the file descriptor gets closed.  Doh!!!
}

bool SGSerialPort::open_port( const string& device, bool nonblock_mode ) {
    struct termios config;

    if ( nonblock_mode ) {
	// sometimes the open() can block if you don't use
	// non-blocking mode
	fd = open( device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    } else {
	fd = open( device.c_str(), O_RDWR | O_NOCTTY );
    }

    printf( "Serial fd created = %d\n", fd);

    if ( fd  == -1 ) {
	printf( "Cannot open %s for serial I/O\n", device.c_str() );
	return false;
    } else {
	dev_open = true;
    }

    // Enable non-blocking IO if requested
    if ( nonblock_mode ) {
        fcntl(fd, F_SETFL, O_NONBLOCK);
    }

    // set required port parameters 
    if ( tcgetattr( fd, &config ) != 0 ) {
	printf( "Unable to poll port settings\n" );
	return false;
    }

    cfmakeraw( &config );

#if 0
    // in addition .... (testing)
    // disable software flow control on both input and output
    config.c_iflag &= ~(IXON | IXOFF );

    // enable any character to restart stopped output
    config.c_iflag |= (IXANY);

    // disable hardware flow control
    config.c_cflag &= ~(CRTSCTS);

    // these settings caused random program corruption on the MPC5200
    // and don't appear to actually be needed, so let's not set them
    // and we'll stick with the defulat bootup port settings for now.

    // cout << "config.c_iflag = " << config.c_iflag << endl;

    // disable LF expanded to CR-LF
    config.c_oflag &= ~(ONLCR);

    // disable software flow control
    config.c_iflag &= ~(IXON | IXOFF | IXANY);

    // enable the receiver and set local mode
    config.c_cflag |= (CLOCAL | CREAD);

#if !defined( sgi ) && !defined(_AIX)
    // disable hardware flow control
    config.c_cflag &= ~(CRTSCTS);
#endif

    // cout << "config.c_iflag = " << config.c_iflag << endl;
    
    // Raw (not cooked/canonical) input mode
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

#endif

    if ( tcsetattr( fd, TCSANOW, &config ) != 0 ) {
	printf( "Unable to update port settings\n" );
	return false;
    }

    return true;
}


bool SGSerialPort::close_port() {
    close(fd);

    dev_open = false;

    return true;
}


bool SGSerialPort::set_baud(int baud) {

    struct termios config;
    speed_t speed = B9600;

    if ( tcgetattr( fd, &config ) != 0 ) {
	printf( "Unable to poll port settings" );
	return false;
    }

    if ( baud == 300 ) {
	speed = B300;
    } else if ( baud == 1200 ) {
	speed = B1200;
    } else if ( baud == 2400 ) {
	speed = B2400;
    } else if ( baud == 4800 ) {
	speed = B4800;
    } else if ( baud == 9600 ) {
	speed = B9600;
    } else if ( baud == 19200 ) {
	speed = B19200;
    } else if ( baud == 38400 ) {
	speed = B38400;
    } else if ( baud == 57600 ) {
	speed = B57600;
    } else if ( baud == 115200 ) {
	speed = B115200;
    } else if ( baud == 230400 ) {
	speed = B230400;
    } else {
	printf( "Unsupported baud rate = %d", baud );
	return false;
    }

    if ( cfsetspeed( &config, speed ) != 0 ) {
	printf( "Problem setting baud rate" );
	return false;
    }

#if 0
    if ( cfsetispeed( &config, speed ) != 0 ) {
	printf( "Problem setting input baud rate" );
	return false;
    }

    if ( cfsetospeed( &config, speed ) != 0 ) {
	printf( "Problem setting output baud rate" );
	return false;
    }
#endif

    if ( tcsetattr( fd, TCSANOW, &config ) != 0 ) {
	printf( "Unable to update port settings" );
	return false;
    }

    return true;
}

bool SGSerialPort::set_nonblocking( ) {
    if ( dev_open ) {
	// Enable non-blocking IO
	if ( fcntl(fd, F_SETFL, O_NONBLOCK) == -1 ) {
	    perror("Setting nonblock mode");
	    return false;
	}
    } else {
	return false;
    }

    return true;
}


string SGSerialPort::read_port() {

    const int max_count = 1024;
    char buffer[max_count+1];
    string result;

    int count = read(fd, buffer, max_count);
    // cout << "read " << count << " bytes" << endl;

    if ( count < 0 ) {
	// error condition
	if ( errno != EAGAIN ) {
	    perror( "Serial I/O on read" );
	}

	return "";
    } else {
	buffer[count] = '\0';
	result = buffer;

	return result;
    }
}

int SGSerialPort::read_port(char *buf, int len) {
    return read(fd, buf, len);
}


int SGSerialPort::write_port(const string& value) {
    static bool error = false;
    int count;

    if ( error ) {
        printf( "attempting serial write error recovery" );
	// attempt some sort of error recovery
	count = write(fd, "\n", 1);
	if ( count == 1 ) {
	    // cout << "Serial error recover successful!\n";
	    error = false;
	} else {
	    return 0;
	}
    }

    count = write(fd, value.c_str(), value.length());
    // cout << "write '" << value << "'  " << count << " bytes" << endl;

    if ( (int)count == (int)value.length() ) {
	error = false;
    } else {
	if ( errno == EAGAIN ) {
	    // ok ... in our context we don't really care if we can't
	    // write a string, we'll just get it the next time around
	    error = false;
	} else {
	    error = true;
	    perror( "Serial I/O on write" );
	}
    }

    return count;
}


int SGSerialPort::write_port(const char* buf, int len) {
    return write(fd, buf, len);
}
