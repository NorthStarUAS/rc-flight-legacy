// a (hopefully) simple program to wrap up serial comms in a separate
// process and then exposing a network interface into this process so
// that a remote application can freely read and write to the comm
// port without any concern of blocking issues or serial port timing
// or buffering.


// Goal of this code: reasonable throughput, low system over head
// (plays nice with others)

// network server is hardwired to allow one connection at a time and
// reject any other connection attempts with a suitable message.

#include <stdio.h>
#include <stdlib.h>
#include "comms/serial.hxx"

#include "netbuffer.hxx"


int main() {
    printf("start of main!\n");

    netBufferChannel server;

    netBufferChannel::tmp_buffer.init(1024);
    netBufferChannel::in_buffer.init(1024);
    netBufferChannel::out_buffer.init(1024);

    netBufferChannel::connection_count = 0;
    netBufferChannel::lossless = false;

    int port = 6500;
    string device = "/dev/ttyUSB0";
    int baud = 115200;

    SGSerialPort console;
    printf("before opening %s\n", device.c_str() );
    if ( ! console.open_port( device, true ) ) {
	printf("error opening serial port %s\n", device.c_str() );
	exit(-1);
    } else {
	printf("opened %s\n", device.c_str() );
    }
    console.set_baud( baud );

    server.open();
    server.bind( "", port );
    server.listen( 5 );
    printf("net server started on port %d\n", port );

    char serial_buf[256];

    // this whole approach is in sore need of some thought about what
    // to do with buffer overruns ... can we stupidly drop data, or is
    // there a more sensible way to drop data and retry?  Could the stupid
    // approach potentially lead to starvation?

    while ( true ) {
	// service any pending network traffic, timeout is set to 10
	// microseconds (0.01 sec) so if we don't see any network
	// traffic within that time period we can forge ahead and do
	// some serial IO
	server.poll( 10 /* microseconds */ );

	// if we have any new data read from the network interface,
	// write it to the serial port.

	// notice: the network server layer attempts to buffer
	// everything and lose nothing, so if we get bombarded with
	// too much data to push down the serial port, then we need to
	// make some hard decisions.  For the moment we make our best
	// attempt to write what is in the buffer, and then flush the
	// buffer so we can receive more data.  We absolutely must not
	// make the remote client block by not servicing the data
	// quickly enough.

	int in_len = netBufferChannel::in_buffer.getLength();
	if ( in_len ) {
	    int bytes_written
		= console.write_port( netBufferChannel::in_buffer.getData(),
				      in_len );
	    if ( bytes_written < 0 ) {
		// perror("serial write");
	    } else if ( bytes_written == 0 ) {
		// nothing was written
	    } else if ( bytes_written != in_len ) {
		// not a full write
		netBufferChannel::in_buffer.remove(0, bytes_written);
	    } else if ( bytes_written == in_len ) {
		// everything was written so clear the entire input buffer
		netBufferChannel::in_buffer.remove();
	    } else {
		// huh?
	    }
	}

	// read the serial port
	int bytes_read = console.read_port( serial_buf, 256 );
	if ( bytes_read > 0 ) {
	    bool result = netBufferChannel::out_buffer.append( serial_buf,
							       bytes_read );
	    if ( ! result ) {
		// the out_buffer is full, bummer.  This means we just
		// dropped some data that we read from the serial port
		// before it could be passed along to the destination.
		// This is a case that I can't anticipate happening.
		// Our inbound feeder pipe is teeny, but the outgoing
		// network pipe is big.

		// is there some other way we should deal with this,
		// other than just dropping the data?  Where should it
		// go, how much more should we hold in limbo?
	    }
	}
    }

    server.close();

    return 0;
}
