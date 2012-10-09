#ifndef _FG_UGEAR_COMMAND_HXX
#define _FG_UGEAR_COMMAND_HXX


#include <stdint.h>

#include <iostream>
#include <string>
#include <queue>

#include "serial.hxx"

using std::cout;
using std::endl;
using std::string;
using std::queue;


// Manage UGear Command Channel
class UGCommand {

private:

    uint8_t cmd_send_index;
    uint8_t cmd_recv_index;
    bool prime_state;
    queue <string> cmd_queue;
    double current_time;
    double last_delivered_time;

public:

    UGCommand();
    ~UGCommand();

    // send current command until acknowledged
    int update( SGSerialPort *serial );

    void add( const string command );
    inline int cmd_queue_size() {
        return cmd_queue.size();
    }
    inline int cmd_queue_empty() {
        return cmd_queue.empty();
    }
    inline void update_cmd_sequence( uint8_t sequence, double time ) {
        // printf("update_cmd_sequence = %.4f\n", time);
	current_time = time;
	if ( sequence != cmd_recv_index ) {
	    last_delivered_time = time;
	    cmd_recv_index = sequence;
	}
    }
    inline uint8_t get_cmd_recv_index() {
	return cmd_recv_index;
    }
    inline bool remote_lost_link_predict() {
	printf("last = %.2f  cur = %.2f\n", last_delivered_time, current_time);
	if ( last_delivered_time + 20 > current_time ) {
	    return true;
	}
	return false;
    }
};


extern UGCommand command_mgr;


#endif // _FG_UGEAR_COMMAND_HXX
