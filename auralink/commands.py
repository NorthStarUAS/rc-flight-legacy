import time

from props import root, getNode

filter_node = getNode('/filters/filter', True)
remote_link_node = getNode('/comms/remote_link', True)

cmd_send_index = 0
cmd_recv_index = 0
prime_state = True

cmd_queue =  []
last_sent_time = 0.0
last_received_time = 0.0

# calculate the nmea check sum
def calc_nmea_cksum(sentence):
    sum = 0
    # print sentence
    size = len(sentence)
    sum = ord(sentence[0])
    for i in range(1, size):
        # print sentence[i],
        sum = (sum ^ ord(sentence[i])) & 0xff
    # print
    # print "sum = %02x\n" % sum
    return sum

# package and send the serial command, returns number of bytes written
def serial_send(serial, sequence, command):
    package = str(sequence) + ',' + command
    pkg_sum = "%02X" % calc_nmea_cksum(package)
    package = package + '*' + pkg_sum + '\n'
    print 'writing:', package
    result = serial.write(package)
    if result != len(package):
        print "ERROR: wrote %d of %d bytes to serial port!\n" % (result, len(package))
    return result

# send current command until acknowledged
def update(serial):
    global cmd_send_index
    global cmd_recv_index
    global last_sent_time
    global last_received_time
    global prime_state

    # look at the remote's report of last message received from base
    sequence = remote_link_node.getInt('sequence_num')
    if sequence != cmd_recv_index:
	last_received_time = time.time()
	cmd_recv_index = sequence

    # if current command has been received, advance to next command
    if cmd_recv_index == cmd_send_index:
        if len(cmd_queue):
            if not prime_state:
                cmd_queue.pop(0)
                cmd_send_index = (cmd_send_index + 1) & 0xff
            else:
                prime_state = False

    gen_heartbeat()
    
    if len(cmd_queue):
        current_time = time.time()
        if current_time > last_sent_time + 0.5:
            # discard any pending heartbeat commands if we have real work
            while len(cmd_queue) > 1 and cmd_queue[0] == 'hb':
                cmd_queue.pop(0)
            # send the command
            command = cmd_queue[0]
            result = serial_send(serial, cmd_send_index, command)
            last_sent_time = current_time
            print "sent = %d  recv = %d" % (cmd_send_index, cmd_recv_index)
            return cmd_send_index
    else:
        # nothing to do if command queue empty
        prime_state = True
        
    return 0

def add(command):
    print 'command queue:', command
    cmd_queue.append(command)

def cmd_queue_size():
    return len(cmd_queue)

def cmd_queue_empty():
    return len(cmd_queue) == 0

def get_cmd_recv_index():
    return cmd_recv_index

# schedule a heartbeat message if needed.
def gen_heartbeat():
    global last_received_time
    elapsed_sec = time.time() - last_received_time
    if cmd_queue_empty() and elapsed_sec > 10.0:
        add('hb')
        
def remote_lost_link_predict():
    global last_received_time
    
    # print "last = %.2f  cur = %.2f", (last_delivered_time, current_time)
    if last_received_time + 60 > time.time():
	return True
    else:
	return False

