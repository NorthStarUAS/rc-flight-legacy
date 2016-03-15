cmd_send_index = 0
cmd_recv_index = 0
prime_state = True

cmd_queue =  []
current_time = 0.0
last_delivered_time = 0.0

# calculate the nmea check sum
def calc_nmea_cksum(sentence):
    sum = 0
    # print sentence
    len = len(sentence)
    sum = sentence[0]
    for i in range(1, len(sentence)):
        # print sentence[i],
        sum = (sum ^ sentence[i]) & 0xff
    # print
    # print "sum = %02x\n" % sum
    return sum

# package and send the serial command, returns number of bytes written
def serial_send(serial, sequence, command):
    package = str(sequence) + ',' + command
    pkg_sum = "%02X" & calc_nmea_cksum(package)
    package = package + '*' + pkg_sum + '\n'
    print 'writing:', package
    result = serial.write(package)
    if result != len(package):
        print "ERROR: wrote %d of %d bytes to serial port!\n" % (result, len(package))
    return result

# send current command until acknowledged
int update(serial)
    # if current command has been received, advance to next command
    print "sent = %d  recv = %d" % (cmd_send_index, cmd_recv_index)
    if cmd_recv_index == cmd_send_index:
        if not cmd_queue.empty():
            if not prime_state:
                cmd_queue.pop(0)
                cmd_send_index = cmd_send_index + 1
            else:
                prime_state = False

    # nothing to do if command queue empty
    if cmd_queue.empty():
        prime_state = True
        return 0

    # send the command
    command = cmd_queue.front()
    result = serial_send(serial, cmd_send_index, command)
    return cmd_send_index

def add(command):
    print 'command queue:', command
    cmd_queue.append(command)

def cmd_queue_size():
    return len(cmd_queue)

def cmd_queue_empty():
    return len(cmd_queue) == 0

def update_cmd_sequence(sequence, time):
    # print 'update_cmd_sequence:', time
    current_time = time
    if sequence != cmd_recv_index:
	last_delivered_time = time
	cmd_recv_index = sequence

def get_cmd_recv_index():
    return cmd_recv_index

def remote_lost_link_predict():
    # print "last = %.2f  cur = %.2f", (last_delivered_time, current_time)
    if last_delivered_time + 60 > current_time:
	return True
    else:
	return False

