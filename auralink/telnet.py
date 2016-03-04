# asynchat example adapted from here:
#   http://www.grantjenks.com/wiki/random/python_asynchat_chat_example

import asynchat
import asyncore
import socket
from props import root, getNode
 
class ChatHandler(asynchat.async_chat):
    def __init__(self, sock):
        asynchat.async_chat.__init__(self, sock=sock)
        self.set_terminator('\n')
        self.buffer = []
        self.path = '/'
        self.mode = 'prompt'
 
    def collect_incoming_data(self, data):
        self.buffer.append(data)
 
    def found_terminator(self):
        msg = ''.join(self.buffer)
        print 'Received:', msg
        self.process_command(msg)
        # self.push(msg + '\n')
        self.buffer = []

    def process_command(self, msg):
        tokens = msg.split()
        if len(tokens) == 0:
            self.usage()
	elif tokens[0] == 'ls':
            newpath = self.path
	    if len(tokens) == 2:
		if tokens[1][0] == '/':
		    newpath = tokens[1]
		else:
                    if self.path[-1] == '/':
		        newpath = self.path + tokens[1]
                    else:
		        newpath = self.path + '/' + tokens[1]
            newpath = self.normalize_path(newpath)
	    node = getNode(newpath)
	    if node:
		children = node.getChildren(False)
		for child in children:
		    line = child
		    if node.isLeaf(child):
			if self.mode == 'prompt':
			    value = node.getString(child)
			    line = line + ' =\t\"' + value + '"\t'
		    else:
			line += '/'
		    line += '\n'
		    self.push(line)
	    else:
		self.push('Error: ' + newpath + ' not found\n')
	elif tokens[0] == 'cd':
            newpath = self.path
	    if len(tokens) == 2:
		if tokens[1][0] == '/':
		    newpath = tokens[1]
		else:
                    if self.path[-1] == '/':
		        newpath = self.path + tokens[1]
                    else:
		        newpath = self.path + '/' + tokens[1]
            newpath = self.normalize_path(newpath)
	    node = getNode(newpath)
	    if node:
		self.push('path ok: ' + newpath + '\n')
		self.path = newpath
	    else:
		self.push('Error: ' + newpath + ' not found\n')
	elif tokens[0] == 'pwd':
	    self.push(self.path + '\n' )
	elif tokens[0] == 'get' or tokens[0] == 'show':
	    if len(tokens) == 2:
                node = getNode(self.path)
                if node:
		    value = node.getString(tokens[1])
                else:
                    value = ''
		if mode == 'prompt':
		    line = tokens[1] + ' = "' + value + '"'
		else:
		    line = value
		push(line + '\n')
	# elif tokens[0] == 'fcs':
	#     if len(tokens) == 2:
	# 	string tmp = ""
	# 	if mode == PROMPT:
	# 	    tmp = tokens[1]
	# 	    tmp += " = "
	# 	if tokens[1] == "heading":
	# 	    tmp += packetizer->get_fcs_nav_string()
	# 	elif tokens[1] == "speed":
	# 	    tmp += packetizer->get_fcs_speed_string()
	# 	elif tokens[1] == "altitude":
	# 	    tmp += packetizer->get_fcs_altitude_string()
	# 	elif tokens[1] == "all":
	# 	    tmp += packetizer->get_fcs_nav_string()
	# 	    tmp += ","
	# 	    tmp += packetizer->get_fcs_speed_string()
	# 	    tmp += ","
	# 	    tmp += packetizer->get_fcs_altitude_string()
	# 	push( tmp.c_str() )
	# 	push( getTerminator() )
	# elif tokens[0] == "fcs-update":
	#     if len(tokens) == 2:
	# 	bool result = fcs_update_helper(tokens[1])
	# 	if mode == PROMPT:
	# 	    string tmp
	# 	    if result:
	# 		tmp = "new values accepted ok"
	# 	    else:
	# 		tmp = "update failed!"
	# 	    push( tmp.c_str() )
	# 	    push( getTerminator() )
	elif tokens[0] == 'set':
	    if len(tokens) >= 2:
		value = ''
                tmp = ''
		for i in range(2..end):
		    if i > 2:
			value = value + ' '
		    value = value + tokens[i]
		node.setString( tokens[1].c_str(), value )
		if mode == PROMPT:
		    # now fetch and write out the new value as confirmation
		    # of the change
		    value = node.getString ( tokens[1].c_str() )
		    tmp = tokens[1] + " = '" + value + "'"
		    push( tmp.c_str() )
		    push( getTerminator() )
	# elif tokens[0] == "run":
	#     if len(tokens) == 2:
	# 	string command = tokens[1]
	# 	if command == "ap.reinit()":
	# 	    control_reinit()
	# 	else:
	# 	    push( "unknown command: " )
	# 	    push( tokens[1].c_str() )
	# 	    push( getTerminator() )
	#     else:
	# 	push( "usage: run <command>" )
	# 	push( getTerminator() )
	elif tokens[0] == "quit":
	    self.close()
	    #self.shouldDelete()
	    return
	elif tokens[0] == 'shutdown-server':
            if len(tokens) == 2:
                if tokens[1] == 'xyzzy':
	            quit()
            self.push('usage: shutdown-server xyzzy\n')
            self.push('extra magic argument is required\n')
	elif tokens[0] == 'data':
	    self.mode = 'data'
	elif tokens[0] == 'prompt':
	    self.mode = 'prompt'
	else:
            self.usage()

        if self.mode == 'prompt':
            self.push('> ')
            
    def usage(self):
        message = """
Valid commands are:

cd <dir>           cd to a directory, '..' to move back
data               switch to raw data mode
dump [<dir>]       dump the current state (in xml)
get <var>          show the value of a parameter
help               show this help message
ls [<dir>]         list directory
prompt             switch to interactive mode (default)
pwd                display your current path
quit               terminate connection
# run <command>      run built in command
set <var> <val>    set <var> to a new <val>
shutdown-server xyzzy  instruct host server to exit (requires magic argument)
"""
        self.push(message)
    
    def normalize_path(self, raw_path):
        tokens = raw_path.split('/')
        tmp = []
        for t in tokens:
            if t == '..':
                if len(tmp):
                    tmp.pop()
            elif t == ".":
                # do nothing
                pass
            else:
                tmp.append(t)
        print tmp
        if len(tmp) > 1:
            sep = '/'
            result = sep.join(tmp)
        else:
            result = '/'
        print 'Original path:', raw_path
        print 'new      path:', result
        return result

class ChatServer(asyncore.dispatcher):
    def __init__(self, host, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind((host, port))
        self.listen(5)
 
    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print 'Incoming connection from %s' % repr(addr)
            handler = ChatHandler(sock)
 
server = ChatServer('localhost', 5050)

print 'Serving on localhost:5050'

def update():
    asyncore.loop(timeout=0, count=1)
