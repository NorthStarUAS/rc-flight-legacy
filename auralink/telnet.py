# asynchat example adapted from here:
#   http://www.grantjenks.com/wiki/random/python_asynchat_chat_example

import asynchat
import asyncore
import socket
import re

from props import root, getNode

import commands

class ChatHandler(asynchat.async_chat):
    def __init__(self, sock):
        asynchat.async_chat.__init__(self, sock=sock)
        self.set_terminator('\n')
        self.buffer = []
        self.path = '/'
        self.prompt = True
 
    def collect_incoming_data(self, data):
        self.buffer.append(data)
 
    def found_terminator(self):
        msg = ''.join(self.buffer)
        print 'Received:', msg  # fixme: if display on
        self.process_command(msg)
        self.buffer = []

    def process_command(self, msg):
        tokens = msg.split()
        if len(tokens) == 0:
            self.usage()
	elif tokens[0] == 'data':
	    self.prompt = False
	elif tokens[0] == 'prompt':
	    self.prompt = True
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
		children = node.getChildren(True)
		for child in children:
		    line = child
		    if node.isLeaf(child):
			if self.prompt:
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
                if re.search('/', tokens[1]):
                    if tokens[1][0] == '/':
                        # absolute path
                        tmp = tokens[1].split('/')
                    else:
                        # relative path
                        combinedpath = '/'.join([self.path, tokens[1]])
                        combinedpath = self.normalize_path(combinedpath)
                        tmp = combinedpath.split('/')
                    tmppath = '/'.join(tmp[0:-1])
                    if tmppath == '':
                        tmppath = '/'
                    node = getNode(tmppath, True)
                    name = tmp[-1]
                else:
                    node = getNode(self.path, True)
                    name = tokens[1]
		value = node.getString(name)
		if self.prompt:
		    self.push(tokens[1] + ' = "' + value + '"\n')
                else:
                    self.push(value + '\n')
	    else:
		self.push('usage: get [[/]path/]attr\n')
	elif tokens[0] == 'set':
	    if len(tokens) >= 3:
                if re.search('/', tokens[1]):
                    if tokens[1][0] == '/':
                        # absolute path
                        tmp = tokens[1].split('/')
                    else:
                        # relative path
                        combinedpath = '/'.join([self.path, tokens[1]])
                        combinedpath = self.normalize_path(combinedpath)
                        tmp = combinedpath.split('/')
                    tmppath = '/'.join(tmp[0:-1])
                    if tmppath == '':
                        tmppath = '/'
                    node = getNode(tmppath, True)
                    name = tmp[-1]
                else:
                    node = getNode(self.path, True)
                    name = tokens[1]
		value = ' '.join(tokens[2:])
		node.setString(name, value)
		if self.prompt:
		    # now fetch and write out the new value as confirmation
		    # of the change
		    value = node.getString(name)
		    self.push(tokens[1] + ' = "' + value + '"\n')
	    else:
		self.push('usage: set [[/]path/]attr value\n')
	elif tokens[0] == 'send':
            c = ' '
            commands.add(c.join(tokens[1:]))
            
	# elif tokens[0] == 'run':
	#     if len(tokens) == 2:
	# 	string command = tokens[1]
	# 	if command == 'ap.reinit()':
	# 	    control_reinit()
	# 	else:
	# 	    push( 'unknown command: ' )
	# 	    push( tokens[1].c_str() )
	# 	    push( getTerminator() )
	#     else:
	# 	push( 'usage: run <command>' )
	# 	push( getTerminator() )
	elif tokens[0] == 'quit':
	    self.close()
	    return
	elif tokens[0] == 'shutdown-server':
            if len(tokens) == 2:
                if tokens[1] == 'xyzzy':
	            quit()
            self.push('usage: shutdown-server xyzzy\n')
            self.push('extra magic argument is required\n')
	# elif tokens[0] == 'fcs':
	#     if len(tokens) == 2:
	# 	string tmp = ""
	# 	if self.prompt:
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
	# elif tokens[0] == 'fcs-update':
	#     if len(tokens) == 2:
	# 	bool result = fcs_update_helper(tokens[1])
	# 	if self.prompt:
	# 	    string tmp
	# 	    if result:
	# 		tmp = "new values accepted ok"
	# 	    else:
	# 		tmp = "update failed!"
	# 	    push( tmp.c_str() )
	# 	    push( getTerminator() )
	else:
            self.usage()

        if self.prompt:
            self.push('> ')
            
    def usage(self):
        message = """
Valid commands are:

help               show this help message
data               switch to raw data mode
prompt             switch to interactive mode (default)
ls [<dir>]         list directory
cd <dir>           cd to a directory, '..' to move back
pwd                display your current path
get <var>          show the value of a parameter
set <var> <val>    set <var> to a new <val>
dump [<dir>]       dump the current state (in xml)
# run <command>      run built in command
quit               terminate client connection
shutdown-server    instruct host server to exit (requires magic argument)
"""
        self.push(message)
    
    def normalize_path(self, raw_path):
        tokens = raw_path.split('/')
        #print tokens
        tmp = ['']
        for t in tokens:
            if t == '..':
                if len(tmp) > 1:
                    tmp.pop()
            elif t == '.':
                # do nothing
                pass
            elif t == '':
                # happens if we have double slashes
                pass
            else:
                tmp.append(t)
        #print tmp
        result = '/'.join(tmp)
        if result == '':
            result = '/'
        #print 'Original path:', raw_path
        #print 'new      path:', result
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
 
def init(port=5050):
    server = ChatServer('localhost', port)
    print 'Telnet server on localhost:' + str(port)

def update():
    asyncore.loop(timeout=0, count=1)
