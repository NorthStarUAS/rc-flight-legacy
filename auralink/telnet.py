# asynchat example adapted from here:
#   http://www.grantjenks.com/wiki/random/python_asynchat_chat_example

import asynchat
import asyncore
import socket
import props
 
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
        self.push(msg + '\n')
        self.buffer = []

    def process_command(self, msg):
        tokens = msg.split()
        print tokens            # fixme: if display_on:
        if len(tokens) == 0:
            self.usage()
            return False
	if tokens[0] == 'ls':
            newpath = self.path
	    if tokens.size() == 2:
		if tokens[1][0] == '/':
		    newpath = tokens[1]
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
	    if tokens.size() == 2:
		if tokens[1][0] == '/':
		    newpath = tokens[1]
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
	    push( getTerminator() )
	elif tokens[0] == 'get' or tokens[0] == 'show':
	    if tokens.size() == 2:
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
	elif tokens[0] == 'fcs':
	    if tokens.size() == 2:
		string tmp = ""
		if mode == PROMPT:
		    tmp = tokens[1]
		    tmp += " = "
		if tokens[1] == "heading":
		    tmp += packetizer->get_fcs_nav_string()
		elif tokens[1] == "speed":
		    tmp += packetizer->get_fcs_speed_string()
		elif tokens[1] == "altitude":
		    tmp += packetizer->get_fcs_altitude_string()
		elif tokens[1] == "all":
		    tmp += packetizer->get_fcs_nav_string()
		    tmp += ","
		    tmp += packetizer->get_fcs_speed_string()
		    tmp += ","
		    tmp += packetizer->get_fcs_altitude_string()
		push( tmp.c_str() )
		push( getTerminator() )
	elif tokens[0] == "fcs-update":
	    if tokens.size() == 2:
		bool result = fcs_update_helper(tokens[1])
		if mode == PROMPT:
		    string tmp
		    if result:
			tmp = "new values accepted ok"
		    else:
			tmp = "update failed!"
		    push( tmp.c_str() )
		    push( getTerminator() )
	elif tokens[0] == "set":
	    if tokens.size() >= 2:
		string value = "", tmp
		for (unsigned int i = 2; i < tokens.size(); i++:
		    if i > 2:
			value += " "
		    value += tokens[i]
		node.setString( tokens[1].c_str(), value )
		if mode == PROMPT:
		    # now fetch and write out the new value as confirmation
		    # of the change
		    value = node.getString ( tokens[1].c_str() )
		    tmp = tokens[1] + " = '" + value + "'"
		    push( tmp.c_str() )
		    push( getTerminator() )
	elif tokens[0] == "run":
	    if tokens.size() == 2:
		string command = tokens[1]
		if command == "ap.reinit()":
		    control_reinit()
		else:
		    push( "unknown command: " )
		    push( tokens[1].c_str() )
		    push( getTerminator() )
	    else:
		push( "usage: run <command>" )
		push( getTerminator() )
	elif tokens[0] == "quit":
	    close()
	    shouldDelete()
	    return
	elif tokens[0] == 'exit-program':
	    quit()
	elif tokens[0] == 'data':
	    mode = 'data'
	elif tokens[0] == 'prompt':
	    mode = 'prompt'1
	else:
            self.usage()
            
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
"""
        self.push(message)
    
    def normalize_path(raw_path):
        tokens = raw_path.split('/')
        tmp = []
        for t in tokens:
            if t == '..':
                if len(tmp):
                    tmp.pop_back()
            elif t == ".":
                # do nothing
                pass
            else:
                tmp.append(t)
        if len(tmp):
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
