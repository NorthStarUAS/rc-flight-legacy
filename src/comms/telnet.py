# asynchat example adapted from here:
#   http://www.grantjenks.com/wiki/random/python_asynchat_chat_example

import asynchat
import asyncore
import socket
import re

from props import getNode

#import commands

class ChatHandler(asynchat.async_chat):
    def __init__(self, sock):
        asynchat.async_chat.__init__(self, sock=sock)
        self.set_terminator(b'\n')
        self.buffer = []
        self.path = '/'
        self.prompt = True

        self.imu_node = getNode("/sensors/imu", True)
        self.targets_node = getNode("/autopilot/targets", True)
        self.filter_node = getNode("/filters/filter", True)
        self.act_node = getNode("/actuators/actuator", True)
        self.vel_node = getNode("/velocity", True)
        self.pos_comb_node = getNode("/position/combined", True)

    def collect_incoming_data(self, data):
        self.buffer.append(data.decode())

    def found_terminator(self):
        msg = ''.join(self.buffer)
        print('Received:', msg)  # fixme: if display on
        self.process_command(msg)
        self.buffer = []

    def gen_fcs_nav_string(self):
        result = [ self.targets_node.getFloat('groundtrack_deg'),
                   self.targets_node.getFloat('roll_deg'),
                   self.filter_node.getFloat('heading_deg'),
                   self.filter_node.getFloat('roll_deg'),
                   self.act_node.getFloatEnum('channel', 0) ]
        return ','.join(map(str, result))

    def gen_fcs_speed_string(self):
        result = [ self.targets_node.getFloat('airspeed_kt'),
                   self.targets_node.getFloat('pitch_deg'),
                   self.vel_node.getFloat('airspeed_smoothed_kt'),
                   self.filter_node.getFloat('pitch_deg'),
                   self.act_node.getFloatEnum('channel', 1) ]
        return ','.join(map(str, result))

    def gen_fcs_altitude_string(self):
        m2ft = 1.0 / 0.3048
        result = [ self.targets_node.getFloat('altitude_msl_ft'),
                   self.pos_comb_node.getFloat('altitude_true_m') * m2ft,
                   self.act_node.getFloatEnum('channel', 2) ]
        return ','.join(map(str, result))

    def my_push(self, msg):
        self.push(str.encode(msg))
        
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
                children = node.getChildren(expand=False)
                for child in children:
                    if node.isEnum(child):
                        line = ''
                        for i in range(node.getLen(child)):
                            if node.isLeaf(child):
                                value = node.getStringEnum(child, i)
                                line += '%s[%d]' % (child, i)
                                line += ' =\t\"' + value + '"\t' + '\n'
                            else:
                                line += '%s[%d]/' % (child, i) + '\n'
                    else:
                        if node.isLeaf(child):
                            value = node.getString(child)
                            line = child + ' =\t\"' + value + '"\t' + '\n'
                        else:
                            line = child + '/' + '\n'
                    self.my_push(line)
            else:
                self.my_push('Error: ' + newpath + ' not found\n')
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
                self.my_push('path ok: ' + newpath + '\n')
                self.path = newpath
            else:
                self.my_push('Error: ' + newpath + ' not found\n')
        elif tokens[0] == 'pwd':
            self.my_push(self.path + '\n' )
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
                    self.my_push(tokens[1] + ' = "' + value + '"\n')
                else:
                    self.my_push(value + '\n')
            else:
                self.my_push('usage: get [[/]path/]attr\n')
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

                done = False
                # test for int
                if not done:
                    result = re.match('[-+]?\d+', value)
                    if result and result.group(0) == value:
                        print('int:', value)
                        node.setInt(name, int(value))
                        done = True
                # test for float
                if not done:
                    result = re.match('[-+]?\d*\.\d+', value)
                    if result and result.group(0) == value:
                        print('float:', value)
                        node.setFloat(name, float(value))
                        done = True
                # test for bool
                if not done:
                    if value == 'True' or value == 'true':
                        print('bool:', True)
                        node.setBool(name, True)
                        done = True
                if not done:
                    if value == 'False' or value == 'false':
                        print('bool:', False)
                        node.setBool(name, False)
                        done = True
                # fall back to string
                if not done:
                    node.setString(name, value)

                if self.prompt:
                    # now fetch and write out the new value as confirmation
                    # of the change
                    value = node.getString(name)
                    self.my_push(tokens[1] + ' = "' + value + '"\n')
            else:
                self.my_push('usage: set [[/]path/]attr value\n')
        elif tokens[0] == 'quit':
            self.close()
            return
        elif tokens[0] == 'shutdown-application':
            if len(tokens) == 2:
                if tokens[1] == 'xyzzy':
                    quit()
            self.my_push('usage: shutdown-application xyzzy\n')
            self.my_push('extra magic argument is required\n')
        else:
            self.usage()

        if self.prompt:
            self.my_push('> ')

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
quit               exit the client telnet session
shutdown-application xyzzy      terminate the host application
"""
        self.my_push(message)

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
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind((host, port))
        self.listen(5)

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            print('Incoming connection from %s' % repr(addr))
            handler = ChatHandler(sock)

telnet_enabled = False
def init(port=6499):
    global telnet_enabled

    telnet_node = getNode( '/config/telnet', True )
    port = telnet_node.getInt('port')
    if port:
        server = ChatServer('localhost', port)
        telnet_enabled = True
        print('Telnet server on localhost:' + str(port))
    else:
        telnet_enabled = False

def update(dt=0):
    # dt is unused but makes pyModuleBase happy
    if telnet_enabled:
        asyncore.loop(timeout=0, count=1)
