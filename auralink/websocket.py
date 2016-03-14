# tornado based websocket server

import tornado
import tornado.httpserver
import tornado.websocket

from props import root, getNode

class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        print 'new connection'
      
    def on_message(self, message):
        print 'message received:  %s' % message
        # Reverse Message and send it back
        #print 'sending back message: %s' % message[::-1]
        #self.write_message(message[::-1])
        tmp = message.split('/')
        tmppath = '/'.join(tmp[0:-1])
        if tmppath == '':
            tmppath = '/'
        node = getNode(tmppath, True)
        name = tmp[-1]
        value = node.getString(name)
        self.write_message(message + ': ' + value)
        
    def on_close(self):
        print 'connection closed'
 
    def check_origin(self, origin):
        return True
    
def nullfunc():
    pass

application = tornado.web.Application([
    (r'/ws', WSHandler),
])

http_server = tornado.httpserver.HTTPServer(application)

def init(port=8888):
    http_server.listen(port)
    print 'Websocket server on http://localhost:' + str(port) + '/ws'
    
def update():
    tornado.ioloop.IOLoop.instance().run_sync(nullfunc)
    
