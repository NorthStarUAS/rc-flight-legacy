# tornado based websocket server

import tornado
import tornado.httpserver
import tornado.websocket

class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        print 'new connection'
      
    def on_message(self, message):
        print 'message received:  %s' % message
        # Reverse Message and send it back
        print 'sending back message: %s' % message[::-1]
        self.write_message(message[::-1])
 
    def on_close(self):
        print 'connection closed'
 
    def check_origin(self, origin):
        return True
    
def nullfunc():
    pass

def update():
    tornado.ioloop.IOLoop.instance().run_sync(nullfunc)
    
application = tornado.web.Application([
    (r'/ws', WSHandler),
])

http_server = tornado.httpserver.HTTPServer(application)
http_server.listen(8888)

print 'Websocket server on http://localhost:8888/ws'
    
