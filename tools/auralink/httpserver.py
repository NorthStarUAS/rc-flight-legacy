# tornado based websocket server

import json
import tornado
import tornado.httpserver
import tornado.websocket

from PropertyTree import PropertyNode
import props_json

import commands
import projects

class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        print('new connection')
        self.bind_props()
      
    def on_message(self, message):
        # print('message received:  %s' % message)
        [command, args] = message.rstrip().split(' ', 1)
        # print(tokens)
        if command == 'get':
            if args == 'full_json':
                commands.remote_lost_link_predict()
                PropertyNode("/").setBool("main_magic", True)
                # print(len(PropertyNode("/").write_as_string()))
                self.write_message(PropertyNode("/").write_as_string() + '\r\n')
        elif command == 'send':
            # request relay 'args' string up to aircraft
            commands.add(str(args))
        elif command == 'projects_get':
            print("request for list of all projects")
            json_str = projects.load()
            print('project json:', json_str)
            self.write_message(json_str + '\r\n')
        elif command == 'projects_update':
            projects.update_name(args)
        elif command == 'projects_delete':
            projects.delete_name(args)

    def on_close(self):
        print('connection closed')
 
    def check_origin(self, origin):
        return True

    def bind_props(self):
        print('binding property nodes')
        self.gps_node = PropertyNode('/sensors/gps/0')
        self.imu_node = PropertyNode('/sensors/imu/0')
        self.airdata_node = PropertyNode('/sensors/airdata')
        self.pilot_node = PropertyNode('/sensors/pilot_input')
        self.filter_node = PropertyNode('/filters/filter')
        self.pos_combined_node = PropertyNode('/position/combined')
        self.velocity_node = PropertyNode('/velocity')
        self.act_node = PropertyNode('/actuators/actuator')
        self.ap_node = PropertyNode('/autopilot')
        self.targets_node = PropertyNode('/autopilot/targets')
        self.route_node = PropertyNode('/task/route')
        self.active_node = PropertyNode('/task/route/active')
        self.home_node = PropertyNode('/task/home')
        self.circle_node = PropertyNode('/task/circle')
        self.status_node = PropertyNode('/status')
        self.cam_node = PropertyNode('/payload/camera') 
        self.wind_node = PropertyNode('/filters/wind') 
        self.apm2_node = PropertyNode('/sensors/APM2') 
   
def nullfunc():
    pass

def init(port=8888, html_root='.'):
    application = tornado.web.Application([
        (r'/ws', WSHandler),
        (r'/(.*)', tornado.web.StaticFileHandler,
         {'path': html_root, 'default_filename': 'index.html'}),
    ])

    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(port)
    print('Http server on http://localhost:' + str(port) + '/')
    print('Websocket server on http://localhost:' + str(port) + '/ws')
    
def update():
    tornado.ioloop.IOLoop.instance().run_sync(nullfunc)
    
