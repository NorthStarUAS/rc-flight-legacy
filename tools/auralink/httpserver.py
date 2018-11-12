# tornado based websocket server

import json
import tornado
import tornado.httpserver
import tornado.websocket

from props import root, getNode
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
                dict_mirror = {}
                root.setBool("main_magic", True)
                props_json.buildDict(dict_mirror, root)
                self.write_message(json.dumps(dict_mirror, separators=(',',':'),
                                              sort_keys=True) + '\r\n')
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
        self.gps_node = getNode('/sensors/gps[0]', True)
        self.imu_node = getNode('/sensors/imu', True)
        self.airdata_node = getNode('/sensors/airdata', True)
        self.pilot_node = getNode('/sensors/pilot_input', True)
        self.filter_node = getNode('/filters/filter', True)
        self.pos_combined_node = getNode('/position/combined', True)
        self.velocity_node = getNode('/velocity', True)
        self.act_node = getNode('/actuators/actuator', True)
        self.ap_node = getNode('/autopilot', True)
        self.targets_node = getNode('/autopilot/targets', True)
        self.route_node = getNode('/task/route', True)
        self.active_node = getNode('/task/route/active', True)
        self.home_node = getNode('/task/home', True)
        self.circle_node = getNode('/task/circle', True)
        self.status_node = getNode('/status', True)
        self.cam_node = getNode('/payload/camera', True) 
        self.wind_node = getNode('/filters/wind', True) 
        self.apm2_node = getNode('/sensors/APM2', True) 
   
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
    
