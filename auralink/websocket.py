# tornado based websocket server

import json
import tornado
import tornado.httpserver
import tornado.websocket

from props import root, getNode

class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        print 'new connection'
        self.bind_props()
      
    def on_message(self, message):
        print 'message received:  %s' % message
        # Reverse Message and send it back
        # print 'sending back message: %s' % message[::-1]
        # self.write_message(message[::-1])

        # I forget what exaclty this tries to do
        # tmp = message.split('/')
        # tmppath = '/'.join(tmp[0:-1])
        # if tmppath == '':
        #     tmppath = '/'
        # node = getNode(tmppath, True)
        # name = tmp[-1]
        # value = node.getString(name)
        # self.write_message(message + ': ' + value)

        tokens = message.split()
        print tokens
        if tokens[0] == 'get':
            if tokens[1] == 'update_json':
                dict = {}
                dict['lon'] = "%.8f" % self.filter_node.getFloat('longitude_deg')
                dict['lat'] = "%.8f" % self.filter_node.getFloat('latitude_deg')
                dict['alt_true'] = "%.1f" % self.pos_combined_node.getFloat('altitude_true_m')
                dict['airspeed'] = "%.1f" % self.velocity_node.getFloat('airspeed_smoothed_kt')
                dict['filter_psi'] = "%.1f" % self.filter_node.getFloat('heading_deg')
                dict['filter_track'] = "%.1f" % self.filter_node.getFloat('track_deg')
                dict['filter_speed'] = "%.1f" % self.filter_node.getFloat('speed_kt')
                dict['wind_deg'] = "%.1f" % self.wind_node.getFloat('wind_dir_deg')
                dict['wind_kts'] = "%.1f" % self.wind_node.getFloat('wind_speed_kt')
                dict['gps_sats'] = "%d" % self.gps_node.getFloat('satellites')
                print json.dumps(dict)
                self.write_message('update_json ' + json.dumps(dict, separators=(',',':')) + '\r\n')
        
    def on_close(self):
        print 'connection closed'
 
    def check_origin(self, origin):
        return True

    def bind_props(self):
        print 'binding property nodes'
        self.gps_node = getNode('/sensors/gps', True)
        self.imu_node = getNode('/sensors/imu', True)
        self.airdata_node = getNode('/sensors/airdata', True)
        self.pilot_node = getNode('/sensors/pilot', True)
        self.filter_node = getNode('/filters/filter', True)
        self.pos_combined_node = getNode('/position/combined', True)
        self.velocity_node = getNode('/velocity', True)
        self.act_node = getNode('/actuators/actuator', True)
        self.ap_node = getNode('/autopilot', True)
        self.targets_node = getNode('/autopilot/targets', True)
        self.route_node = getNode('/task/route', True)
        self.status_node = getNode('/status', True)
        self.cam_node = getNode('/payload/camera', True) 
        self.wind_node = getNode('/filters/wind', True) 
   
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
    
