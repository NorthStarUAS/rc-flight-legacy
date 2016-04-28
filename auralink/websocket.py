# tornado based websocket server

import json
import tornado
import tornado.httpserver
import tornado.websocket

from props import root, getNode

import commands

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
                dict['gps_sats'] = "%d" % self.gps_node.getInt('satellites')
                dict['lost_link'] = "%d" % commands.remote_lost_link_predict()
                dict['control_mode'] = "%0f" % self.pilot_node.getFloatEnum('channel', 5)
                dict['ap_hdg'] = "%.1f" % self.targets_node.getFloat('groundtrack_deg')
                dict['airdata_climb'] = "%.2f" % (self.velocity_node.getFloat('pressure_vertical_speed_fps') * 60)
                dict['ap_climb'] = "%.2f" % 0.0
                dict['imu_ay'] = "%.2f" % self.imu_node.getFloat('ay_mps_sec')
                dict['imu_az'] = "%.2f" % self.imu_node.getFloat('az_mps_sec')
                dict['imu_r'] = "%.2f" % self.imu_node.getFloat('r_rad_sec')
                dict['filter_phi'] = "%.2f" % self.filter_node.getFloat('roll_deg')
                dict['filter_theta'] = "%.2f" % self.filter_node.getFloat('pitch_deg')
                dict['ap_altitude'] = "%.2f" % self.targets_node.getFloat('altitude_msl_ft')
                dict['ap_speed'] = "%.1f" % self.targets_node.getFloat('airspeed_kt')
                dict['pitot_scale'] = "%.1f" % self.wind_node.getFloat('pitot_scale_factor')
                dict['avionics_vcc'] = "%.2f" % self.apm2_node.getFloat('board_vcc')
                dict['main_volts'] = "%.2f" % self.apm2_node.getFloat('extern_volt')
                dict['cell_volts'] = "%.2f" % self.apm2_node.getFloat('extern_cell_volt')
                dict['main_amps'] = "%.2f" % self.apm2_node.getFloat('extern_amps')
                dict['main_mah'] = "%.2f" % self.apm2_node.getFloat('extern_current_mah')
                dict['flight_timer'] = "%.1f" % 0.0 # fixme
                dict['airdata_temp'] = "%.1f" % self.airdata_node.getFloat('temp_degC')
                dict['camera_trigger'] = "%d" % self.cam_node.getInt('trigger-num')
                # print json.dumps(dict)
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
        self.apm2_node = getNode('/sensors/APM2', True) 
   
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
    
