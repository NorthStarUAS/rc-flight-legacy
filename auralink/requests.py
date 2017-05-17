# generate requests for information we want but don't yet have

from props import root, getNode

import commands

ident_node = getNode('/config/identity', True)
power_node = getNode('/config/power', True)
speed_node = getNode('/config/speed', True)

requests_pending = True
def gen_requests():
    global requests_pending
    if not requests_pending:
        # we have everything we asked for
        return
    if commands.cmd_queue_empty():
        requests_pending = False # tentatively mark us as finished
        if ident_node.getString('call_sign') == '':
            requests_pending = True
            commands.add('get,/config/identity/call_sign')
        if ident_node.getString('make') == '':
            requests_pending = True
            commands.add('get,/config/identity/make')
        if ident_node.getString('model') == '':
            requests_pending = True
            commands.add('get,/config/identity/model')
        if ident_node.getString('serial_number') == '':
            requests_pending = True
            commands.add('get,/config/identity/serial_number')
            
        if power_node.getString('battery_cells') == '':
            requests_pending = True
            commands.add('get,/config/power/battery_cells')
        if power_node.getString('battery_mah') == '':
            requests_pending = True
            commands.add('get,/config/power/battery_mah')
            
        if speed_node.getString('cruise_kt') == '':
            requests_pending = True
            commands.add('get,/config/speed/cruise_kt')
        if speed_node.getString('max_kt') == '':
            requests_pending = True
            commands.add('get,/config/speed/max_kt')
        if speed_node.getString('min_kt') == '':
            requests_pending = True
            commands.add('get,/config/speed/min_kt')
