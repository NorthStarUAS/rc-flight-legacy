# generate requests for information we want but don't yet have

from props import root, getNode

import commands

ident_node = getNode('/config/identity', True)
specs_node = getNode('/config/specs', True)

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
            
        if specs_node.getString('battery_cells') == '':
            requests_pending = True
            commands.add('get,/config/specs/battery_cells')
        if specs_node.getString('battery_mah') == '':
            requests_pending = True
            commands.add('get,/config/specs/battery_mah')
            
        if specs_node.getString('cruise_kt') == '':
            requests_pending = True
            commands.add('get,/config/specs/cruise_kt')
        if specs_node.getString('max_kt') == '':
            requests_pending = True
            commands.add('get,/config/specs/max_kt')
        if specs_node.getString('min_kt') == '':
            requests_pending = True
            commands.add('get,/config/specs/min_kt')
