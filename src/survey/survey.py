import math

from props import getNode

import control.route
import control.waypoint

import survey.point as point
import survey.vector as vector
import survey.area as area

ft2m = 0.3048
d2r = math.pi / 180.0

wind_node = getNode("/filters/wind", True)
task_node = getNode( '/task', True )
targets_node = getNode( '/autopilot/targets', True )

def do_survey( request ):
    # validate the inputs
    print('do survey:', request)
    if 'agl_ft' in request:
        agl_ft = request['agl_ft']
    else:
        agl_ft = 200
    if 'extend_m' in request:
        extend_m = request['extend_m']
    else:
        extend_m = 50
    if 'overlap_perc' in request:
        flap = request['overlap_perc']
    else:
        flap = 0.7
    if 'sidelap_perc' in request:
        slap = request['sidelap_perc']
    else:
        slap = 0.7
    if 'forward_fov' in request:
        ffov = request['forward_fov']
    else:
        ffov = 60
    if 'lateral_fov' in request:
        lfov = request['lateral_fov']
    else:
        lfov = 40

    # generate the boundary polygon
    poly = []
    if not 'area' in request:
        return    
    for p in request['area']:
        poly.append( point.Point(p[0], p[1]) )
    ref = poly[0]
    cart = area.geod2cart( ref, poly )
    cart_area = area.Area( cart )

    # convert to cartesian coordinates
    # advance direction is upwind by default
    if wind_node.getFloat('wind_speed_kt') > 2:
        advance_dir = vector.Vector( wind_node.getFloat('wind_east_mps'),
                                     wind_node.getFloat('wind_north_mps') )
    else:
        # little or no wind
        advance_dir = vector.Vector(0, 1) # north

    # survey altitude
    if agl_ft >= 100 and agl_ft <= 400:
        # autoset target altitude if the value is reasonably sane
        # (otherwise leave it up to the operator to set the altitude
        # from the gcs)
        targets_node.setFloat( 'altitude_agl_ft', agl_ft )
    agl_m = agl_ft * ft2m

    # compute sidelap step in m
    fov2_tan = math.tan(lfov*0.5 * d2r)
    slap_dist_m = 2 * fov2_tan * agl_m * (1.0 - slap)

    # slice the area and make the route
    cart_route = area.slice(cart_area, advance_dir, step=slap_dist_m,
                            extend=extend_m)
    geod_route = area.cart2geod( ref, cart_route )

    # assemble the route
    control.route.standby_route = []
    for p in geod_route:
        wp = control.waypoint.Waypoint()
        wp.mode = 'absolute'
        wp.lon_deg = p.x
        wp.lat_deg = p.y
        control.route.standby_route.append(wp)

    # make active
    control.route.swap()

    # request a task change
    task_node.setString( 'command', 'route' )

    # dribble
    control.route.dribble(reset=True)
