from props import root, getNode

import comms.events

import task.is_airborne
import task.circle
import taks.flaps_mgr
import task.home_mgr
import task.idle
import task.land
import task.launch
import task.lost_link
import task.preflight
import task.recalibrate
import task.route
import task.throttle_safety

import mission.greatcircle

class MissionMgr:
    def __init__(self):
        self.targets_node = getNode("/autopilot/targets", True)
        self.missions_node = getNode("/config/mission", True)
        self.pos_node = getNode("/position", True)
        self.task_node = getNode("/task", True)
        self.circle_node = getNode("/task/circle", True)
        self.home_node = getNode("/task/home", True)
        self.wind_node = getNode("/filters/wind", True)
        self.global_tasks = []
        self.seq_tasks = []
        self.standby_tasks = []
        
    def make_task(self, config_node):
        result = None
        task_name = config_node.name
        print "  make_task():", task_name
        if task_name == 'is_airborne':
            result = task.is_airborne.IsAirborne(config_node)
        elif task_name == 'circle':
            result = task.circle.Circle(config_node)
        elif task_name == 'flaps_manager':
            result = task.flaps_mgr.FlapsMgr(config_node)
        elif task_name == 'home_manager':
            result = task.home_mgr.HomeMgr(config_node)
        elif task_name == 'idle':
            result = task.idle.Idle(config_node)
        elif task_name == 'land':
            result = task.land.Land(config_node)
        elif task_name == 'launch':
            result = task.launch.Launch(config_node)
        elif task_name == 'lost_link':
            result = task.lost_link.LostLink(config_node)
        elif task_name == 'preflight':
            result = task.preflight.Preflight(config_node)
        elif task_name == 'recalibrate':
            result = task.recalibrate.Recalibrate(config_node)
        elif task_name == 'route':
            result = task.route.Route(config_node)
        elif task_name == 'throttle_safety':
            result = task.throttle_safety.ThrottleSafety(config_node)
        else:
            print "mission_mgr: unknown task name:", task_name
        return result
    
    def init(self):
        print "global_tasks:"
        global_node = self.missions_node.getChild("global_tasks", True)
        for name in global_node.getChildren():
            config_node = global_node.getChild(name)
            task = self.make_task(config_node)
            if task != None:
                self.global_tasks.append( task )
            
        print "sequential_tasks:"
        seq_node = self.missions_node.getChild("sequential_tasks", True)
        for name in seq_node.getChildren():
            config_node = seq_node.getChild(name)
            task = self.make_task(config_node)
            if task != None:
                self.seq_tasks.append( task )

        print "standby_tasks:"
        standby_node = self.missions_node.getChild("standby_tasks", True)
        for name in standby_node.getChildren():
            config_node = standby_node.getChild(name)
            task = self.make_task(config_node)
            if task != None:
                self.standby_tasks.append( task )

        # activate all the tasks in the global queue
        for task in self.global_tasks:
            task.activate()
            
        # activate the first task on the sequential queue
        if len(self.seq_tasks):
            self.seq_tasks[0].activate()

    def update(self):
        self.process_command_request()

        # run all tasks in the global queue
        for task in self.global_tasks:
            task.update()

        if len(self.seq_tasks):
            # run the first task in the sequential queue
            task = self.seq_tasks[0]
            self.task_node.setString("current_task_id", task.name)
            task.update()
            if task.is_complete():
	        # current task is complete, close it and pop it off the list
		comms.events.log("mission", "task complete: " + task.name)
                # FIXME
	        # if ( display_on ) {
		#     printf("task complete: %s\n", front->get_name_cstr())
	        # }
                task.close()
	        self.pop_seq_task()

	        # activate next task if there is one
                if len(self.seq_tasks):
	            task = self.seq_tasks[0]
		    task.activate()
		    comms.events.log("mission", "next task: " + task.name)
        if not len(self.seq_tasks):
	    # sequential queue is empty so request the idle task
	    self.request_task_idle()
        return True

    def find_global_task(self, name):
        for task in self.global_tasks:
            if task.name == name:
                return task
        return None
    
    def front_seq_task(self):
        if len(self.seq_tasks):
            return self.seq_tasks[0]
        else:
            return None

    def push_seq_task(self, task):
	self.seq_tasks.insert(0, task)

    def pop_seq_task(self):
        if len(self.seq_tasks):
            self.seq_tasks.pop(0)

    def find_seq_task(self, name):
        for task in self.seq_tasks:
            if task.name == name:
                return task
        return None
    
    def find_standby_task(self, name):
        for task in self.standby_tasks:
            if task.name == name:
                return task
        return None
    
    def find_standby_task_by_nickname(self, nickname):
        for task in self.standby_tasks:
            if task.nickname == nickname:
                return task
        return None
    
    def process_command_request(self):
        command = self.task_node.getString("command_request")
        result = "successful: " + command # let's be optimistic!
        if len(command):
            tokens = command.split(",")
            # these commands 'push' a new task onto the front of the
            # sequential task queue (prioritizing over what was
            # previously happening.)  The 'resume' task will pop the
            # task off and resume the original task if it exists.
            if len(tokens) == 2 and tokens[1] == "home":
                self.request_task_home()
            elif len(tokens) == 2 and tokens[1] == "circle":
                self.request_task_circle()
            elif len(tokens) == 4 and tokens[1] == "circle":
                self.request_task_circle( tokens[2], tokens[3] )
            elif len(tokens) == 2 and tokens[1] == "idle":
                self.request_task_idle()
            elif len(tokens) == 2 and tokens[1] == "resume":
                self.request_task_resume()
            elif len(tokens) == 2 and tokens[1] == "land":
                wind_deg = self.wind_node.getFloat("wind_dir_deg")
                self.request_task_land(wind_deg)
            elif len(tokens) == 3 and tokens[1] == "land":
                wind_deg = float(tokens[2])
                self.request_task_land(wind_deg)
            elif len(tokens) == 2 and tokens[1] == "preflight":
                self.request_task_preflight()
            elif len(tokens) == 2 and tokens[1] == "recalibrate":
                self.request_task_recalibrate()
            elif len(tokens) == 2 and tokens[1] == "route":
                self.request_task_route()
            else:
                result = "syntax error: " + command # bummer
            self.task_node.setString("command_request", "")
            self.task_node.setString("command_result", result)

    def request_task_home(self):
        nickname = "circle_home"
        task = None

        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.nickname == nickname:
	        return

        task = self.find_standby_task_by_nickname(nickname)
        if task:
	    # activate task
            self.push_seq_task(task)
	    task.activate()
        # FIXME: elif display_on:
	#    print "oops, couldn't find 'circle-home' task"


    def request_task_circle(self, lon_deg=None, lat_deg=None):
        lon = 0.0
        lat = 0.0
        if lon_deg == None or lat_deg == None:
            # no coordinates specified, use current position
            lon = self.pos_node.getFloat("longitude_deg")
            lat = self.pos_node.getFloat("latitude_deg")
        else:
            lon = float(lon_deg)
            lat = float(lat_deg)

        nickname = "circle_target"
        task = None

        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.nickname != nickname:
                task = self.find_standby_task_by_nickname( nickname )
                if task:
                    # activate task
                    self.push_seq_task(task)
                    task.activate()
            
        # setup the target coordinates
        self.circle_node.setFloat( "longitude_deg", lon )
        self.circle_node.setFloat( "latitude_deg", lat )

        # FIXME else if display_on:
        #    print "oops, couldn't find task by nickname:", nickname

        
    def request_task_circle_descent(self, lon_deg, lat_deg,
                                    radius_m, direction,
                                    exit_agl_ft, exit_heading_deg):
        lon = 0.0
        lat = 0.0
        if lon_deg == None or lat_deg == None:
            # no coordinates specified, use current position
            lon = self.pos_node.getFloat("longitude_deg")
            lat = self.pos_node.getFloat("latitude_deg")
        else:
            lon = float(lon_deg)
            lat = float(lat_deg)

        nickname = "circle_descent"
        task = None

        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.nickname != nickname:
                task = self.find_standby_task_by_nickname( nickname )
                if task:
                    # activate task
                    self.push_seq_task(task)
                    task.activate()
            
                    # setup the target coordinates
                    self.circle_node.setFloat( "longitude_deg", lon )
                    self.circle_node.setFloat( "latitude_deg", lat )

                    # circle configuration
                    self.circle_node.setFloat("radius_m", radius_m)
                    self.circle_node.setString("direction", direction)
        
                    # set the exit condition settings
                    task.exit_agl_ft = exit_agl_ft
                    task.exit_heading_deg = exit_heading_deg

        # FIXME else if display_on:
        #    print "oops, couldn't find task by nickname:", nickname

    def request_task_idle(self):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "idle":
                return

        task = self.find_standby_task( "idle" )
        if task:
            # activate task
            self.push_seq_task(task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'idle' task"

    def request_task_resume(self):
        # look for any 'circle-coord' task at the front of the sequential
        # queue and remove it if it exists
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "circle-coord" or task.name == "land":
                task.close()
	        self.pop_seq_task()

    def request_task_preflight(self):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "preflight":
                return
        task = self.find_standby_task( "preflight" )
        if task:
            # activate task
            self.push_seq_task(task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'preflight' task"

    def request_task_recalibrate(self):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "recalibrate":
                return
        task = self.find_standby_task("recalibrate")
        if task:
            # activate task
            self.push_seq_task(task)
	    task.activate()
        else:
            # FIXME else if display_on:
            print "oops, couldn't find 'recalibrate' task"

    def request_task_land(self, final_heading_deg):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "land":
                return
        task = self.find_standby_task( "land" )
        if not task:
            # FIXME if display_on:
            #     print "oops, couldn't find 'land' task"
            return
        # push landing task onto the todo list (and activate)
        self.home_node.setFloat( "azimuth_deg", final_heading_deg )
        self.push_seq_task(task)
	task.activate()

    def request_task_route(self):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "route":
                return
        task = self.find_standby_task( "route" )
        if task:
            # activate task
            self.push_seq_task(task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'route' task"

m = MissionMgr()

def init():
    m.init()

def update():
    m.update()
