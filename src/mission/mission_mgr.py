from props import root, getNode

import task.is_airborne

class MissionMgr:
    def __init__(self):
        self.ap_node = getNode("/autopilot/settings", True)
        self.missions_node = getNode("/config/mission", True)
        self.pos_node = getNode("/position", True)
        self.task_node = getNode("/task", True)
        self.circle_node = getNode("/task/circle", True)
        self.home_node = getNode("/task/home", True)
        self.wind_node = getNode("/filters/wind-est", True)
        self.global_tasks = []
        self.seq_tasks = []
        self.standby_tasks = []
        
    def make_task(self, config_node):
        result = None
        task_name = config_node.name
        print "  make_task():", task_name
        if task_name == 'is_airborne':
            result = task.is_airborne.IsAirborne(config_node)
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
            root.tasks.current_task_id = task.name
            task.update()
            if task.is_complete():
	        # current task is complete, close it and pop it off the list
		comms.events.log("mission", "task complete:");
		comms.events.log("   task", task.name)

                # FIXME
	        # if ( display_on ) {
		#     printf("task complete: %s\n", front->get_name_cstr());
	        # }
                task.close()
	        self.seq_tasks.pop(0)

	        # activate next task if there is one
                if len(self.seq_tasks):
	            task = self.seq_tasks[0]
		    task.activate()
		    comms.events.log("mission", "next task:")
		    comms.events.log("   task", task.name)
        else:
	    # sequential queue is empty so request the idle task
	    self.request_task_idle()
        return True

    def process_command_request(self):
        command = self.task_node.getString("command_request");
        result = "successful: " + command; # let's be optimistic!
        if len(command):
            tokens = command.split(",");
            # these commands 'push' a new task onto the front of the
            # sequential task queue (prioritizing over what was
            # previously happening.)  The 'resume' task will pop the
            # task off and resume the original task if it exists.
            if len(tokens) == 2 and tokens[1] == "home":
                request_task_home()
            elif len(tokens) == 2 and tokens[1] == "circle":
                request_task_circle()
            elif len(tokens) == 4 and tokens[1] == "circle":
                request_task_circle( tokens[2], tokens[3] )
            elif len(tokens) == 2 and tokens[1] == "idle":
                request_task_idle()
            elif len(tokens) == 2 and tokens[1] == "resume":
                request_task_resume()
            elif len(tokens) == 2 and tokens[1] == "land":
                wind_deg = self.wind_node.getDouble("wind_dir_deg")
                request_task_land(wind_deg)
            elif len(tokens) == 3 and tokens[1] == "land":
                wind_deg = float(tokens[2])
                request_task_land(wind_deg)
            elif len(tokens) == 2 and tokens[1] == "preflight":
                request_task_preflight()
            elif len(tokens) == 2 and tokens[1] == "recalibrate":
                request_task_recalibrate()
            elif len(tokens) == 2 and tokens[1] == "route":
                request_task_route()
            else:
                result = "syntax error: " + command # bummer
            task_node.setString("command_request", "")
            task_node.setString("command_result", result)

    def request_task_home(self):
        nickname = "circle_home";
        task = None

        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.nickname == nickname:
	        return

        task = find_standby_task_by_nickname(nickname)
        if task:
	    # activate task
	    self.seq_tasks.insert(0, task)
	    task.activate()
        # FIXME: elif display_on:
	#    print "oops, couldn't find 'circle-home' task"


    def request_task_circle( lon_deg=None, lat_deg=None, offset_hdg_deg=0.0,
			     offset_dist_m=0.0 ):
        lon = 0.0
        lat = 0.0
        if lon_deg == None or lat_deg == None:
            # no coordinates specified, use current position
            lon = self.pos_node.getDouble("longitude_deg")
            lat = self.pos_node.getDouble("latitude_deg")
        else:
            lon = float(lon_deg)
            lat = float(lat_deg)

        nickname = "circle_target";
        task = None;

        # FIXME: offset heading/degree support
        # SGGeoc orig;
        # orig.setLongitudeDeg( lon_deg );
        # orig.setLatitudeDeg( lat_deg );

        # if ( offset_dist_m > 0.1 ) {
        #     double course = home_node.getDouble("azimuth_deg") + offset_hdg_deg;
        #     if ( course < 0.0 ) { course += 360.0; }
        #     if ( course > 360.0 ) { course -= 360.0; }
        #     course = 360.0 - course; // invert to make this routine happy

        #     SGGeoc result;
        #     SGGeodesy::advanceRadM( orig, course * SGD_DEGREES_TO_RADIANS,
        #                             offset_dist_m, result );
        #     orig = result;
        # }

        # setup the target coordinates
        self.circle_node.setDouble( "longitude_deg", lon_deg )
        self.circle_node.setDouble( "latitude_deg", lat_deg )

        # clear the exit condition settings
        self.circle_node.setString( "exit_agl_ft", "" );
        self.circle_node.setString( "exit_heading_deg", "" );

        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.nickname == nickname:
                return

        task = find_standby_task_by_nickname( nickname )
        if task:
            # activate task
	    self.seq_tasks.insert(0, task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find task by nickname:", nickname

        
    def request_task_circle_setup( radius_m, direction ):
        # assumes we are in a circling state, but check just in case...
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "circle-coord":
                task.radius_m = radius_m
                task.direction = direction

    def request_task_circle_set_exit_conditions( exit_agl_ft, exit_heading_deg):
        # setup the exit conditions (which the basic task request resets/clears)
        self.circle_node.setDouble( "exit_agl_ft", exit_agl_ft )
        self.circle_node.setDouble( "exit_heading_deg", exit_heading_deg )

        # set the autopilot to climb/descend to the requested
        # altitude.  (If the auotopilot is never commanded, presumably
        # we would never get to the exit condition.)
        self.ap_node.setDouble( "target_agl_ft", exit_agl_ft )

    def request_task_idle():
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "idle":
                return

        task = find_standby_task( "idle" )
        if task:
            # activate task
            self.seq_tasks.insert(0, task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'idle' task"


    def request_task_resume():
        # look for any 'circle-coord' task at the front of the sequential
        # queue and remove it if it exists
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "circle-coord" or task.name == "land":
                task.close()
	        self.seq_tasks.pop(0)

    def request_task_preflight():
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "preflight":
                return
        task = find_standby_task( "preflight" );
        if task:
            # activate task
            self.seq_tasks.insert(0, task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'preflight' task"

    def request_task_recalibrate():
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "recalibrate":
                return
        task = find_standby_task( "recalibrate" )
        if task:
            # activate task
            self.seq_tasks.insert(0, task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'recalibrate' task"

    def request_task_land( final_heading_deg ):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "land":
                return
        task = find_standby_task( "land" );
        if not task:
            # FIXME if display_on:
            #     print "oops, couldn't find 'land' task"
            return
        # push landing task onto the todo list (and activate)
        self.home_node.setDouble( "azimuth_deg", final_heading_deg )
        self.seq_tasks.insert(0, task)
	task.activate()

    def request_task_route:
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "route":
                return
        task = find_standby_task( "route" )
        if task:
            # activate task
            self.seq_tasks.insert(0, task)
	    task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'route' task"

m = MissionMgr()

def init():
    m.init()

def update():
    m.update()
