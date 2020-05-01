from props import getNode

import comms.events

from mission.task import calib_accels
from mission.task import calib_home
from mission.task import calib_mags
from mission.task import camera
from mission.task import circle
from mission.task import excite
from mission.task import flaps_mgr
from mission.task import home_mgr
from mission.task import idle
from mission.task import is_airborne
from mission.task import land3
from mission.task import launch
from mission.task import lost_link
from mission.task import mode_mgr
from mission.task import parametric
from mission.task import preflight
from mission.task import route
from mission.task import switches
from mission.task import throttle_safety

class MissionMgr:
    def __init__(self):
        self.targets_node = getNode("/autopilot/targets", True)
        self.missions_node = getNode("/config/mission", True)
        self.pos_node = getNode("/position", True)
        self.task_node = getNode("/task", True)
        self.preflight_node = getNode("/task/preflight", True)
        self.circle_standby_node = getNode("/task/circle/standby", True)
        self.home_node = getNode("/task/home", True)
        self.wind_node = getNode("/filters/wind", True)
        self.global_tasks = []
        self.seq_tasks = []
        self.standby_tasks = []

    def make_task(self, config_node):
        if not config_node.hasChild("name"):
            return None
        result = None
        task_name = config_node.getString("name")
        print("  make_task(): '%s'" % task_name)
        if task_name == "calib_accels":
            result = calib_accels.CalibrateAccels(config_node)
        elif task_name == "calib_home":
            result = calib_home.Calibrate(config_node)
        elif task_name == "calib_mags":
            result = calib_mags.CalibrateMagnetometer(config_node)
        elif task_name == "camera":
            result = camera.Camera(config_node)
        elif task_name == "circle":
            result = circle.Circle(config_node)
        elif task_name == "excite":
            result = excite.Excite(config_node)
        elif task_name == "flaps_manager":
            result = flaps_mgr.FlapsMgr(config_node)
        elif task_name == "home_manager":
            result = home_mgr.HomeMgr(config_node)
        elif task_name == "idle":
            result = idle.Idle(config_node)
        elif task_name == "is_airborne":
            result = is_airborne.IsAirborne(config_node)
        elif task_name == "land":
            result = land3.Land(config_node)
        elif task_name == "launch":
            result = launch.Launch(config_node)
        elif task_name == "lost_link":
            result = lost_link.LostLink(config_node)
        elif task_name == "mode_manager":
            result = mode_mgr.ModeMgr(config_node)
        elif task_name == "parametric":
            result = parametric.Parametric(config_node)
        elif task_name == "preflight":
            result = preflight.Preflight(config_node)
        elif task_name == "route":
            result = route.Route(config_node)
        elif task_name == "switches":
            result = switches.Switches(config_node)
        elif task_name == "throttle_safety":
            result = throttle_safety.ThrottleSafety(config_node)
        else:
            print("mission_mgr: unknown task name:", task_name)
        return result

    def init(self):
        print("global_tasks:")
        global_node = self.missions_node.getChild("global_tasks", True)
        if global_node:
            for name in global_node.getChildren():
                config_node = global_node.getChild(name)
                task = self.make_task(config_node)
                if task != None:
                    self.global_tasks.append( task )

        print("sequential_tasks:")
        seq_node = self.missions_node.getChild("sequential_tasks", True)
        if seq_node:
            for name in seq_node.getChildren():
                config_node = seq_node.getChild(name)
                task = self.make_task(config_node)
                if task != None:
                    self.seq_tasks.append( task )

        print("standby_tasks:")
        standby_node = self.missions_node.getChild("standby_tasks", True)
        if standby_node:
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

    def update(self, dt):
        self.process_command()

        # run all tasks in the global queue
        for task in self.global_tasks:
            task.update(dt)

        if len(self.seq_tasks):
            # run the first task in the sequential queue
            task = self.seq_tasks[0]
            self.task_node.setString("current_task", task.name)
            task.update(dt)
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
            self.request_task("idle")
        return True

    def find_global_task(self, name):
        for task in self.global_tasks:
            if task.name == name:
                return task
        comms.events.log("mission", "global task not found: " + name)
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
            task = self.seq_tasks.pop(0)
            task.close()

    def find_seq_task(self, name):
        for task in self.seq_tasks:
            if task.name == name:
                return task
        comms.events.log("mission", "sequential task not found: " + name)
        return None

    def find_standby_task(self, name):
        if name != "":
            for task in self.standby_tasks:
                if task.name == name:
                    return task
        comms.events.log("mission", "standby task not found: " + name)
        return None

    def process_command(self):
        command = self.task_node.getString("command")
        result = "successful: " + command # let's be optimistic!
        if len(command):
            tokens = command.split(",")
            # these commands 'push' a new task onto the front of the
            # sequential task queue (prioritizing over what was
            # previously happening.)  The 'resume' task will pop the
            # task off and resume the original task if it exists.
            if len(tokens) == 1 and tokens[0] == "circle":
                self.request_task_circle()
            elif len(tokens) == 3 and tokens[0] == "circle":
                self.request_task_circle( tokens[1], tokens[2] )
            elif len(tokens) == 1 and tokens[0] == "home":
                self.request_task_home()
            elif len(tokens) == 1 and tokens[0] == "resume":
                self.request_task_resume()
            elif len(tokens) == 1 and tokens[0] == "land":
                hdg_deg = self.wind_node.getFloat("wind_dir_deg")
                self.request_task_land(hdg_deg)
            elif len(tokens) == 2 and tokens[0] == "land":
                hdg_deg = float(tokens[1])
                self.request_task_land(hdg_deg)
            elif len(tokens) == 1 and tokens[0] == "pop":
                self.pop_seq_task()
            elif len(tokens) == 2 and tokens[1] == "preflight":
                self.request_task_preflight(tokens[1])
            elif len(tokens) == 1:
                # catch all simple tasks with no parameters or extra
                # setup logic
                result = self.request_task(tokens[0])
            else:
                result = "syntax error: " + command # bummer
            self.task_node.setString("command", "")
            self.task_node.setString("command_result", result)

    def request_task(self, name):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == name:
                return "task already running: " + name
        task = self.find_standby_task(name)
        if task:
            # activate task
            self.push_seq_task(task)
            task.activate()
            return "successful: " + name
        else:
            comms.events.log("mission", "cannot find requested standby task: " + name)
            return "task not found: " + name

    # lookup the home location and request a circle task at that
    # point.  There should always be a home location defined, but if
    # not, the fallback is to circle the current position.
    def request_task_home(self):
        lon_deg = None
        lat_deg = None
        if self.home_node.hasChild("longitude_deg"):
            tmp = self.home_node.getFloat("longitude_deg")
            if abs(tmp) > 0.01:
                lon_deg = tmp
        if self.home_node.hasChild("latitude_deg"):
            tmp = self.home_node.getFloat("latitude_deg")
            if abs(tmp) > 0.01:
                lat_deg = tmp
        self.request_task_circle(lon_deg, lat_deg)


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

        # setup the target coordinates
        self.circle_standby_node.setFloat( "longitude_deg", lon )
        self.circle_standby_node.setFloat( "latitude_deg", lat )
        
        # sanity check, are we already running the requested task
        if len(self.seq_tasks) and self.seq_tasks[0].name == "circle":
            self.seq_tasks[0].update_parameters()
            comms.events.log("mission", "updating circle parameters")
        else:
            # activate task
            task = self.find_standby_task( "circle" )
            if task:
                self.push_seq_task(task)
                task.activate()

    def request_task_resume(self):
        # look for any 'circle-coord' task at the front of the sequential
        # queue and remove it if it exists
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "circle-coord" or task.name == "land":
                task.close()
                self.pop_seq_task()

    def request_task_preflight(self, duration):
        # sanity check, are we already in the requested state
        if len(self.seq_tasks):
            task = self.seq_tasks[0]
            if task.name == "preflight":
                return
        self.preflight_node.setFloat("duration_sec", duration)
        task = self.find_standby_task( "preflight" )
        if task:
            # activate task
            self.push_seq_task(task)
            task.activate()
        # FIXME else if display_on:
        #    print "oops, couldn't find 'preflight' task"

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
            
m = MissionMgr()

def init():
    m.init()

def update(dt):
    m.update(dt)
