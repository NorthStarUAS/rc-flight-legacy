from props import root, getNode

import task.is_airborne

class MissionMgr:
    def __init__(self):
        self.missions_node = None
        self.global_tasks = []
        self.seq_tasks = []
        self.standby_tasks = []
        
    def init(self):
        print "hello from MissionMgr.init()"
        self.missions_node = getNode("/config/mission", True)
        self.build()
        # activate all global tasks
        # activate first sequential task

    def make_task(self, task_node):
        result = None
        task_name = task_node.name
        print "  make_task():", task_name
        if task_name == 'is_airborne':
            result = task.is_airborne.IsAirborne(task_node)
        else:
            print "mission_mgr: unknown task name:", task_name
        return result
    
    def build(self):
        print "global_tasks:"
        global_node = self.missions_node.getChild("global_tasks", True)
        for name in global_node.getChildren():
            task_node = global_node.getChild(name)
            task = self.make_task(task_node)
            if task != None:
                self.global_tasks.append( task )
            
        print "sequential_tasks:"
        seq_node = self.missions_node.getChild("sequential_tasks", True)
        for name in seq_node.getChildren():
            task_node = seq_node.getChild(name)
            task = self.make_task(task_node)
            if task != None:
                self.seq_tasks.append( task )

        print "standby_tasks:"
        standby_node = self.missions_node.getChild("standby_tasks", True)
        for name in standby_node.getChildren():
            task_node = standby_node.getChild(name)
            task = self.make_task(task_node)
            if task != None:
                self.standby_tasks.append( task )

        # activate all the tasks in the global queue
        for task in self.global_tasks:
            task.activate()
            
        # activate the first task on the sequential queue
        if len(self.seq_tasks):
            self.seq_tasks[0].activate()
            
    def update(self):
        # FIXME: self.process_command_requests()

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
            pass
	    # sequential queue is empty so request the idle task
	    # FIXME: self.request_task_idle()
        return True
        
m = MissionMgr()

def init():
    m.init()

def update():
    m.update()
