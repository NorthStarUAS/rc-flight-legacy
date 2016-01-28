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
        print "hello from MissionMgr.update()"
        

m = MissionMgr()

def init():
    m.init()

def update():
    m.update()
