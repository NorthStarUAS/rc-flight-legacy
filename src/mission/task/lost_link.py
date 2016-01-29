#import mission.mission_mgr
from props import root, getNode

import comms.events
from task import Task

class LostLink(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.status_node = getNode("/status", True)
        self.task_node = getNode("/task", True)
        self.remote_link_node = getNode("/comms/remote_link", True)
        self.remote_link_node.setString("link", "inactive")
        self.link_state = False
        self.push_task = ""
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.timeout_sec = config_node.getFloat("timeout_sec")
        if self.timeout_sec < 1.0:
            # set a sane default if none provided
            self.timetout_sec = 60.0
        self.action = config_node.getString("action")

    def activate(self):
        self.active = True
        comms.events.log("comms", "lost link monitor started")
    
    def update(self):
        if not self.active:
            return False
        
        # FIXME: this needs to be fleshed out a *lot* more in the future
        # with more flexible options.  FIXME: what about a sensible
        # fallback in case we can't find the push_task or other desired
        # actions?

        last_message_sec = self.remote_link_node.getFloat("last_message_sec")

        if last_message_sec > 0.0:
            current_time = self.status_node.getFloat("frame_time")
            message_age = current_time - last_message_sec
            # print "update lost link task, msg age = %.1f timeout=%.1f" % \
            #       (message_age, self.timeout_sec)
            if message_age > self.timeout_sec:
                # lost link state
                if self.link_state:
                    self.link_state = False
                    self.remote_link_node.setString("link", "lost")
                    comms.events.log("comms", "link timed out (lost) last_message=%.1f timeout_sec=%.1f" % (last_message_sec, self.timeout_sec))
                    # do lost link action here (iff airborne)
                    task = mission.mission_mgr.m.find_standby_task_by_nickname( action )
                    if ( task and self.task_node.getBool("is_airborne") ):
                        comms.events.log("comms", "action=" + task.name + "(" + action + ")")
                        # activate task
                        mission.mission_mgr.m.push_seq_task( task )
                        task.activate()
                        self.push_task = action
            else:
                # good link state
                if not self.link_state:
                    self.link_state = True
                    self.remote_link_node.setString("link", "ok")
                    comms.events.log("comms", "link ok")
                    # do resume link action here now.  We don't care
                    # if we are airborne when undoing the action.
                    if self.push_task != "":
                        # we've pushed something on the task queue
                        task = mission.mission_mgr.m.front_seq_task()
                        if task:
                            if task.nickname == action:
                                # pop the front task, but only if we
                                # were the ones that pushed on on
                                task.close()
                                mission.mission_mgr.m.pop_seq_task()

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True
