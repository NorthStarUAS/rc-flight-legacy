from props import root, getNode

import comms.events
from task import Task
import mission_mgr

class LostLink(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.status_node = getNode("/status", True)
        self.task_node = getNode("/task", True)
        self.remote_link_node = getNode("/comms/remote_link", True)
        self.link_state = False
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")
        self.timeout_sec = config_node.getFloat("timeout_sec")
        if self.timeout_sec < 1.0:
            # set a sane default if none provided
            self.timetout_sec = 60.0
        self.action = config_node.getString("action")

    def activate(self):
        self.active = True
        comms.events.log("mission", "On Ground")
    
    def update(self):
        if not self.active:
            return False
        
        # FIXME: this needs to be fleshed out a *lot* more in the future
        # with more flexible options.  FIXME: what about a sensible
        # fallback in case we can't find the push_task or other desired
        # actions?

        current_time = self.status_node.getFloat("frame_time")
        last_message_sec = remote_link_node.getDouble("last_message_sec")
        message_age = current_time - last_message_sec
        print "update lost link task, msg age = %.1f timeout=%.1f" % (message_age, timeout_sec)

        if last_message_sec > 0.0 and message_age > self.timeout_sec:
            # lost link state
            if self.link_state:
                self.link_state = False
                comms.events.log("comms", "link timed out (lost) last_message=%.1f timeout_sec=%.1f" % (last_message_sec, self.timeout_sec))
                # do lost link action here (iff airborne)
                do_action =  task_node.getBool("is_airborne")
                task = mission_mgr.find_standby_task_by_nickname( action )
                if ( task and task_node.getBool("is_airborne") ):
                    comms.events.log("comms", "action=" + task.name + "(" + action + ")")
                    # activate task
                    mission_mgr.m.push_seq_task( task )
                    taskactivate()
        else:
            # good link state (or we never had a link)
            if not self.link_state:
                self.link_state = True
                comms.events.log("comms", "link resumed")
                # do resume link action here now.  We don't care if we
                # are airborne or not, we just care if our "push task" is
                # at the front of the sequential queue or not (even if we
                # are on the ground now, we could have been airborne when
                # the link was lost.)
                task = mission_mgr.front_seq_task()
                if task:
                    if task.nickname == action:
                        task.close()
                        mission_mgr.pop_seq_task()
    
    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True
