# lost_link.py: monitors messages from the ground station and flags an
# alert when too much time has elapsed since the last received
# message.  If airborne at this time, request the circle home task
# which sends the aircraft home to circle.  The operator much chose
# the next course of action after link is successfully resumed.
# Otherwise the assumption is that once returned home and circling,
# the operator could take over manual control and land the aircraft
# manually.

import mission.mission_mgr
from props import getNode

import comms.events
from mission.task.task import Task

class LostLink(Task):
    def __init__(self, config_node):
        Task.__init__(self)
        self.status_node = getNode("/status", True)
        self.task_node = getNode("/task", True)
        self.home_node = getNode("/task/home", True)
        self.targets_node = getNode("/autopilot/targets", True)
        self.remote_link_node = getNode("/comms/remote_link", True)
        self.remote_link_node.setString("link", "inactive")
        self.link_state = False
        self.push_task = ""
        self.name = config_node.getString("name")
        self.timeout_sec = config_node.getFloat("timeout_sec")
        if self.timeout_sec < 1.0:
            # set a sane default if none provided
            self.timetout_sec = 60.0

    def activate(self):
        self.active = True
        comms.events.log("comms", "lost link monitor started")
    
    def update(self, dt):
        if not self.active:
            return
        
        # FIXME: this needs to be fleshed out a *lot* more in the future
        # with more flexible options.  FIXME: what about a sensible
        # fallback in case we can't find the push_task or other desired
        # actions?

        last_message_sec = self.remote_link_node.getFloat("last_message_sec")
        if last_message_sec < 0.00001:
            # likely zero, likely never received a message from GCS yet
            return
        
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
                if self.task_node.getBool("is_airborne"):
                    comms.events.log("lost_link", "circle home")
                    mission.mission_mgr.m.request_task_home()
                    # sanity check on transit altitude (boost to 200'
                    # agl if below that)
                    target_agl = self.targets_node.getFloat("altitude_agl_ft")
                    if target_agl < 200.0:
                        self.targets_node.setFloat("altitude_agl_ft", 200.0)
        else:
            # good link state
            if not self.link_state:
                self.link_state = True
                self.remote_link_node.setString("link", "ok")
                comms.events.log("comms", "link ok")

                # Note: don't take any action when/if link resumes
                # (simply continue with circle home task). Operator
                # decision/action required to for next steps.

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True
