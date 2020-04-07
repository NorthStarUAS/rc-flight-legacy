# mode manager: translate high level 'mode' requests to the corresponding
# sets of enable flags

import re

from props import getNode

import comms.events
from mission.task.task import Task

           
class ModeMgr(Task):
    def __init__(self, config_node):
        Task.__init__(self)

        self.name = config_node.getString("name")

        self.ap_node = getNode("/autopilot", True)
        self.locks_node = getNode("/autopilot/locks", True)

        self.last_master_switch = False
        self.last_fcs_mode = ''
        
    def activate(self):
        self.active = True
    
    def update(self, dt):
        if not self.active:
            return False
        
        # handle master_switch changes
        master_switch = self.ap_node.getBool('master_switch')
        if master_switch != self.last_master_switch:
            if master_switch:
                comms.events.log('control', 'ap master switch: on (auto)')
            else:
                comms.events.log('control', 'ap master switch: off (manual)')
            self.last_master_switch = master_switch

        # high level mode name
        if master_switch:
            fcs_mode = self.ap_node.getString('mode')
        else:
            fcs_mode = 'manual'

        # manage detailed enable switches from high level mode
        if self.last_fcs_mode != fcs_mode:
            comms.events.log('control', 'mode change = ' + fcs_mode)
            if fcs_mode == '' or fcs_mode == 'manual':
                # unset all locks if no mode defined
                self.locks_node.setBool( 'roll', False )
                self.locks_node.setBool( 'yaw', False )
                self.locks_node.setBool( 'pitch', False )
                self.locks_node.setBool( 'tecs', False )
            elif fcs_mode == 'basic':
                # set lock modes for 'basic' inner loops only
                self.locks_node.setBool( 'roll', True )
                self.locks_node.setBool( 'yaw', True )
                self.locks_node.setBool( 'pitch', True )
                self.locks_node.setBool( 'tecs', False )
            elif fcs_mode == 'roll':
                # set lock modes for roll only
                self.locks_node.setBool( 'roll', True )
                self.locks_node.setBool( 'yaw', False )
                self.locks_node.setBool( 'pitch', False )
                self.locks_node.setBool( 'tecs', False )
            elif fcs_mode == 'roll+pitch':
                # set lock modes for roll and pitch
                self.locks_node.setBool( 'roll', True )
                self.locks_node.setBool( 'yaw', False )
                self.locks_node.setBool( 'pitch', True )
                self.locks_node.setBool( 'tecs', False )
            elif fcs_mode == 'basic+tecs':
                # set lock modes for 'basic' + alt hold + speed hold
                self.locks_node.setBool( 'roll', True )
                self.locks_node.setBool( 'yaw', True )
                self.locks_node.setBool( 'pitch', True )
                self.locks_node.setBool( 'tecs', True )
            self.last_fcs_mode = fcs_mode
                
    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True
