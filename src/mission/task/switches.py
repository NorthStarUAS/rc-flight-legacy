# This task monitors pilot inputs and maps RC channel inputs to specificly
# configured switch states.
#
# This allows a configurable mapping of RC channels to property values.
# It is likely that common conventions emerge, but this can be configured
# per aircraft.

import re

from props import root, getNode

import comms.events
from task import Task

class Switch():
    def __init__(self, switch_node):
        print "switch: ", switch_node.getChildren(expand=True)
        self.valid = True
        if switch_node.hasChild("input_prop"):
            prop_name = switch_node.getString("input_prop")
            tmp = prop_name.split('/')
            input_path = '/'.join(tmp[0:-1])
            self.input_node = getNode(input_path, True)
            self.input_name = tmp[-1]
            print "input_path:", input_path
            print "input_name:", self.input_name
        else:
            self.valid = False
        if switch_node.hasChild("output_prop"):
            prop_name = switch_node.getString("output_prop")
            tmp = prop_name.split('/')
            output_path = '/'.join(tmp[0:-1])
            self.output_node = getNode(output_path, True)
            self.output_name = tmp[-1]
        else:
            self.valid = False
        if switch_node.hasChild("output_type"):
            self.output_type = switch_node.getString("output_type")
            self.states = 2
        else:
            self.output_type = 'boolean'
            self.states = 2
        self.choices = []
        if self.output_type == 'choice':
            self.states = switch_node.getLen('choice')
            #print 'found:', self.states, 'choices'
            if self.states > 0:
                for i in range(self.states):
                    choice = switch_node.getStringEnum('choice', i)
                    #print ' choice:', choice
                    self.choices.append(choice)
            else:
                self.states = 1
                self.choices = [ 'switch_config_error' ]
        self.min = -1.0
        self.max = 1.0
        self.range = self.max - self.min
        self.step = self.range / self.states

    def update(self):
        if not self.valid:
            return

        #print "switch update:"
        input_val = self.input_node.getFloat(self.input_name)
        if input_val < self.min: input_val = self.min
        if input_val > self.max: input_val = self.max
        #print "  range:", self.min, self.max
        #print "  input_val =", input_val
        #print "  input_range =", self.input_range
        #print "  input_name =", self.input_name
        #print "  step =", self.step
        #print "  input children:", self.input_node.getChildren()
        state = int((input_val - self.min) / self.step)
        #print "  state =", state
        if self.output_type == 'boolean':
            self.output_node.setBool(self.output_name, state)
        elif self.output_type == 'choice':
            #print 'choice:', state, self.choices[state]
            self.output_node.setString(self.output_name, self.choices[state])
            
class Switches(Task):
    def __init__(self, config_node):
        Task.__init__(self)

        self.switches = []
        
        self.name = config_node.getString("name")
        self.nickname = config_node.getString("nickname")

        children = config_node.getChildren(expand=True)
        #print "switches task children:", children
        for child_name in children:
            if re.match("switch", child_name):
                switch = Switch(config_node.getChild(child_name))
                #print switch.__dict__
                self.switches.append(switch)        

    def activate(self):
        self.active = True
    
    def update(self):
        if not self.active:
            return False
        for i, switch in enumerate(self.switches):
            switch.update()

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True
