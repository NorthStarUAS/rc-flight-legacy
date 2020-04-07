# This task monitors pilot inputs and maps RC channel inputs to specificly
# configured switch states.
#
# This allows a configurable mapping of RC channels to property values.
# It is likely that common conventions emerge, but this can be configured
# per aircraft.
#
# Switch types:
#
# boolean: output property is set to true or false based on switch position

# choice: one (and only one) of the defined set of output
#            properties is set to true based on switch position.
#            Binary and trinary switches are supported.  4 or more
#            position switches have not been tested

# enumerate: similar to choice, but with only a single output property
#            which is set to one of the defined string values based on
#            switch position.  


import os
import re

from props import getNode

import comms.events
from mission.task.task import Task

class Switch():
    def __init__(self, switch_node):
        print("switch:")
        self.valid = True
        self.output_type = 'boolean'
        self.states = 2
        
        if switch_node.hasChild("type"):
            self.output_type = switch_node.getString("type")
            
        if switch_node.hasChild("input"):
            prop_name = switch_node.getString("input")
            (input_path, self.input_name) = os.path.split(prop_name)
            self.input_node = getNode(input_path, True)
            print("  input_path:", prop_name)
        else:
            self.valid = False

        if switch_node.hasChild("output"):
            prop_name = switch_node.getString("output")
            (output_path, self.output_name) = os.path.split(prop_name)
            self.output_node = getNode(output_path, True)
            print("  output_path:", prop_name)
        elif self.output_type == 'choice':
            # ok with no output prop
            pass
        else:
            self.valid = False
            
        self.choice_nodes = []
        self.choice_keys = []
        if self.output_type == 'choice':
            self.states = switch_node.getLen('outputs')
            print('  switch choice, found:', self.states, 'choices')
            if self.states > 0:
                for i in range(self.states):
                    output = switch_node.getStringEnum('outputs', i)
                    print('    output:', output)
                    (node_name, key_name) = os.path.split(output)
                    self.choice_nodes.append( getNode(node_name, True) )
                    self.choice_keys.append( key_name )
            else:
                self.states = 1
                self.choice_nodes.append( getNode("autopilot", True) )
                self.choice_keys.append( 'switch_config_error' )
                
        self.enums = []
        if self.output_type == 'enumerate':
            self.states = switch_node.getLen('enumerate')
            #print 'found:', self.states, 'enums'
            if self.states > 0:
                for i in range(self.states):
                    enum = switch_node.getStringEnum('enumerate', i)
                    #print ' enum:', enum
                    self.enums.append(enum)
            else:
                self.states = 1
                self.choices = [ 'switch_config_error' ]
                
        if self.output_type == 'boolean' and switch_node.hasChild("force_value"):
            self.force_value = switch_node.getBool("force_value")
        else:
            self.force_value = None
            
        if switch_node.hasChild("invert"):
            self.invert = switch_node.getBool("invert")
        else:
            self.invert = False
            
        self.min = -1.0
        self.max = 1.0
        self.range = self.max - self.min
        self.step = self.range / self.states

    def update(self, dt):
        if not self.valid:
            return

        #print "switch update:"
        input_val = self.input_node.getFloat(self.input_name)
        if self.invert:
            input_val = -input_val
        if input_val < self.min: input_val = self.min
        if input_val > self.max: input_val = self.max
        #print "  range:", self.min, self.max
        #print "  input_val =", input_val
        #print "  input_range =", self.input_range
        #print "  input_name =", self.input_name
        #print "  step =", self.step
        #print "  input children:", self.input_node.getChildren()
        if self.states == 2:
            if input_val < -0.1:
                state = 0
            else:
                state = 1
        else:
            state = int((input_val - self.min) / self.step)
            if state >= self.states:
                state = self.states - 1
        #print "  state =", state
            
        if self.output_type == 'boolean':
            if self.force_value != None:
                self.output_node.setBool(self.output_name, self.force_value)
            else:
                self.output_node.setBool(self.output_name, state)
        elif self.output_type == 'choice':
            for i in range(len(self.choice_nodes)):
                #print 'choice', i, state, self.choice_keys[i]
                if i == state:
                    self.choice_nodes[i].setBool(self.choice_keys[i], True)
                else:
                    self.choice_nodes[i].setBool(self.choice_keys[i], False)
        elif self.output_type == 'enumerate':
            #print 'choice:', state, self.choices[state]
            self.output_node.setString(self.output_name, self.enums[state])
            
class Switches(Task):
    def __init__(self, config_node):
        Task.__init__(self)

        self.switches = []
        
        self.name = config_node.getString("name")

        children = config_node.getChildren(expand=True)
        #print "switches task children:", children
        for child_name in children:
            if re.match("switch", child_name):
                switch = Switch(config_node.getChild(child_name))
                #print switch.__dict__
                self.switches.append(switch)        

    def activate(self):
        self.active = True
    
    def update(self, dt):
        if not self.active:
            return False
        for i, switch in enumerate(self.switches):
            if switch != None:
                switch.update(dt)

    def is_complete(self):
        return False
    
    def close(self):
        self.active = False
        return True
