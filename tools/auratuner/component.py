import matplotlib.pyplot as plt
from PyQt4 import QtGui, QtCore
import subprocess
import time

import fetcher
import fgtelnet

class PlotFields():
    def __init__(self):
        self.fields = [
            "time",
            "/autopilot/targets/groundtrack_deg",
            "/autopilot/targets/roll_deg",
            "filter_hdg",
            "/orientation/roll_deg",
            "/controls/flight/aileron",
            "/autopilot/targets/airspeed_kt",
            "/autopilot/targets/pitch_deg",
            "/velocity/airspeed_smoothed_kt",
            "/orientation/pitch_deg",
            "/controls/flight/elevator",
            "/autopilot/targets/altitude_agl_ft",
            "/position/altitude_agl_ft",
            "/controls/engine/throttle",
        ]

    def get_index(self, field):
        for i,f in enumerate(self.fields):
            if f == field:
                return i
        return 1

class Component():
    def __init__(self, index, changefunc, host="localhost", port=6499, type="pid"):
        self.index = index
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.type = type
        self.container = self.make_page()
        self.fields = PlotFields()

    def onChange(self):
        self.changefunc()

    def make_page(self):
        toppage = QtGui.QFrame()
        toplayout = QtGui.QVBoxLayout()
        toppage.setLayout(toplayout)

        page = QtGui.QFrame()
        layout = QtGui.QFormLayout()
        page.setLayout( layout )
        toplayout.addWidget( page )

        self.edit_name = QtGui.QLineEdit()
        self.edit_name.setFixedWidth(350)
        self.edit_name.textChanged.connect(self.onChange)
        self.edit_desc = QtGui.QLineEdit()
        self.edit_desc.setFixedWidth(350)
        self.edit_desc.textChanged.connect(self.onChange)
        if self.type == "pid" or self.type == "vel":
            self.edit_input = QtGui.QLineEdit()
            self.edit_input.setFixedWidth(350)
            self.edit_input.textChanged.connect(self.onChange)
        elif self.type == "sum":
            self.edit_input = []
            for i in range(4):
                edit_input = QtGui.QLineEdit()
                edit_input.setFixedWidth(350)
                edit_input.textChanged.connect(self.onChange)
                self.edit_input.append(edit_input)
        self.edit_ref = QtGui.QLineEdit()
        self.edit_ref.setFixedWidth(350)
        self.edit_ref.textChanged.connect(self.onChange)
        self.edit_output = QtGui.QLineEdit()
        self.edit_output.setFixedWidth(350)
        self.edit_output.textChanged.connect(self.onChange)
        self.edit_Kp = QtGui.QLineEdit()
        self.edit_Kp.setFixedWidth(350)
        self.edit_Kp.textChanged.connect(self.onChange)

        if self.type == "pid":
            self.edit_Ti = QtGui.QLineEdit()
            self.edit_Ti.setFixedWidth(350)
            self.edit_Ti.textChanged.connect(self.onChange)
            self.edit_Td = QtGui.QLineEdit()
            self.edit_Td.setFixedWidth(350)
            self.edit_Td.textChanged.connect(self.onChange)
        elif self.type == "vel":
            self.edit_beta = QtGui.QLineEdit()
            self.edit_beta.setFixedWidth(350)
            self.edit_beta.textChanged.connect(self.onChange)
            self.edit_alpha = QtGui.QLineEdit()
            self.edit_alpha.setFixedWidth(350)
            self.edit_alpha.textChanged.connect(self.onChange)
            self.edit_gamma = QtGui.QLineEdit()
            self.edit_gamma.setFixedWidth(350)
            self.edit_gamma.textChanged.connect(self.onChange)
            self.edit_Ti = QtGui.QLineEdit()
            self.edit_Ti.setFixedWidth(350)
            self.edit_Ti.textChanged.connect(self.onChange)
            self.edit_Td = QtGui.QLineEdit()
            self.edit_Td.setFixedWidth(350)
            self.edit_Td.textChanged.connect(self.onChange)
 
        self.edit_min = QtGui.QLineEdit()
        self.edit_min.setFixedWidth(350)
        self.edit_min.textChanged.connect(self.onChange)
        self.edit_max = QtGui.QLineEdit()
        self.edit_max.setFixedWidth(350)
        self.edit_max.textChanged.connect(self.onChange)
        self.edit_trim = QtGui.QLineEdit()
        self.edit_trim.setFixedWidth(350)
        self.edit_trim.textChanged.connect(self.onChange)

        layout.addRow( "<b>Stage Name:</b>", self.edit_name )
        layout.addRow( "<b>Description:</b>", self.edit_desc )
        if self.type == "pid" or self.type == "vel":
            layout.addRow( "<b>Input Prop:</b>", self.edit_input )
        elif self.type == "sum":
            for i in range(4):
                layout.addRow( "<b>Input Prop:</b>", self.edit_input[i] )
        if self.type == "pid" or self.type == "vel":
            layout.addRow( "<b>Reference Prop:</b>", self.edit_ref )
        layout.addRow( "<b>Output Prop:</b>", self.edit_output )
        if self.type == "pid" or self.type == "vel":
            layout.addRow( "<b>Kp (global gain):</b>", self.edit_Kp )
        if self.type == "pid":
            layout.addRow( "<b>Ti (integrator time gain):</b>", self.edit_Ti )
            layout.addRow( "<b>Td (derivative time gain):</b>", self.edit_Td )
        elif self.type == "vel":
            layout.addRow( "<b>beta (input weight):</b>", self.edit_beta )
            layout.addRow( "<b>alpha (low pass filter):</b>", self.edit_alpha )
            layout.addRow( "<b>gamma:</b>", self.edit_gamma )
            layout.addRow( "<b>Ti (integrator time gain):</b>", self.edit_Ti )
            layout.addRow( "<b>Td (derivative time gain):</b>", self.edit_Td )
        layout.addRow( "<b>min (output limit):</b>", self.edit_min )
        layout.addRow( "<b>max (output limit):</b>", self.edit_max )
        if self.type == "pid" or self.type == "vel":
            layout.addRow( "<b>trim (P output for zero error):</b>",
                           self.edit_trim )

        # 'Command' button bar
        cmd_group = QtGui.QFrame()
        toplayout.addWidget(cmd_group)
        cmd_layout = QtGui.QHBoxLayout()
        cmd_group.setLayout( cmd_layout )
        cmd_layout.addWidget( QtGui.QLabel("<b>Component Commands:</b> ") )
        update = QtGui.QPushButton('Update')
        update.clicked.connect(self.update)
        cmd_layout.addWidget(update)
        revert = QtGui.QPushButton('Revert')
        revert.clicked.connect(self.revert)
        cmd_layout.addWidget(revert)
        plot = QtGui.QPushButton('Plot')
        plot.clicked.connect(self.plot)
        cmd_layout.addWidget(plot)
        cmd_layout.addStretch(1)

        return toppage

    def get_widget(self):
        return self.container

    def get_name(self):
        return self.edit_name.text()

    def parse(self, node):
        self.edit_name.setText(node.getString('name'))
        self.edit_desc.setText(node.getString('description'))
        if self.type == "pid" or self.type == "vel":
            tmp = node.getChild('input', True)
            self.edit_input.setText(tmp.getString('prop'))
        elif self.type == "sum":
            tmp = node.getChild('input', True)
            len = tmp.getLen('prop')
            for i in range(len):
                prop = tmp.getStringEnum('prop', i)
                self.edit_input[i].setText(prop)
        tmp = node.getChild('reference', True)
        self.edit_ref.setText(tmp.getString('prop'))
        tmp = node.getChild('output', True)
        self.edit_output.setText(tmp.getString('prop'))
        tmp = node.getChild('config', True)
        self.edit_Kp.setText(tmp.getString('Kp'))
        if self.type == "pid":
            self.edit_Ti.setText(tmp.getString('Ti'))
            self.edit_Td.setText(tmp.getString('Td'))
        elif self.type == "vel":
            self.edit_beta.setText(tmp.getString('beta'))
            self.edit_alpha.setText(tmp.getString('alpha'))
            self.edit_gamma.setText(tmp.getString('gamma'))
            self.edit_Ti.setText(tmp.getString('Ti'))
            self.edit_Td.setText(tmp.getString('Td'))
        self.edit_min.setText(tmp.getString('u_min'))
        self.edit_max.setText(tmp.getString('u_max'))
        self.edit_trim.setText(tmp.getString('u_trim'))
        self.original_values = self.value_array()

    def value_array(self):
        result = []
        if self.type == "pid" or self.type == "vel":
            result.append( str(self.edit_Kp.text()) )
        if self.type == "pid":
            result.append( str(self.edit_Ti.text()) )
            result.append( str(self.edit_Td.text()) )
        elif self.type == "vel":
            result.append( str(self.edit_beta.text()) )
            result.append( str(self.edit_alpha.text()) )
            result.append( str(self.edit_gamma.text()) )
            result.append( str(self.edit_Ti.text()) )
            result.append( str(self.edit_Td.text()) )
        result.append( str(self.edit_min.text()) )
        result.append( str(self.edit_max.text()) )
        if self.type == "pid" or self.type == "vel":
            result.append( str(self.edit_trim.text()) )
        print result
        return result

    def update(self):
        command = "fcs-update " + str(self.index)
        for value in self.value_array():
            command += "," + value
        # print "update: " + str(self.value_array())
        print command
        print self.port
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        t.send(command)
        t.quit()

    def revert(self):
        # revert form
        if self.type == "pid":
            self.edit_Kp.setText(self.original_values[0])
            self.edit_Ti.setText(self.original_values[1])
            self.edit_Td.setText(self.original_values[2])
            self.edit_min.setText(self.original_values[3])
            self.edit_max.setText(self.original_values[4])
            self.edit_trim.setText(self.original_values[5])
        elif self.type == "vel":
            self.edit_Kp.setText(self.original_values[0])
            self.edit_beta.setText(self.original_values[1])
            self.edit_alpha.setText(self.original_values[2])
            self.edit_gamma.setText(self.original_values[3])
            self.edit_Ti.setText(self.original_values[4])
            self.edit_Td.setText(self.original_values[5])
            self.edit_min.setText(self.original_values[6])
            self.edit_max.setText(self.original_values[7])
            self.edit_trim.setText(self.original_values[8])
        # send original values to remote
        command = "fcs-update " + str(self.index)
        for value in self.original_values:
            command += "," + value
        #print "update: " + str(self.value_array())
        print command
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        t.send(command)
        t.quit()

    def plot(self):
        print "plotting place holder"
        input = self.fields.get_index( self.edit_input.text() )
        ref = self.fields.get_index( self.edit_ref.text() )
        output = self.fields.get_index( self.edit_output.text() )
        print input, ref, output
        
        plt.ion()
        ax1 = plt.gca()
        ax2 = ax1.twinx()
        data = fetcher.df.get_data()
        p1 = ax1.plot(data[:,0], data[:,input], 'b', label='current')[0]
        p2 = ax1.plot(data[:,0], data[:,ref], 'g', label='target')[0]
        p3 = ax2.plot(data[:,0], data[:,output], 'r', label='output')[0]
        ax1.set_xlabel('time (sec)')
        if 'deg' in self.edit_input.text():
            ax1.set_ylabel('degrees')
        elif 'ft' in self.edit_input.text():
            ax1.set_ylabel('feet')
        elif 'kt' in self.edit_input.text():
            ax1.set_ylabel('kts')
        else:
            ax1.set_ylabel('unknown')
        if 'deg' in self.edit_output.text():
            ax2.set_ylabel('degree')
        elif 'flight' in self.edit_output.text() or 'engine' in self.edit_output.text():
            ax2.set_ylabel('actuator norm')
        else:
            ax2.set_ylabel('unknown')
            
        ax1.legend(loc=2)
        ax2.legend(loc=3)
        ax1.set_title(self.edit_name.text())
        
        while True:
            data = fetcher.df.get_data()
            p1.set_data(data[:,0], data[:,input])
            p2.set_data(data[:,0], data[:,ref])
            p3.set_data(data[:,0], data[:,output])
            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()
            plt.pause(0.25)
            #time.sleep(0.25)
