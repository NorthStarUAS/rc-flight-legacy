#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ugear fcs tuner 

This program hopefully does something eventually ...

author: Curtis L. Olson
website: www.atiak.com
started: March 2014
"""

import os.path
import sys
from PyQt4 import QtGui, QtCore
import lxml.etree as ET
import threading
from collections import deque
import time
import subprocess

import fgtelnet

class PlotFields():
    def __init__(self):
        self.fields = [ \
                        "time", \
                        "/autopilot/settings/target-groundtrack-deg", \
                        "/autopilot/settings/target-roll-deg", \
                        "filter_hdg", \
                        "/orientation/roll-deg", \
                        "/controls/flight/aileron", \
                        "time", \
                        "/autopilot/settings/target-speed-kt", \
                        "/autopilot/settings/target-pitch-deg", \
                        "/velocity/airspeed-kt", \
                        "/orientation/pitch-deg", \
                        "/controls/flight/elevator", \
                        "time", \
                        "/autopilot/settings/target-agl-ft", \
                        "/position/altitude-filter-agl-ft", \
                        "/controls/engine/throttle", \
                    ]

    def get_index(self, field):
        # gnuplot numbers fields starting with 1
        for i,f in enumerate(self.fields):
            if f == field:
                return i+1
        return 1

data_fetcher_quit = False
class DataFetcher():
    def __init__(self, port=6499):
        self.hz = 10
        self.dt = 1.0 / float(self.hz)
        self.seconds = 100
        self.lines = deque()
        self.t = fgtelnet.FGTelnet("localhost", port)
        self.t.send("data")
        self.count = 1

    def update(self):
        self.t.send("fcs all")
        result = self.t.receive()
        tokens = map(float, result.split(','))
        if tokens[3] < 0.0:
            tokens[3] += 360.0
            #print str(tokens[3])
        #tokens[5] *= 25.0
        #tokens[11] *= 25.0
        #tokens[15] *= 200.0
        line = " ".join(map(str, tokens))
        #print line
        self.lines.append(line)
        while len(self.lines) > self.hz * self.seconds:
            self.lines.popleft()
        #print len(self.lines)
        if self.count >= self.hz:
            #print "updating output file"
            self.count = 0
            f = open("plotdata", "w")
            for line in self.lines:
                f.write(line + "\n")
            f.close()
        
        self.count += 1
        #time.sleep(self.dt)
        if not data_fetcher_quit:
            threading.Timer(self.dt, self.update).start()

class Controller():
    def __init__(self, index, changefunc, port=6499):
        self.index = index
        self.changefunc = changefunc
        self.port = port
        self.container = self.make_page()
        self.xml = None
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
        self.edit_input = QtGui.QLineEdit()
        self.edit_input.setFixedWidth(350)
        self.edit_input.textChanged.connect(self.onChange)
        self.edit_ref = QtGui.QLineEdit()
        self.edit_ref.setFixedWidth(350)
        self.edit_ref.textChanged.connect(self.onChange)
        self.edit_output = QtGui.QLineEdit()
        self.edit_output.setFixedWidth(350)
        self.edit_output.textChanged.connect(self.onChange)
        self.edit_Kp = QtGui.QLineEdit()
        self.edit_Kp.setFixedWidth(350)
        self.edit_Kp.textChanged.connect(self.onChange)
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

        layout.addRow( "<b>Stage Name:</b>", self.edit_name )
        layout.addRow( "<b>Description:</b>", self.edit_desc )
        layout.addRow( "<b>Input Prop:</b>", self.edit_input )
        layout.addRow( "<b>Reference Prop:</b>", self.edit_ref )
        layout.addRow( "<b>Output Prop:</b>", self.edit_output )
        layout.addRow( "<b>Kp (global gain):</b>", self.edit_Kp )
        layout.addRow( "<b>beta (input weight):</b>", self.edit_beta )
        layout.addRow( "<b>alpha (low pass filter):</b>", self.edit_alpha )
        layout.addRow( "<b>gamma:</b>", self.edit_gamma )
        layout.addRow( "<b>Ti (integrator time gain):</b>", self.edit_Ti )
        layout.addRow( "<b>Td (derivative time gain):</b>", self.edit_Td )
        layout.addRow( "<b>min (output limit):</b>", self.edit_min )
        layout.addRow( "<b>max (output limit):</b>", self.edit_max )

        # 'Command' button bar
        cmd_group = QtGui.QFrame()
        toplayout.addWidget(cmd_group)
        cmd_layout = QtGui.QHBoxLayout()
        cmd_group.setLayout( cmd_layout )
        cmd_layout.addWidget( QtGui.QLabel("<b>Stage Commands:</b> ") )
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

    def get_value(self, child, parent=None):
        if parent == None:
            parent = self.xml
        e = parent.find(child)
        if e != None and e.text != None:
            return e.text
        else:
            return ""

    def get_node(self, child, parent=None):
        if parent == None:
            parent = self.xml
        return parent.find(child)

    def parse_xml(self, node):
        self.xml = node
        self.edit_name.setText(self.get_value('name'))
        self.edit_desc.setText(self.get_value('description'))
        tmp = self.get_node('input')
        if tmp != None:
            self.edit_input.setText(self.get_value('prop', parent=tmp))
        tmp = self.get_node('reference')
        if tmp != None:
            self.edit_ref.setText(self.get_value('prop', parent=tmp))
        tmp = self.get_node('output')
        if tmp != None:
            self.edit_output.setText(self.get_value('prop', parent=tmp))
        tmp = self.get_node('config')
        if tmp != None:
            self.edit_Kp.setText(self.get_value('Kp', parent=tmp))
            self.edit_beta.setText(self.get_value('beta', parent=tmp))
            self.edit_alpha.setText(self.get_value('alpha', parent=tmp))
            self.edit_gamma.setText(self.get_value('gamma', parent=tmp))
            self.edit_Ti.setText(self.get_value('Ti', parent=tmp))
            self.edit_Td.setText(self.get_value('Td', parent=tmp))
            self.edit_min.setText(self.get_value('u_min', parent=tmp))
            self.edit_max.setText(self.get_value('u_max', parent=tmp))
        self.original_values = self.value_array()

    def value_array(self):
        tmp = self.get_node('config')
        if tmp != None:
            result = []
            result.append( str(self.edit_Kp.text()) )
            result.append( str(self.edit_beta.text()) )
            result.append( str(self.edit_alpha.text()) )
            result.append( str(self.edit_gamma.text()) )
            result.append( str(self.edit_Ti.text()) )
            result.append( str(self.edit_Td.text()) )
            result.append( str(self.edit_min.text()) )
            result.append( str(self.edit_max.text()) )
            return result
        else:
            return None

    def update(self):
        command = "fcs-update " + str(self.index)
        for value in self.value_array():
            command += "," + value
        #print "update: " + str(self.value_array())
        print command
        print self.port
        t = fgtelnet.FGTelnet("localhost", self.port)
        t.send("data")
        t.send(command)
        t.quit()

    def revert(self):
        # revert form
        self.edit_Kp.setText(self.original_values[0])
        self.edit_beta.setText(self.original_values[1])
        self.edit_alpha.setText(self.original_values[2])
        self.edit_gamma.setText(self.original_values[3])
        self.edit_Ti.setText(self.original_values[4])
        self.edit_Td.setText(self.original_values[5])
        self.edit_min.setText(self.original_values[6])
        self.edit_max.setText(self.original_values[7])
        # send original values to remote
        command = "fcs-update " + str(self.index)
        for value in self.original_values:
            command += "," + value
        #print "update: " + str(self.value_array())
        print command
        t = fgtelnet.FGTelnet("localhost", self.port)
        t.send("data")
        t.send(command)
        t.quit()

    def plot(self):
        print "plotting place holder"
        input = self.fields.get_index( self.edit_input.text() )
        ref = self.fields.get_index( self.edit_ref.text() )
        output = self.fields.get_index( self.edit_output.text() )
        script_name = "plot-stage-" + str(self.index) + ".plt"
        f = open(script_name, "w")
        f.write("set xlabel \"" + self.edit_name.text() + "\"\n")
        f.write("set ytics nomirror\n")
        f.write("set y2tic\n")
        f.write("set y2range [" + self.edit_min.text() + ":" + self.edit_max.text() + "]\n")
        f.write("plot \"plotdata\" using 1:" + str(input) + " with lines title \"" + self.edit_input.text() + "\", ")
        f.write(" \"plotdata\" using 1:" + str(ref) + " with lines title \"" + self.edit_ref.text() + "\", ")
        f.write(" \"plotdata\" using 1:" + str(output) + " with lines axis x1y2 title \"" + self.edit_output.text() + "\"\n")
        f.close()

        cmd = "gnuplot -e \"config='" + script_name + "'\" loop.plt"
        print cmd

        command = []
        command.append("gnuplot")
        command.append("-e")
        command.append("config='" + script_name + "'")
        command.append("loop.plt")
        pid = subprocess.Popen(command).pid
        print "spawned plot command = " + str(pid)

class Tuner(QtGui.QWidget):
    def __init__(self, filename="", port=6499):
        super(Tuner, self).__init__()
        self.default_title = "FCS Tuner"
        self.controllers = []
        self.initUI()
        self.load(filename, port=port)
        self.clean = True

    def initUI(self):
        self.setWindowTitle( self.default_title )
        layout = QtGui.QVBoxLayout()
        self.setLayout(layout)

        # Main work area
        self.tabs = QtGui.QTabWidget()
        layout.addWidget( self.tabs )

        #self.overview = Overview(changefunc=self.onChange)
        #self.tabs.addTab( self.overview.get_widget(), "Overview" );

        # 'File' button bar
        file_group = QtGui.QFrame()
        layout.addWidget(file_group)
        file_layout = QtGui.QHBoxLayout()
        file_group.setLayout( file_layout )

        save = QtGui.QPushButton('Save')
        save.clicked.connect(self.save)
        file_layout.addWidget(save)

        quit = QtGui.QPushButton('Quit')
        quit.clicked.connect(self.quit)
        file_layout.addWidget(quit)

        file_layout.addStretch(1)

        self.resize(800, 700)
        self.show()

    def load(self, filename, port=6499):
        print "Tuner.load " + str(port)
        basename = os.path.basename(str(filename))
        fileroot, ext = os.path.splitext(basename)

        if filename == "":
            error = QtGui.QErrorMessage(self)
            error.showMessage( "Error, you must specify an autopilot.xml config file name" )
            return
        elif not os.path.exists(filename):
            error = QtGui.QErrorMessage(self)
            error.showMessage( filename + ": does not exist" )
            return

        try:
            self.xml = ET.parse(filename)
        except:
            error = QtGui.QErrorMessage(self)
            error.showMessage( filename + ": xml parse error:\n"
                               + str(sys.exc_info()[1]) )
            return

        self.filename = str(filename)
        self.fileroot, ext = os.path.splitext(self.filename)

        root = self.xml.getroot()
        for i,pid_node in enumerate(root.findall('pid-controller')):
            print "controller found..."
            pid = Controller(index=i, changefunc=self.onChange, port=port)
            pid.parse_xml(pid_node)
            self.controllers.append(pid)
            self.tabs.addTab( pid.get_widget(), pid.get_name() )
        for pid in root.findall('pi-simple-controller'):
            print "simple controller ignored right now ..."

    def save(self):
        print "called for save, but does nothing yet"

    def quit(self):
        global data_fetcher_quit
        data_fetcher_quit = True
        QtCore.QCoreApplication.instance().quit()

    def onChange(self):
        #print "parent onChange() called!"
        #result = self.rebuildTabNames()
        #if result:
        #    self.rebuildWingLists()
        self.clean = False

    def isClean(self):
        return self.clean

    def setClean(self):
        self.clean = True



def usage():
    print "Usage: " + sys.argv[0] + " [autopilot.xml]"

def main():
    #port = 6499
    port = 5402

    app = QtGui.QApplication(sys.argv)
    filename = ""
    if len(sys.argv) > 2:
        usage()
        return
    elif len(sys.argv) == 2:
        filename = sys.argv[1]

    df = DataFetcher(port=port)
    df.update()

    ex = Tuner(filename, port=port)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
