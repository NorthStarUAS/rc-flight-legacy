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

from circle import Circle
#from route import Route
from controller import Controller
from land import Land
from L1 import L1Controller

import fgtelnet

data_fetcher_quit = False
class DataFetcher():
    def __init__(self, host="localhost", port=6499):
        self.hz = 10
        self.dt = 1.0 / float(self.hz)
        self.seconds = 100
        self.lines = deque()
        self.t = fgtelnet.FGTelnet(host, port)
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

class Tuner(QtGui.QWidget):
    def __init__(self, filename="", host="localhost", port=6499):
        super(Tuner, self).__init__()
        self.default_title = "FCS Tuner"
        self.circle = None
        self.controllers = []
        self.initUI()
        self.load(filename, host=host, port=port)
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

    def load(self, filename, host="localhost", port=6499):
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

        # Circle hold parameters
        self.circle = Circle(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.circle.get_widget(), "Circle" )

        # Land parameters
        self.land = Land(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.land.get_widget(), "Land" )

        # Route follow parameters
        self.L1 = L1Controller(changefunc=self.onChange, host=host, port=port)
        self.L1.parse_xml( root.find('L1-controller') )
        self.tabs.addTab( self.L1.get_widget(), "L1" )

        # PID controller parameters
        for i,pid_node in enumerate(root.findall('pid-controller')):
            print "controller found..."
            pid = Controller(index=i, changefunc=self.onChange, host=host, port=port, type="full")
            pid.parse_xml(pid_node)
            self.controllers.append(pid)
            self.tabs.addTab( pid.get_widget(), pid.get_name() )
        for i,pid_node in enumerate(root.findall('pi-simple-controller')):
            print "simple controller found..."
            pid = Controller(index=i, changefunc=self.onChange, host=host, port=port, type="simple")
            pid.parse_xml(pid_node)
            self.controllers.append(pid)
            self.tabs.addTab( pid.get_widget(), pid.get_name() )

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
    host = "localhost"
    port = 6499
    #host = "192.168.1.64"
    #port = 5402

    app = QtGui.QApplication(sys.argv)
    filename = ""
    if len(sys.argv) > 2:
        usage()
        return
    elif len(sys.argv) == 2:
        filename = sys.argv[1]

    df = DataFetcher(host=host, port=port)
    df.update()

    ex = Tuner(filename, host=host, port=port)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
