#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ugear task master 

This program is intended to be an interface to the mission/task system
running on board the remote vehicle.  Initially the goal is to allow
configuration and instantiation/cancelation of mission tasks.

author: Curtis L. Olson - curtolson@flightgear.org
website: gallinazo.flightgear.org
started: June 2016
"""

import os.path
import sys
from PyQt4 import QtGui, QtCore
import threading
from collections import deque
import time

from props import root, getNode
import props_xml

from chirp import Chirp
from circle import Circle
from land import Land
from preflight import Preflight
from recalibrate import Recalibrate

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
        print "result '%s'" % result
        if result == 'Valid commands are:':
            print "fcs all not supported by remote server"
            return    
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
    def __init__(self, host="localhost", port=6499):
        super(Tuner, self).__init__()
        self.default_title = "Aura Tasks"
        #self.chirp = None
        #self.circle = None
        #self.land = None
        self.initUI()
        self.load(host=host, port=port)
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

    def load(self, host="localhost", port=6499):
        print "Tuner.load " + str(port)

        # Chirp page
        self.chirp = Chirp(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.chirp.get_widget(), "Chirp" )

        # Circle hold page
        self.circle = Circle(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.circle.get_widget(), "Circle" )

        # Land page
        self.land = Land(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.land.get_widget(), "Land" )

        # Preflight page
        self.preflight = Preflight(changefunc=self.onChange, host=host,
                                   port=port)
        self.tabs.addTab( self.preflight.get_widget(), "Preflight" )

        # Recalibrate page
        self.recalibrate = Recalibrate(changefunc=self.onChange, host=host,
                                       port=port)
        self.tabs.addTab( self.recalibrate.get_widget(), "Recalibrate" )

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
    #port = 6499
    #host = "192.168.1.64"
    port = 5050

    app = QtGui.QApplication(sys.argv)
    filename = ""
    if len(sys.argv) > 2:
        usage()
        return
    elif len(sys.argv) == 2:
        filename = sys.argv[1]

    #df = DataFetcher(host=host, port=port)
    #df.update()

    root = getNode('/', create=True)
    props_xml.load(filename, root)
    
    ex = Tuner(host=host, port=port)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
