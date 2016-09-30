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

from chirp import Chirp
from circle import Circle
from land import Land
from launch import Launch
from preflight import Preflight
from recalibrate import Recalibrate

import fgtelnet

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

        # Recalibrate page
        self.recalibrate = Recalibrate(changefunc=self.onChange, host=host,
                                       port=port)
        self.tabs.addTab( self.recalibrate.get_widget(), "Recalibrate" )

        # Preflight page
        self.preflight = Preflight(changefunc=self.onChange, host=host,
                                   port=port)
        self.tabs.addTab( self.preflight.get_widget(), "Preflight" )

        # Launch page
        self.launch = Launch(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.launch.get_widget(), "Launch" )

        # Circle hold page
        self.circle = Circle(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.circle.get_widget(), "Circle" )

        # Chirp page
        self.chirp = Chirp(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.chirp.get_widget(), "Chirp" )

        # Land page
        self.land = Land(changefunc=self.onChange, host=host, port=port)
        self.tabs.addTab( self.land.get_widget(), "Land" )

    def save(self):
        print "called for save, but does nothing yet"

    def quit(self):
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
    print "Usage: " + sys.argv[0]

def main():
    host = "localhost"
    #port = 6499
    #host = "192.168.1.64"
    port = 5050

    app = QtGui.QApplication(sys.argv)
    filename = ""
    if len(sys.argv) > 1:
        usage()
        return

    root = getNode('/', create=True)
    
    ex = Tuner(host=host, port=port)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
