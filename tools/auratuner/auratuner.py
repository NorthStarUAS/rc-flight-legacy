#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ugear fcs tuner 

This program hopefully does something eventually ...

author: Curtis L. Olson - curtolson@flightgear.org
website: gallinazo.flightgear.org
started: March 2014
"""

import os.path
import sys
from PyQt4 import QtGui, QtCore
import lxml.etree as ET

from component import Component
from L1 import L1Controller

import fetcher


class Tuner(QtGui.QWidget):
    def __init__(self, filename="", host="localhost", port=6499):
        super(Tuner, self).__init__()
        self.default_title = "FCS Tuner"
        self.components = []
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

        # Route follow parameters
        self.L1 = L1Controller(changefunc=self.onChange, host=host, port=port)
        self.L1.parse_xml( root.find('L1_controller') )
        self.tabs.addTab( self.L1.get_widget(), "L1" )

        # PID controller parameters
        for i,pid_node in enumerate(root.findall('component')):
            e = pid_node.find('module')
            if e != None and e.text != None:
                comp_type = e.text
            else:
                comp_type = 'unknown'
            print "component found:", comp_type
            if comp_type == 'pid_component':
                pid = Component(index=i, changefunc=self.onChange, host=host,
                                port=port, type="pid")
            elif comp_type == 'pid_vel_component':
                pid = Component(index=i, changefunc=self.onChange, host=host,
                                port=port, type="vel")
            else:
                print "unknown ..."
                next
            pid.parse_xml(pid_node)
            self.components.append(pid)
            self.tabs.addTab( pid.get_widget(), pid.get_name() )

    def save(self):
        print "called for save, but does nothing yet"

    def quit(self):
        global data_fetcher_quit
        fetcher.data_fetcher_quit = True
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

    fetcher.df.connect(host, port)
    fetcher.df.update_data()

    ex = Tuner(filename, host=host, port=port)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
