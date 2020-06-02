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

from props import root, getNode
import props_json

from L1 import L1Controller
from TECS import TECS
from component import Component

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
            error.showMessage( "Error, you must specify an autopilot config file name" )
            return
        elif not os.path.exists(filename):
            error = QtGui.QErrorMessage(self)
            error.showMessage( filename + ": does not exist" )
            return

        try:
            props_json.load(filename, root)
        except:
            error = QtGui.QErrorMessage(self)
            error.showMessage( filename + ": parse error:\n"
                               + str(sys.exc_info()[1]) )
            return

        self.filename = str(filename)
        self.fileroot, ext = os.path.splitext(self.filename)

        # L1 Route follow parameters
        self.L1 = L1Controller(changefunc=self.onChange, host=host, port=port)
        L1_node = getNode('/L1_controller', create=True)
        self.L1.parse( L1_node )
        self.tabs.addTab( self.L1.get_widget(), "L1" )

        # TECS parameters
        self.TECS = TECS(changefunc=self.onChange, host=host, port=port)
        TECS_node = getNode('/TECS', create=True)
        self.TECS.parse( TECS_node )
        self.tabs.addTab( self.TECS.get_widget(), "TECS" )

        # PID controller parameters
        print root.getChild('component')
        len = root.getLen('component')
        #comp_node = getNode('/component', create=True)
        for i in range(len):
            node_name = 'component[%d]' % i
            print node_name
            node = root.getChild(node_name)
            comp_type = node.getString('module')
            if comp_type == 'pid':
                pid = Component(index=i, changefunc=self.onChange, host=host,
                                port=port, type="pid")
            elif comp_type == 'pid_velocity':
                pid = Component(index=i, changefunc=self.onChange, host=host,
                                port=port, type="vel")
            elif comp_type == 'summer':
                pid = Component(index=i, changefunc=self.onChange, host=host,
                                port=port, type="sum")
            else:
                print "unknown ..."
                next
            pid.parse(node)
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
