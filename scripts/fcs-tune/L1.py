from PyQt4 import QtGui, QtCore
import subprocess

import fgtelnet

class L1Controller():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "30", "15", "0.7" ]
        self.container = self.make_page()
        self.xml = None

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

        self.edit_bank_limit = QtGui.QLineEdit()
        self.edit_bank_limit.setFixedWidth(350)
        self.edit_bank_limit.textChanged.connect(self.onChange)
        self.edit_L1_period = QtGui.QLineEdit()
        self.edit_L1_period.setFixedWidth(350)
        self.edit_L1_period.textChanged.connect(self.onChange)
        self.edit_L1_damping = QtGui.QLineEdit()
        self.edit_L1_damping.setFixedWidth(350)
        self.edit_L1_damping.textChanged.connect(self.onChange)

        layout.addRow( "<b>Bank Limit (deg):</b>", self.edit_bank_limit )
        layout.addRow( "<b>L1 Period (10-25):</b>", self.edit_L1_period )
        layout.addRow( "<b>L1 Damping (0.7):</b>", self.edit_L1_damping )

        # 'Parameter' button bar
        param_group = QtGui.QFrame()
        toplayout.addWidget(param_group)
        param_layout = QtGui.QHBoxLayout()
        param_group.setLayout( param_layout )
        param_layout.addWidget( QtGui.QLabel("<b>L1 Parameters:</b> ") )
        update = QtGui.QPushButton('Update')
        update.clicked.connect(self.update)
        param_layout.addWidget(update)
        revert = QtGui.QPushButton('Revert')
        revert.clicked.connect(self.revert)
        param_layout.addWidget(revert)
        param_layout.addStretch(1)

        # 'Command' button bar
        cmd_group = QtGui.QFrame()
        toplayout.addWidget(cmd_group)
        cmd_layout = QtGui.QHBoxLayout()
        cmd_group.setLayout( cmd_layout )
        cmd_layout.addWidget( QtGui.QLabel("<b>Task Commands:</b> ") )
        resume = QtGui.QPushButton('Resume Route')
        resume.clicked.connect(self.task_resume)
        cmd_layout.addWidget(resume)
        cmd_layout.addStretch(1)

        toplayout.addStretch(1)

        # set initial values
        self.revert()

        return toppage

    def get_widget(self):
        return self.container

    def send_value(self, t, prop, val):
        if len(val):
            if self.port == 5402:
                command = "send set," + prop + "," + str(val)
                print command
                t.send(command)
            else:
                command = "set " + prop + " " + str(val)
                print command
                t.send(command)

    def update(self):
        print "update circle hold params"
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        self.send_value(t, "/config/fcs/autopilot/L1-controller/bank-limit-deg",
                        self.edit_bank_limit.text())
        self.send_value(t, "/config/fcs/autopilot/L1-controller/period",
                        self.edit_L1_period.text())
        self.send_value(t, "/config/fcs/autopilot/L1-controller/damping",
                        self.edit_L1_damping.text())
        t.quit()

    def revert(self):
        print str(self.original_values)
        # revert form
        self.edit_bank_limit.setText( self.original_values[0] )
        self.edit_L1_period.setText( self.original_values[1] )
        self.edit_L1_damping.setText( self.original_values[2] )

        # send original values to remote
        self.update()

    def task_resume(self):
        print "Resume route ..."
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        if self.port == 5402:
            t.send("send task,resume")
        else:
            t.send("set /mission/command-request task,resume")
        t.quit()

