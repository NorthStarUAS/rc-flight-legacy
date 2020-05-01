from PyQt4 import QtGui, QtCore
import subprocess

import fgtelnet

class L1Controller():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "30", "0", "15", "0.7" ]
        self.container = self.make_page()

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
        self.edit_bank_bias = QtGui.QLineEdit()
        self.edit_bank_bias.setFixedWidth(350)
        self.edit_bank_bias.textChanged.connect(self.onChange)
        self.edit_L1_period = QtGui.QLineEdit()
        self.edit_L1_period.setFixedWidth(350)
        self.edit_L1_period.textChanged.connect(self.onChange)
        self.edit_L1_damping = QtGui.QLineEdit()
        self.edit_L1_damping.setFixedWidth(350)
        self.edit_L1_damping.textChanged.connect(self.onChange)

        layout.addRow( "<b>Bank Limit (deg):</b>", self.edit_bank_limit )
        layout.addRow( "<b>Bank Bias (deg):</b>", self.edit_bank_bias )
        layout.addRow( "<b>L1 Period (10-25):</b>", self.edit_L1_period )
        layout.addRow( "<b>L1 Damping (0.7):</b>", self.edit_L1_damping )
        
        note_group = QtGui.QFrame()
        toplayout.addWidget(note_group)
        note_layout = QtGui.QVBoxLayout()
        note_group.setLayout( note_layout )
        note_layout.addWidget( QtGui.QLabel(
"""
<b>Notes</b>:
<ul>
  <li>Tune L1 period for good circle hold first, then tune L1 damping to
      fine tune route following.</li>
<ul>
"""
        ) )

        # 'Command' button bar
        cmd_group = QtGui.QFrame()
        toplayout.addWidget(cmd_group)
        cmd_layout = QtGui.QHBoxLayout()
        cmd_group.setLayout( cmd_layout )
        cmd_layout.addWidget( QtGui.QLabel("<b>L1 Commands:</b> ") )
        update = QtGui.QPushButton('Update')
        update.clicked.connect(self.update)
        cmd_layout.addWidget(update)
        revert = QtGui.QPushButton('Revert')
        revert.clicked.connect(self.revert)
        cmd_layout.addWidget(revert)
        cmd_layout.addStretch(1)

        toplayout.addStretch(1)

        return toppage

    def get_widget(self):
        return self.container

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

    def parse(self, node):
        self.edit_bank_limit.setText(node.getString('bank_limit_deg'))
        self.edit_bank_bias.setText(node.getString('bank_bias_deg'))
        self.edit_L1_period.setText(node.getString('period'))
        self.edit_L1_damping.setText(node.getString('damping'))
        self.original_values = self.value_array()

    def value_array(self):
        result = []
        result.append( str(self.edit_bank_limit.text()) )
        result.append( str(self.edit_bank_bias.text()) )
        result.append( str(self.edit_L1_period.text()) )
        result.append( str(self.edit_L1_damping.text()) )
        return result

    def send_value(self, t, prop, val):
        if len(val):
            if self.port == 5050:
                command = "send set," + prop + "," + str(val)
                print command
                t.send(command)
            else:
                command = "set " + prop + " " + str(val)
                print command
                t.send(command)

    def update(self):
        print "update L1 params"
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        self.send_value(t, "/config/autopilot/L1_controller/bank_limit_deg",
                        self.edit_bank_limit.text())
        self.send_value(t, "/config/autopilot/L1_controller/bank_bias_deg",
                        self.edit_bank_bias.text())
        self.send_value(t, "/config/autopilot/L1_controller/period",
                        self.edit_L1_period.text())
        self.send_value(t, "/config/autopilot/L1_controller/damping",
                        self.edit_L1_damping.text())
        t.quit()

    def revert(self):
        print str(self.original_values)
        # revert form
        self.edit_bank_limit.setText( self.original_values[0] )
        self.edit_bank_bias.setText( self.original_values[1] )
        self.edit_L1_period.setText( self.original_values[2] )
        self.edit_L1_damping.setText( self.original_values[3] )

        # send original values to remote
        self.update()

    # def task_resume(self):
    #     print "Resume route ..."
    #     t = fgtelnet.FGTelnet(self.host, self.port)
    #     t.send("data")
    #     if self.port == 5402:
    #         t.send("send task,resume")
    #     else:
    #         t.send("set /task/command resume")
    #     t.quit()

