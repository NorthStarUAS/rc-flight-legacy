from PyQt4 import QtGui, QtCore
import subprocess

import fgtelnet

class TECS():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "3", "0.5", "20", "35" ]
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

        self.edit_mass = QtGui.QLineEdit()
        self.edit_mass.setFixedWidth(350)
        self.edit_mass.textChanged.connect(self.onChange)
        self.edit_weight = QtGui.QLineEdit()
        self.edit_weight.setFixedWidth(350)
        self.edit_weight.textChanged.connect(self.onChange)
        self.edit_min_kt = QtGui.QLineEdit()
        self.edit_min_kt.setFixedWidth(350)
        self.edit_min_kt.textChanged.connect(self.onChange)
        self.edit_max_kt = QtGui.QLineEdit()
        self.edit_max_kt.setFixedWidth(350)
        self.edit_max_kt.textChanged.connect(self.onChange)

        layout.addRow( "<b>Aircraft Total Flying Mass (kg):</b>", self.edit_mass )
        layout.addRow( "<b>TECS pitch weight (0.0 - 1.0):</b>", self.edit_weight )
        layout.addRow( "<b>Minimum speed (kts):</b>", self.edit_min_kt )
        layout.addRow( "<b>Maximum speed (kts):</b>", self.edit_max_kt )
        
        note_group = QtGui.QFrame()
        toplayout.addWidget(note_group)
        note_layout = QtGui.QVBoxLayout()
        note_group.setLayout( note_layout )
        note_layout.addWidget( QtGui.QLabel(
"""<b>Notes</b>:<br>
<ul>
  <li>Mass should represent total flying weight including batteries and payload.</li>
  <li>TECS weight:
    <ul>
      <li>0.0 = pitch controls airspeed, ignores altitude errors.</li>
      <li>1.0 = pitch controls altitude, ignore airspeed errors.</li>
      <li>Default = 0.5.</li>
      <li>For any particular aircraft, you may want to 'slide' this value<br>
          towards the least noisy sensor or the value you care more about.</li>
    <ul>
  </li>
  <li>Min/Max speeds: these are limits to the TECS controller which<br>
     attempts to keep the speed within this range.  In most cases this<br>
     should <b>not</b> be Vs/Vne (stall/never exceed).</li>
<ul>
"""
        ) )
                               
        # 'Command' button bar
        cmd_group = QtGui.QFrame()
        toplayout.addWidget(cmd_group)
        cmd_layout = QtGui.QHBoxLayout()
        cmd_group.setLayout( cmd_layout )
        cmd_layout.addWidget( QtGui.QLabel("<b>TECS Commands:</b> ") )
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
        self.edit_mass.setText(node.getString('mass_kg'))
        self.edit_weight.setText(node.getString('weight'))
        self.edit_min_kt.setText(node.getString('min_kt'))
        self.edit_max_kt.setText(node.getString('max_kt'))
        self.original_values = self.value_array()

    def value_array(self):
        result = []
        result.append( str(self.edit_mass.text()) )
        result.append( str(self.edit_weight.text()) )
        result.append( str(self.edit_min_kt.text()) )
        result.append( str(self.edit_max_kt.text()) )
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
        self.send_value(t, "/config/autopilot/TECS/mass_deg",
                        self.edit_mass.text())
        self.send_value(t, "/config/autopilot/TECS/weight",
                        self.edit_weight.text())
        self.send_value(t, "/config/autopilot/TECS/min_kt",
                        self.edit_min_kt.text())
        self.send_value(t, "/config/autopilot/TECS/max_kt",
                        self.edit_max_kt.text())
        t.quit()

    def revert(self):
        print str(self.original_values)
        # revert form
        self.edit_mass.setText( self.original_values[0] )
        self.edit_weight.setText( self.original_values[1] )
        self.edit_min_kt.setText( self.original_values[2] )
        self.edit_max_kt.setText( self.original_values[3] )

        # send original values to remote
        self.update()

    # def task_resume (self):
    #     print "Resume route ..."
    #     t = fgtelnet.FGTelnet(self.host, self.port)
    #     t.send("data")
    #     if self.port == 5402:
    #         t.send("send task,resume")
    #     else:
    #         t.send("set /task/command-request task,resume")
    #     t.quit()

