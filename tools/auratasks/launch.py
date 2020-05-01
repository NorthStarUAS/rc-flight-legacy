from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout, QTabWidget, QFrame, QHBoxLayout, QPushButton, QLabel, QFormLayout, QLineEdit
import subprocess

from combobox_nowheel import QComboBoxNoWheel
import fgtelnet

class Launch():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "20", "150", "300", "0.5", "10.0", "True", "-0.03", "0.5" ]
        self.container = self.make_page()

    def onChange(self):
        self.changefunc()

    def make_page(self):
        toppage = QFrame()
        toplayout = QVBoxLayout()
        toppage.setLayout(toplayout)

        page = QFrame()
        layout = QFormLayout()
        page.setLayout( layout )
        toplayout.addWidget( page )

        self.edit_speed = QLineEdit()
        self.edit_speed.setFixedWidth(350)
        self.edit_speed.textChanged.connect(self.onChange)
        self.edit_completion_agl = QLineEdit()
        self.edit_completion_agl.setFixedWidth(350)
        self.edit_completion_agl.textChanged.connect(self.onChange)
        self.edit_mission_agl = QLineEdit()
        self.edit_mission_agl.setFixedWidth(350)
        self.edit_mission_agl.textChanged.connect(self.onChange)
        self.edit_roll_gain = QLineEdit()
        self.edit_roll_gain.setFixedWidth(350)
        self.edit_roll_gain.textChanged.connect(self.onChange)
        self.edit_roll_limit = QLineEdit()
        self.edit_roll_limit.setFixedWidth(350)
        self.edit_roll_limit.textChanged.connect(self.onChange)
        self.edit_rudder_enable = QComboBoxNoWheel()
        self.edit_rudder_enable.setFixedWidth(350)
        self.edit_rudder_enable.addItem('False')
        self.edit_rudder_enable.addItem('True')
        self.edit_rudder_enable.currentIndexChanged.connect(self.onChange)
        self.edit_rudder_gain = QLineEdit()
        self.edit_rudder_gain.setFixedWidth(350)
        self.edit_rudder_gain.textChanged.connect(self.onChange)
        self.rudder_max = QLineEdit()
        self.rudder_max.setFixedWidth(350)
        self.rudder_max.textChanged.connect(self.onChange)

        layout.addRow( "<b>Climbout Speed (kts):</b>", self.edit_speed )
        layout.addRow( "<b>Completion AGL (ft):</b>", self.edit_completion_agl )
        layout.addRow( "<b>Mission AGL (ft):</b>", self.edit_mission_agl )
        layout.addRow( "<b>Roll Gain:</b>", self.edit_roll_gain )
        layout.addRow( "<b>Roll Limit (deg):</b>", self.edit_roll_limit )
        layout.addRow( "<b>Rudder Enable:</b>", self.edit_rudder_enable )
        layout.addRow( "<b>Rudder Gain:</b>", self.edit_rudder_gain )
        layout.addRow( "<b>Rudder Max (norm):</b>", self.rudder_max )

        # 'Command' button bar
        cmd_group = QFrame()
        toplayout.addWidget(cmd_group)
        cmd_layout = QHBoxLayout()
        cmd_group.setLayout( cmd_layout )
        cmd_layout.addWidget( QLabel("<b>Commands:</b> ") )
        update = QPushButton('Update')
        update.clicked.connect(self.update)
        cmd_layout.addWidget(update)
        revert = QPushButton('Revert')
        revert.clicked.connect(self.revert)
        cmd_layout.addWidget(revert)
        launch = QPushButton('Launch Now!')
        launch.clicked.connect(self.task_launch)
        cmd_layout.addWidget(launch)
        cmd_layout.addStretch(1)

        toplayout.addStretch(1)

        # set initial values
        self.revert()

        return toppage

    def get_widget(self):
        return self.container

    def send_value(self, t, prop, val):
        if len(val):
            if self.port != 6499:
                command = "send set," + prop + "," + str(val)
                print(command)
                t.send(command)
            else:
                command = "set " + prop + " " + str(val)
                print(command)
                t.send(command)

    def update(self):
        print("update launch params")
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        self.send_value(t, "/task/launch/speed_kt",
                        self.edit_speed.text())
        self.send_value(t, "/task/launch/completion_agl_ft",
                        self.edit_completion_agl.text())
        self.send_value(t, "/task/launch/mission_agl_ft",
                        self.edit_mission_agl.text())
        self.send_value(t, "/task/launch/roll_gain",
                        self.edit_roll_gain.text())
        self.send_value(t, "/task/launch/roll_limit",
                        self.edit_roll_limit.text())
        self.send_value(t, "/task/launch/rudder_enable",
                        self.edit_rudder_enable.currentText())
        self.send_value(t, "/task/launch/rudder_gain",
                        self.edit_rudder_gain.text())
        self.send_value(t, "/task/launch/rudder_max",
                        self.rudder_max.text())
        t.quit()

    def revert(self):
        print(str(self.original_values))
        # revert form
        self.edit_speed.setText( self.original_values[0] )
        self.edit_completion_agl.setText( self.original_values[1] )
        self.edit_mission_agl.setText( self.original_values[2] )
        self.edit_roll_gain.setText( self.original_values[3] )
        self.edit_roll_limit.setText( self.original_values[4] )
        index = self.edit_rudder_enable.findText(self.original_values[5])
        if index == None: index = 1
        self.edit_rudder_enable.setCurrentIndex(index)
        self.edit_rudder_gain.setText( self.original_values[6] )
        self.rudder_max.setText( str(self.original_values[7]) )
        
        # send original values to remote
        # no # self.update()

    def task_launch(self):
        print("Launch!")

        # send over current launching configuration
        self.update()

        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        if self.port != 6499:
            t.send("send task,launch")
        else:
            t.send("set /task/command launch")
        t.quit()

    def task_resume(self):
        print("Resume route ...")
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        if self.port != 6499:
            t.send("send task,resume")
        else:
            t.send("set /task/command resume")
        t.quit()
