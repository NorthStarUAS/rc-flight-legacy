from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout, QTabWidget, QFrame, QHBoxLayout, QPushButton, QLabel, QFormLayout, QLineEdit
import subprocess

from combobox_nowheel import QComboBoxNoWheel
import fgtelnet

class Circle():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "", "", "left", "100" ]
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

        self.edit_alt = QLineEdit()
        self.edit_alt.setFixedWidth(350)
        self.edit_alt.textChanged.connect(self.onChange)
        self.edit_speed = QLineEdit()
        self.edit_speed.setFixedWidth(350)
        self.edit_speed.textChanged.connect(self.onChange)
        self.edit_direction = QComboBoxNoWheel()
        self.edit_direction.setFixedWidth(350)
        self.edit_direction.addItem('left')
        self.edit_direction.addItem('right')
        self.edit_direction.currentIndexChanged.connect(self.onChange)
        self.edit_radius = QLineEdit()
        self.edit_radius.setFixedWidth(350)
        self.edit_radius.textChanged.connect(self.onChange)

        layout.addRow( "<b>Altitude AGL (ft):</b>", self.edit_alt )
        layout.addRow( "<b>Speed (kt):</b>", self.edit_speed )
        layout.addRow( "<b>Direction:</b>", self.edit_direction )
        layout.addRow( "<b>Radius (m):</b>", self.edit_radius )

        # 'Parameter' button bar
        param_group = QFrame()
        toplayout.addWidget(param_group)
        param_layout = QHBoxLayout()
        param_group.setLayout( param_layout )
        param_layout.addWidget( QLabel("<b>Circling Parameters:</b> ") )
        update = QPushButton('Update')
        update.clicked.connect(self.update)
        param_layout.addWidget(update)
        revert = QPushButton('Revert')
        revert.clicked.connect(self.revert)
        param_layout.addWidget(revert)
        param_layout.addStretch(1)

        # 'Command' button bar
        cmd_group = QFrame()
        toplayout.addWidget(cmd_group)
        cmd_layout = QHBoxLayout()
        cmd_group.setLayout( cmd_layout )
        cmd_layout.addWidget( QLabel("<b>Task Commands:</b> ") )
        circle = QPushButton('Circle Here')
        circle.clicked.connect(self.task_circle)
        cmd_layout.addWidget(circle)
        home = QPushButton('Go Home (and Circle)')
        home.clicked.connect(self.task_home)
        cmd_layout.addWidget(home)
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
        print("update circle hold params")
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        self.send_value(t, "/autopilot/targets/altitude_agl_ft",
                        self.edit_alt.text())
        self.send_value(t, "/autopilot/targets/airspeed_kt",
                        self.edit_speed.text())
        self.send_value(t, "/task/circle/direction",
                        self.edit_direction.currentText())
        self.send_value(t, "/task/circle/radius_m",
                        self.edit_radius.text())
        t.quit()

    def revert(self):
        print(str(self.original_values))
        # revert form
        self.edit_alt.setText( self.original_values[0] )
        self.edit_speed.setText( self.original_values[1] )
        index = self.edit_direction.findText(self.original_values[2])
        if index == None: index = 1
        self.edit_direction.setCurrentIndex(index)
        self.edit_radius.setText( self.original_values[3] )

        # send original values to remote
        self.update()

    def task_circle(self):
        print("request circle hold at current location")
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        if self.port != 6499:
            t.send("send task,circle")
        else:
            t.send("set /task/command circle")
        t.quit()

    def task_home(self):
        print("request circle hold at home")
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        if self.port != 6499:
            t.send("send task,home")
        else:
            t.send("set /task/command home")
        t.quit()

