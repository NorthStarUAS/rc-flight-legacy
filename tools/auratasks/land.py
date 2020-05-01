from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout, QTabWidget, QFrame, QHBoxLayout, QPushButton, QLabel, QFormLayout, QLineEdit
import subprocess

from combobox_nowheel import QComboBoxNoWheel
import fgtelnet

class Land():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "0", "5", "75", "left", "50", "10", "25", "0", "5" ]
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

        self.edit_lat_offset = QLineEdit()
        self.edit_lat_offset.setFixedWidth(350)
        self.edit_lat_offset.textChanged.connect(self.onChange)
        self.edit_glideslope = QLineEdit()
        self.edit_glideslope.setFixedWidth(350)
        self.edit_glideslope.textChanged.connect(self.onChange)
        self.edit_turn_radius = QLineEdit()
        self.edit_turn_radius.setFixedWidth(350)
        self.edit_turn_radius.textChanged.connect(self.onChange)
        self.edit_direction = QComboBoxNoWheel()
        self.edit_direction.setFixedWidth(350)
        self.edit_direction.addItem('left')
        self.edit_direction.addItem('right')
        self.edit_direction.currentIndexChanged.connect(self.onChange)
        self.edit_final_leg = QLineEdit()
        self.edit_final_leg.setFixedWidth(350)
        self.edit_final_leg.textChanged.connect(self.onChange)
        self.edit_alt_bias = QLineEdit()
        self.edit_alt_bias.setFixedWidth(350)
        self.edit_alt_bias.textChanged.connect(self.onChange)
        self.edit_appr_speed = QLineEdit()
        self.edit_appr_speed.setFixedWidth(350)
        self.edit_appr_speed.textChanged.connect(self.onChange)
        self.edit_flare_pitch = QLineEdit()
        self.edit_flare_pitch.setFixedWidth(350)
        self.edit_flare_pitch.textChanged.connect(self.onChange)
        self.edit_flare_time = QLineEdit()
        self.edit_flare_time.setFixedWidth(350)
        self.edit_flare_time.textChanged.connect(self.onChange)

        layout.addRow( "<b>TD Lateral Offset (m):</b>", self.edit_lat_offset )
        layout.addRow( "<b>Approach Glideslope (deg):</b>", self.edit_glideslope )
        layout.addRow( "<b>Final Turn Radius (m):</b>", self.edit_turn_radius )
        layout.addRow( "<b>Turn Direction:</b>", self.edit_direction )
        layout.addRow( "<b>Extend Final Leg Len (m):</b>", self.edit_final_leg )
        layout.addRow( "<b>Altitude Bias (ft):</b>", self.edit_alt_bias )
        layout.addRow( "<b>Approach Speed (kt):</b>", self.edit_appr_speed )
        layout.addRow( "<b>Flare Pitch (deg):</b>", self.edit_flare_pitch )
        layout.addRow( "<b>Flare Time (sec):</b>", self.edit_flare_time )

        # 'Parameter' button bar
        param_group = QFrame()
        toplayout.addWidget(param_group)
        param_layout = QHBoxLayout()
        param_group.setLayout( param_layout )
        param_layout.addWidget( QLabel("<b>Commands:</b> ") )
        update = QPushButton('Update')
        update.clicked.connect(self.update)
        param_layout.addWidget(update)
        revert = QPushButton('Revert')
        revert.clicked.connect(self.revert)
        param_layout.addWidget(revert)
        land = QPushButton('Land Now!')
        land.clicked.connect(self.task_land)
        param_layout.addWidget(land)
        param_layout.addStretch(1)

        page1 = QFrame()
        layout1 = QFormLayout()
        page1.setLayout( layout1 )
        toplayout.addWidget( page1 )

        self.edit_rwy_lon = QLineEdit()
        self.edit_rwy_lon.setFixedWidth(350)
        self.edit_rwy_lon.textChanged.connect(self.onChange)
        self.edit_rwy_lat = QLineEdit()
        self.edit_rwy_lat.setFixedWidth(350)
        self.edit_rwy_lat.textChanged.connect(self.onChange)
        self.edit_rwy_hdg = QLineEdit()
        self.edit_rwy_hdg.setFixedWidth(350)
        self.edit_rwy_hdg.textChanged.connect(self.onChange)

        layout1.addRow( "<b>Runway Longitude (deg):</b>", self.edit_rwy_lon )
        layout1.addRow( "<b>Runway Latitude (deg):</b>", self.edit_rwy_lat )
        layout1.addRow( "<b>Runway Heading (deg):</b>", self.edit_rwy_hdg )

        # 'rwy' button bar
        rwy_group = QFrame()
        toplayout.addWidget(rwy_group)
        rwy_layout = QHBoxLayout()
        rwy_group.setLayout( rwy_layout )
        rwy_layout.addWidget( QLabel("<b>Runway Presets:</b> ") )
        home = QPushButton('Land "HOME"')
        home.clicked.connect(self.task_rwy_home)
        rwy_layout.addWidget(home)
        sprc36 = QPushButton('SPRC 36 (to N)')
        sprc36.clicked.connect(self.task_rwy_sprc36)
        rwy_layout.addWidget(sprc36)
        sprc18 = QPushButton('SPRC 18 (to S)')
        sprc18.clicked.connect(self.task_rwy_sprc18)
        rwy_layout.addWidget(sprc18)
        rwy_layout.addStretch(1)

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
        print("update land params")
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        self.send_value(t, "/task/land/lateral_offset_m",
                        self.edit_lat_offset.text())
        self.send_value(t, "/task/land/glideslope_deg",
                        self.edit_glideslope.text())
        self.send_value(t, "/task/land/turn_radius_m",
                        self.edit_turn_radius.text())
        self.send_value(t, "/task/land/direction",
                        self.edit_direction.currentText())
        self.send_value(t, "/task/land/extend_final_leg_m",
                        self.edit_final_leg.text())
        self.send_value(t, "/task/land/altitude_bias_ft",
                        self.edit_alt_bias.text())
        self.send_value(t, "/task/land/approach_speed_kt",
                        self.edit_appr_speed.text())
        self.send_value(t, "/task/land/flare_pitch_deg",
                        self.edit_flare_pitch.text())
        self.send_value(t, "/task/land/flare_seconds",
                        self.edit_flare_time.text())

        # send these last so our 'route' doesn't stay moved long
        # before we kick into land mode.
        if len(self.edit_rwy_lon.text()):
            self.send_value(t, "/task/home/longitude_deg",
                            self.edit_rwy_lon.text())
        if len(self.edit_rwy_lat.text()):
            self.send_value(t, "/task/home/latitude_deg",
                            self.edit_rwy_lat.text())
        if len(self.edit_rwy_hdg.text()):
            self.send_value(t, "/task/home/azimuth_deg",
                            self.edit_rwy_hdg.text())
        t.quit()

    def revert(self):
        print(str(self.original_values))
        # revert form
        self.edit_rwy_lon.setText( "" )
        self.edit_rwy_lat.setText( "" )
        self.edit_rwy_hdg.setText( "" )
        self.edit_lat_offset.setText( self.original_values[0] )
        self.edit_glideslope.setText( self.original_values[1] )
        self.edit_turn_radius.setText( self.original_values[2] )
        index = self.edit_direction.findText(self.original_values[3])
        if index == None: index = 1
        self.edit_direction.setCurrentIndex(index)
        self.edit_final_leg.setText( self.original_values[4] )
        self.edit_alt_bias.setText( self.original_values[5] )
        self.edit_appr_speed.setText( self.original_values[6] )
        self.edit_flare_pitch.setText( self.original_values[7] )
        self.edit_flare_time.setText( self.original_values[8] )

    def set_runway(self, lon, lat, az):
        self.edit_rwy_lon.setText( str(lon) )
        self.edit_rwy_lat.setText( str(lat) )
        self.edit_rwy_hdg.setText( str(az) )

    def task_rwy_home(self):
        print("Set Runway to 'Home'")
        self.set_runway("", "", "")

    def task_rwy_sprc36(self):
        print("Set Runway to SPRC 36")
        self.set_runway( -93.1454915, 45.2202454, 5 )

    def task_rwy_sprc18(self):
        print("Set Runway to SPRC 18")
        self.set_runway( -93.1453326, 45.2205971, 185 )

    def task_land(self):
        print("Land!")

        # send over current landing configuration and touchdown point
        self.update()

        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        cmd = "land"
        az = self.edit_rwy_hdg.text()
        if len(az):
            cmd += "," + az
        if self.port != 6499:
            cmd = 'send task,' + cmd
        else:
            cmd = 'set /task/command ' + cmd
        print('sending:', cmd)
        t.send(str(cmd))
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
