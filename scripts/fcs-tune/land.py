from PyQt4 import QtGui, QtCore
import subprocess

import fgtelnet

class Land():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "0", "3.0", "75", "8", "left", "50", "0", "25", "5", "5" ]
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

        self.edit_rwy_lon = QtGui.QLineEdit()
        self.edit_rwy_lon.setFixedWidth(350)
        self.edit_rwy_lon.textChanged.connect(self.onChange)
        self.edit_rwy_lat = QtGui.QLineEdit()
        self.edit_rwy_lat.setFixedWidth(350)
        self.edit_rwy_lat.textChanged.connect(self.onChange)
        self.edit_rwy_hdg = QtGui.QLineEdit()
        self.edit_rwy_hdg.setFixedWidth(350)
        self.edit_rwy_hdg.textChanged.connect(self.onChange)
        self.edit_lat_offset = QtGui.QLineEdit()
        self.edit_lat_offset.setFixedWidth(350)
        self.edit_lat_offset.textChanged.connect(self.onChange)
        self.edit_glideslope = QtGui.QLineEdit()
        self.edit_glideslope.setFixedWidth(350)
        self.edit_glideslope.textChanged.connect(self.onChange)
        self.edit_turn_radius = QtGui.QLineEdit()
        self.edit_turn_radius.setFixedWidth(350)
        self.edit_turn_radius.textChanged.connect(self.onChange)
        self.edit_turn_steps = QtGui.QLineEdit()
        self.edit_turn_steps.setFixedWidth(350)
        self.edit_turn_steps.textChanged.connect(self.onChange)
        self.edit_direction = QtGui.QLineEdit()
        self.edit_direction.setFixedWidth(350)
        self.edit_direction.textChanged.connect(self.onChange)
        self.edit_final_leg = QtGui.QLineEdit()
        self.edit_final_leg.setFixedWidth(350)
        self.edit_final_leg.textChanged.connect(self.onChange)
        self.edit_alt_bias = QtGui.QLineEdit()
        self.edit_alt_bias.setFixedWidth(350)
        self.edit_alt_bias.textChanged.connect(self.onChange)
        self.edit_appr_speed = QtGui.QLineEdit()
        self.edit_appr_speed.setFixedWidth(350)
        self.edit_appr_speed.textChanged.connect(self.onChange)
        self.edit_flare_pitch = QtGui.QLineEdit()
        self.edit_flare_pitch.setFixedWidth(350)
        self.edit_flare_pitch.textChanged.connect(self.onChange)
        self.edit_flare_time = QtGui.QLineEdit()
        self.edit_flare_time.setFixedWidth(350)
        self.edit_flare_time.textChanged.connect(self.onChange)

        layout.addRow( "<b>Runway Longitude (deg):</b>", self.edit_rwy_lon )
        layout.addRow( "<b>Runway Latitude (deg):</b>", self.edit_rwy_lat )
        layout.addRow( "<b>Runway Heading (deg):</b>", self.edit_rwy_hdg )
        layout.addRow( "<b>TD Lateral Offset (m):</b>", self.edit_lat_offset )
        layout.addRow( "<b>Approach Glideslope (deg):</b>", self.edit_glideslope )
        layout.addRow( "<b>Final Turn Radius (m):</b>", self.edit_turn_radius )
        layout.addRow( "<b>Turn Steps (8):</b>", self.edit_turn_steps )
        layout.addRow( "<b>Turn Direction (left/right):</b>", self.edit_direction )
        layout.addRow( "<b>Extend Final Leg Len (m):</b>", self.edit_final_leg )
        layout.addRow( "<b>Altitude Bias (ft):</b>", self.edit_alt_bias )
        layout.addRow( "<b>Approach Speed (kt):</b>", self.edit_appr_speed )
        layout.addRow( "<b>Flare Pitch (deg):</b>", self.edit_flare_pitch )
        layout.addRow( "<b>Flare Time (sec):</b>", self.edit_flare_time )

        # 'Parameter' button bar
        param_group = QtGui.QFrame()
        toplayout.addWidget(param_group)
        param_layout = QtGui.QHBoxLayout()
        param_group.setLayout( param_layout )
        param_layout.addWidget( QtGui.QLabel("<b>Land Parameters:</b> ") )
        home = QtGui.QPushButton('Land "HOME"')
        home.clicked.connect(self.task_rwy_home)
        param_layout.addWidget(home)
        sprc36 = QtGui.QPushButton('SPRC 36 (to N)')
        sprc36.clicked.connect(self.task_rwy_sprc36)
        param_layout.addWidget(sprc36)
        sprc18 = QtGui.QPushButton('SPRC 18 (to S)')
        sprc18.clicked.connect(self.task_rwy_sprc18)
        param_layout.addWidget(sprc18)
        #update = QtGui.QPushButton('Update')
        #update.clicked.connect(self.update)
        #param_layout.addWidget(update)
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
        land = QtGui.QPushButton('Land')
        land.clicked.connect(self.task_land)
        cmd_layout.addWidget(land)
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

    def update(self, t):
        print "update land params"

        self.send_value(t, "/mission/land/lateral-offset-m",
                        self.edit_lat_offset.text())
        self.send_value(t, "/mission/land/glideslope-deg",
                        self.edit_glideslope.text())
        self.send_value(t, "/mission/land/turn-radius-m",
                        self.edit_turn_radius.text())
        self.send_value(t, "/mission/land/turn-steps",
                        self.edit_turn_steps.text())
        self.send_value(t, "/mission/land/direction",
                        self.edit_direction.text())
        self.send_value(t, "/mission/land/extend-final-leg-m",
                        self.edit_final_leg.text())
        self.send_value(t, "/mission/land/altitude-bias-ft",
                        self.edit_alt_bias.text())
        self.send_value(t, "/mission/land/approach-speed-kt",
                        self.edit_appr_speed.text())
        self.send_value(t, "/mission/land/flare-pitch-deg",
                        self.edit_flare_pitch.text())
        self.send_value(t, "/mission/land/flare-seconds",
                        self.edit_flare_time.text())

        # send these last so our 'route' doesn't stay moved long
        # before we kick into land mode.
        if len(self.edit_rwy_lon.text()):
            self.send_value(t, "/mission/home/longitude-deg",
                            self.edit_rwy_lon.text())
        if len(self.edit_rwy_lat.text()):
            self.send_value(t, "/mission/home/latitude-deg",
                            self.edit_rwy_lat.text())
        if len(self.edit_rwy_hdg.text()):
            self.send_value(t, "/mission/home/azimuth-deg",
                            self.edit_rwy_hdg.text())


    def revert(self):
        print str(self.original_values)
        # revert form
        self.edit_rwy_lon.setText( "" )
        self.edit_rwy_lat.setText( "" )
        self.edit_rwy_hdg.setText( "" )
        self.edit_lat_offset.setText( self.original_values[0] )
        self.edit_glideslope.setText( self.original_values[1] )
        self.edit_turn_radius.setText( self.original_values[2] )
        self.edit_turn_steps.setText( self.original_values[3] )
        self.edit_direction.setText( self.original_values[4] )
        self.edit_final_leg.setText( self.original_values[5] )
        self.edit_alt_bias.setText( self.original_values[6] )
        self.edit_appr_speed.setText( self.original_values[7] )
        self.edit_flare_pitch.setText( self.original_values[8] )
        self.edit_flare_time.setText( self.original_values[9] )

    def set_runway(self, lon, lat, az):
        self.edit_rwy_lon.setText( str(lon) )
        self.edit_rwy_lat.setText( str(lat) )
        self.edit_rwy_hdg.setText( str(az) )

    def task_rwy_home(self):
        print "Set Runway to 'Home'"
        self.edit_rwy_lon.setText( "" )
        self.edit_rwy_lat.setText( "" )

    def task_rwy_sprc36(self):
        print "Set Runway to SPRC 36"
        self.set_runway( -93.1456618, 45.2202032, 5 )

    def task_rwy_sprc18(self):
        print "Set Runway to SPRC 36"
        self.set_runway( -93.1455029, 45.2205971, 185 )

    def task_land(self):
        print "Land!"

        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        #t.send("set /mission/command-request task,land,5")

        # send over current landing configuration and touchdown point
        self.update(t)

        cmd = "task,land"
        az = self.edit_rwy_hdg.text()
        if len(az):
            cmd += "," + az
        if self.port == 5402:
            t.send(str("send " + cmd))
        else:
            t.send(str("set /mission/command-request " + cmd))
        t.quit()

    def task_resume(self):
        print "Resume route ..."
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        if self.port == 5402:
            t.send("send task,resume")
        else:
            t.send("set /mission/command-request task,resume")
        t.quit()
