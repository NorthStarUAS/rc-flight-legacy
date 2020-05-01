import math
from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout, QTabWidget, QFrame, QHBoxLayout, QPushButton, QLabel, QFormLayout, QLineEdit
import subprocess

from combobox_nowheel import QComboBoxNoWheel
import fgtelnet

class Chirp():
    def __init__(self, changefunc, host="localhost", port=6499):
        self.changefunc = changefunc
        self.host = host
        self.port = port
        self.original_values = [ "0.5", "7", "20", "5", "aileron" ]
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

        self.edit_start_freq = QLineEdit()
        self.edit_start_freq.setFixedWidth(350)
        self.edit_start_freq.textChanged.connect(self.onChange)
        self.edit_end_freq = QLineEdit()
        self.edit_end_freq.setFixedWidth(350)
        self.edit_end_freq.textChanged.connect(self.onChange)
        self.edit_duration = QLineEdit()
        self.edit_duration.setFixedWidth(350)
        self.edit_duration.textChanged.connect(self.onChange)
        self.edit_ampl = QLineEdit()
        self.edit_ampl.setFixedWidth(350)
        self.edit_ampl.textChanged.connect(self.onChange)
        self.edit_inject = QComboBoxNoWheel()
        self.edit_inject.setFixedWidth(350)
        self.edit_inject.addItem('aileron')
        self.edit_inject.addItem('elevator')
        self.edit_inject.addItem('rudder')
        self.edit_inject.addItem('flaps')
        self.edit_inject.addItem('throttle')
        self.edit_inject.currentIndexChanged.connect(self.onChange)

        layout.addRow( "<b>Start Freqency (hz):</b>", self.edit_start_freq )
        layout.addRow( "<b>End Freqency (hz):</b>", self.edit_end_freq )
        layout.addRow( "<b>Duration (sec):</b>", self.edit_duration )
        layout.addRow( "<b>Amplitude (deg):</b>", self.edit_ampl )
        layout.addRow( "<b>Injection Point:</b>", self.edit_inject )

        # 'Parameter' button bar
        param_group = QFrame()
        toplayout.addWidget(param_group)
        param_layout = QHBoxLayout()
        param_group.setLayout( param_layout )
        param_layout.addWidget( QLabel("<b>Chirp Parameters:</b> ") )
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
        cmd_layout.addWidget( QLabel("<b>Task Commands: (set params, then engage with physical switch)</b> ") )
        # circle = QPushButton('Circle Here')
        # circle.clicked.connect(self.task_circle)
        # cmd_layout.addWidget(circle)
        # home = QPushButton('Go Home (and Circle)')
        # home.clicked.connect(self.task_home)
        # cmd_layout.addWidget(home)
        # resume = QPushButton('Resume Route')
        # resume.clicked.connect(self.task_resume)
        # cmd_layout.addWidget(resume)
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
        print("update chirp params")
        t = fgtelnet.FGTelnet(self.host, self.port)
        t.send("data")
        start_str = self.edit_start_freq.text()
        if start_str == "": start_str = "0.0"
        freq_start = "%.2f" % (float(start_str) * 2.0 * math.pi)
        self.send_value(t, "/task/chirp/freq_start_rad_sec", freq_start)
        end_str = self.edit_end_freq.text()
        if end_str == "": end_str = "0.0"
        freq_end = "%.2f" % (float(end_str) * 2.0 * math.pi)
        self.send_value(t, "/task/chirp/freq_end_rad_sec", freq_end)
        self.send_value(t, "/task/chirp/duration_sec",
                        self.edit_duration.text())
        ampl_str = self.edit_ampl.text()
        if ampl_str == "": ampl_str = "0.0"
        ampl = "%.3f" % (float(ampl_str) * math.pi / 180.0)
        self.send_value(t, "/task/chirp/amplitude", ampl)
        self.send_value(t, "/controls/signal/inject", self.edit_inject.currentText())
        t.quit()

    def revert(self):
        print(str(self.original_values))
        # revert form
        self.edit_start_freq.setText( self.original_values[0] )
        self.edit_end_freq.setText( self.original_values[1] )
        self.edit_duration.setText( self.original_values[2] )
        self.edit_ampl.setText( self.original_values[3] )
        index = self.edit_inject.findText(self.original_values[4])
        if index == None: index = 1
        self.edit_inject.setCurrentIndex(index)

        # send original values to remote
        # no # self.update()

    # retained in comments as an example for some future tbd function
    # def task_circle(self):
    #     print("request circle hold at current location")
    #     t = fgtelnet.FGTelnet(self.host, self.port)
    #     t.send("data")
    #     if self.port != 6499:
    #         t.send("send task,circle")
    #     else:
    #         t.send("set /task/command circle")
    #     t.quit()
