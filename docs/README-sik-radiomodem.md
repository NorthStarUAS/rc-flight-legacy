# Sik setup and notes

These notes reference the sik (firmware) style radio modems sold by
mRobotics and others which are wildly popular in the DIY drones
community.

Reference: http://ardupilot.org/copter/docs/common-3dr-radio-advanced-configuration-and-technical-information.html

Out of the box default baud is 57600 so set minicom to this to talk to
the modem!

To show settings:

    ATI5
    RTI5

Set host baud rate:

    RTS1=115
    ATS1=115

Set over-the-air (ota) baud rate:

    RTS2=128
    ATS2=128

Set net id (default 25, ok to leave the same or change if desired):
Note: it appears that all modems on a particular id will see each
other's messages.  This could be a feature if multiple vehicles are
cooperating.  If you want unique pairs to communicate without
inteference, then setup unique pairs of id's here.

    RTS3=25
    ATS3=25

Set maximum tx power code to 20 (100mw)

    RTS4=20
    ATS4=20
    
Turn off mavlink packet injection (assuming we aren't a mavlink based system):

    RTS6=0
    ATS6=0

Save the changes, write to eeprom:

    RT&W
    AT&W

Reboot the radio to make new settings effective (and change baud on
minicom to verify)

    RTZ
    ATZ
