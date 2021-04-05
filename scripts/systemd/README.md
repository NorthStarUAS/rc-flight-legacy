# Automatic startup on boot

configure a beaglebone (systemd) system to automatically start the
Rice Creek UAS flight control app when booted.

/usr/local/bin should already exist
sudo cp bb-flight.sh /usr/local/bin
sudo cp setup-uarts.sh /usr/local/bin

sudo cp bb-flight.service /lib/systemd/
sudo cp setup-uarts.service /lib/systemd/

# note this needs to be a hard link, systemctl enable will refuse if it's a
# symbolic (-s) link.
sudo ln /lib/systemd/bb-flight.service /etc/systemd/system/
sudo ln /lib/systemd/setup-uarts.service /etc/systemd/system/

sudo systemctl daemon-reload

sudo systemctl start bb-flight.service
sudo systemctl start setup-uarts.service

sudo systemctl enable bb-flight.service
sudo systemctl enable setup-uarts.service

Troubleshooting:

- If the systemctl reports: Failed to get D-Bus connection: No connection to service manager.

  Make sure beaglebone /boot/uEnv.txt file has:

    cmdline=init=/lib/systemd/systemd

  This causes the system to boot with systemd

- uEnv.txt example, boot with systemd, enable specific uarts, disable hdmi
  which can generate RF noise that hinders gps:

  ### specifically for Debian 7.9 (wheezy)
  # Enable systemd
  cmdline=quiet init=/lib/systemd/systemd
  # enable specific UARTs
  cape_enable=capemgr.enable_partno=BB-UART1,BB-UART2,BB-UART4
  # disable hdmi (generates gps interference)
  cape_disable=capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN
