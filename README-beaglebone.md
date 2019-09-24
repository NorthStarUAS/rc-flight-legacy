# Beaglebone Setup Notes

February 15, 2018:  Last tested with bone-debian-9.3 (28 January, 2018)

## Update your beaglebone to the newest root image:

    https://beagleboard.org/latest-images

To create an updater sd card from a linux host (example):

    dd if=bone-debian-9.3-iot-armhf-2018-01-28-4gb.img of=/dev/sdd bs=1M

Important note: if booting a beaglebone from sd card, it may still
have an older uboot loaded in the mmc image and that may not support
all the new pin configuration features we need.  This command will
tell you what you have, if you see: U-Boot 2016...something then what
you have is too old:

    $ sudo /opt/scripts/tools/version.sh

You can't work around this by booting from an SD card.  The primary
boot loader actually lives in eMMC so you need to flash the new image
to the eMMC before proceeding.  Someday new beaglebones will ship with
a newer uboot and this will no longer be a concern.

So you can boot with the newest image on the SD card (or edit this
file via the host system)

    $ vi /boot/uEnv.txt

uncomment the last line to enable the emmc flasher script.

    $ reboot

Wait for the cylon lights to finish and the system to poweroff.
Remove the SD card.
Power up.

Boot the newly flashed beaglebone and login as debian/temppwd

    host$ ssh debian@192.168.7.2
    $ sudo bash
    # adduser aura
      password: aura
      Full Name []: AuraUAS
    # usermod -aG admin aura
      
## Setup host/beaglebone Network Relay

The beaglebone can route through the usb net connection to the wide
internet via your Fedora host computer (some other ubuntu based
instructions don't seem to quite work.)

From: http://robotic-controls.com/learn/beaglebone/beaglebone-internet-over-usb-only

On the BeagleBone side:

    # /sbin/route add default gw 192.168.7.1
      echo "nameserver 8.8.8.8" > /etc/resolv.conf

On the host:

    # iptables --flush
      iptables --table nat --flush
      iptables --delete-chain
      iptables --table nat --delete-chain

On the Linux (Fedora) computer. (Note: on my system the wifi interface
is wlp3s0, the usb interface to the beaglebone is enp0s20u1)

    # iptables --table nat --append POSTROUTING --out-interface wlp3s0 -j MASQUERADE
    # iptables --append FORWARD --in-interface enp0s20u1 -j ACCEPT
    # echo 1 > /proc/sys/net/ipv4/ip_forward

On my home desktop:

    # iptables --table nat --append POSTROUTING --out-interface enp7s0 -j MASQUERADE
    # iptables --append FORWARD --in-interface enp0s22f2u2 -j ACCEPT
    # echo 1 > /proc/sys/net/ipv4/ip_forward

Network config: (required before beaglebone dns lookups will actually work)

Fedora < 27:

    -> Select usb ethernet interface
    -> Select gear icon for settings.
    -> Select identity
    -> Firewall Zone -> trusted
    -> Finally, turn off usb ethernet and reactivate it.
    
Fedora >= 27

    -> Install/launch firewall gnome app
    -> select usb interface
    -> select trusted zone

## Configure the Beaglebone 9.3 (iot/non-gui) (stretch) setup:

Edit /boot/uEnv.txt: (only for debian < 9)

    # AuraUAS
    cape_disable=bone_capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN
    cape_enable=bone_capemgr.enable_partno=BB-UART1,BB-UART2,BB-UART4
    
Remove wicd (if gui image)

    apt remove python-wicd wicd-cli wicd wicd-curses wicd-daemon wicd-gtk

Ethernet setup.

- Note 1: enabling eth0 can generate some gps interference close to
  the beaglebone, so use caution and sufficient antenna separation if
  your use case requires this.)
- Note 2: enabling eth0 without it plugged in can introduce extra boot
  delays.

    vi /etc/network/interfaces

Remove stuff we don't need that could cause performance disruptions

    apt remove --purge c9-core-installer nodejs bonescript bone101
    rm -rf /usr/local/lib/node_modules/
    apt remove --purge apache2 apache2-bin apache2-data apache2-utils

(If gui image):

    apt remove --purge xrdp tightvncserver lxqt-common lxqt-panel lxqt-runner lxqt-session gnome-accessibility-themes gnome-icon-theme

Check/remove stuff that takes up significant disk space:

    dpkg-query -W --showformat='${Installed-Size;10}\t${Package}\n' | sort -k1,1n
    apt remove --purge ti-llvm-3.6 chromium-browser
    apt remove --purge c9-core-installer libllvm3.9 ti-opencl firmware-am57xx-opencl-monitor oxygen-icon-theme
    apt remove upm libllvm3.5

Download the available package databases:

    apt update
    apt upgrade

Remove any abandoned packages (installed previously to support
something that was removed and is no longer needed.)

    apt autoremove

Install extra required and/or useful things

    apt install telnet minicom python3-lxml python3-serial libeigen3-dev zlib1g-dev rapidjson-dev python3-pybind11 python3-numpy python3-scipy

Force highest performance mode (already the default in debian >= 9)

    vi /etc/init.d/cpufrequtils (line #43)
    GOVERNOR="performance"

If you have a specific reason to upgrade the stock kernel, here is how
you do it.  Details can be found here:
http://elinux.org/Beagleboard:BeagleBoneBlack_Debian#Kernel_Upgrade

    cd /opt/scripts/tools/
    git pull
    ./update_kernel.sh
    reboot

## Setup Autopilot Software

Setup onboard software repositories (note choice of referencing github
repositories directly, but requires net access to update versus
referencing a local --mirror repository which doesn't require internet
access, just visibility to a host via usb network, but requires
setting up and maintaining the --mirror repositories.)

    aura$ mkdir Source
    aura$ cd Source
    aura$ git clone curt@192.168.7.1:GIT/aura-props.git
    aura$ git clone curt@192.168.7.1:GIT/aura-config.git
    aura$ git clone curt@192.168.7.1:GIT/aura-core.git

finding the software repositories on the web: https://github.com/AuraUAS

    aura$ mkdir Source
    aura$ cd Source
    aura$ git clone https://github.com/AuraUAS/aura-props.git
    aura$ git clone https://github.com/AuraUAS/aura-config.git
    aura$ git clone https://github.com/AuraUAS/aura-core.git 

Build/install aura-props package:

    cd aura-props/python
    ./setup.py build
    sudo ./setup.py install

    cd aura-props/library
    ./autogen.sh
    <follow instructrions>
    cd build
    make
    sudo make install
  
Tip: how to setup a swap partition to 'temporarily' increase the
available RAM.  This might be useful for running a really big task
(but at the expense of performance.)

    sudo mkdir -p /var/cache/swap/
    sudo dd if=/dev/zero of=/var/cache/swap/swapfile bs=1M count=768 (size in MB)
    sudo chmod 0600 /var/cache/swap/swapfile
    sudo mkswap /var/cache/swap/swapfile
    sudo swapon /var/cache/swap/swapfile

Don't forget to remove the swap file (to free up a big chunk of disk space):

    sudo swapoff /var/cache/swap/swapfile
    sudo rm /var/cache/swap/swapfile

Build/install aura-core package

    sudo mkdir /usr/local/AuraUAS
    sudo chown aura.aura /usr/local/AuraUAS
  
    cd aura-core
    ./autogen.sh
    mkdir build; cd build
    ../configure --prefix=/usr/local/AuraUAS CFLAGS="-Wall -O3" CXXFLAGS="-Wall -O3"
    make
    make install

Setup FlightData directory

    mkdir /usr/local/AuraUAS/FlightData
  
AuraUAS Configuration

    ln -s /home/aura/Source/aura-config/config /usr/local/AuraUAS/config
    cd ~/Source/aura-config/config
    ln -s main-<aircraft_name>.json main.json

Test run (assuming Goldy3 cape w/ Aura3 firmware is installed) verify
imu info is updating at minimum, overall update rates, load avg if gps
connected, verify gps position and ekf orientation

    cd
    /usr/local/AuraUAS/bin/aura --python_path /home/aura/Source/aura-core/src --config /usr/local/AuraUAS/config/ --display on
  
Setup the systemd autorun scripts:

    See .../aura-core/scripts/systemd/

Enable beaglebone uarts.  Note, source:  https://electronics.trev.id.au/2018/02/09/get-uart-serial-ports-working-beaglebone-black/

    # UART1 (beaglebone black)
    config-pin p9.24 uart
    config-pin p9.26 uart
 
    # UART4 (beaglebone black)
    config-pin p9.11 uart
    config-pin p9.13 uart
  