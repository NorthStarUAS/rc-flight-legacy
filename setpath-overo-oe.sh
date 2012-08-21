#!/bin/sh

# Uncomment one of the following lines and adjust the path for your local
# installation locations.

# GumStix "Open Embedded" build environment for Overo (on an x86_64 system)
export PATH=${PATH}:/home/curt/Projects/OveroOE/tmp/sysroots/x86_64-linux/usr/armv7a/bin

echo "PATH set to: ${PATH}"
echo

echo "Recommended build commands:"
echo ""
echo "mkdir build_overo-oe"
echo "cd build_overo-oe"
echo "../configure CC=arm-angstrom-linux-gnueabi-gcc CFLAGS=\"-Wall -O3\" CXX=arm-angstrom-linux-gnueabi-g++ CXXFLAGS=\"-Wall -O3\" --host=arm-angstrom-linux-gnueabi --prefix=/usr/local/ugear.overo-oe/"
echo "make"
