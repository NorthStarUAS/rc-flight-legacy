#!/bin/sh

# set the correct cross compiler.  This needs to be in the default path
# echo Default C compiler is \"${CC:="arm-angstrom-linux-gnueabi-gcc"}\"
# echo Default C++ compiler is \"${CXX:="arm-angstrom-linux-gnueabi-g++"}\"

# make automake happy
if [ ! -f COPYING ]; then
    echo "linking COPYING to LICENSE"
    ln -s LICENSE COPYING
fi
if [ ! -f README ]; then
    echo "linking README to README.md"
    ln -s README.md README
fi
if [ ! -f NEWS ]; then
    echo "linking NEWS to NEWS.md"
    ln -s NEWS.md NEWS
fi

echo "Running aclocal"
aclocal

echo "Running autoheader"
AH_RESULT="src/include/aura_config.h.in"
autoheader
if [ ! -e "$AH_RESULT" ]; then
    echo "ERROR: autoheader didn't create $AH_RESULT!"
    exit 1
fi    

echo "Running automake --add-missing"
automake --add-missing

echo "Running autoconf"
autoconf

if [ ! -e configure ]; then
    echo "ERROR: configure was not created!"
    exit 1
fi

echo ""
echo "======================================"

echo ""
echo "Now you are ready to run:"
echo ""
echo "  mkdir build; cd build"
echo "  ../configure --prefix=/root CFLAGS=\"-Wall -O3\" CXXFLAGS=\"-Wall -O3\""
echo ""
echo "or perhaps:"
echo ""
echo "  mkdir build_arm; cd build_arm"
echo   "../configure --prefix=/root CC=arm-linux-gnueabi-gcc CFLAGS=\"-Wall -O3\" CXX=arm-linux-gnueabi-g++ CXXFLAGS=\"-Wall -O3\" --host arm-linux-gnueabi"
