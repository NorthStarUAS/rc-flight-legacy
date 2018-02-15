# Quick tips on using Qemu

Enables cross installing native compilers and libraries and using them
to cross compile software.  This without the memory or space or cpu
limits of a small embedded cpu.

Reference: https://mmatyas.github.io/blog/servo-short-cross-compilation-guide

## For Fedora

    dnf install debootstrap qemu qemu-user-static
    dnf provides '*/qemu-debootstrap'
    qemu-debootstrap --arch=armhf --verbose jessie rootfs-jessie-armhf

Go get some coffee ...

    $ sudo chroot rootfs-jessie-armhf

    sh# apt install automake make git g++ libeigen3-dev 
    sh# apt install python-dev libboost-python-dev python-serial python-lxml



