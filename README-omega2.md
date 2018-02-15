export STAGING_DIR=/home/curt/Source/Omega2/source/staging_dir
export TOOLCHAIN_DIR=${STAGING_DIR}/toolchain-mipsel_24kc_gcc-5.4.0_musl-1.1.16
export LEDEBIN_DIR=${TOOLCHAIN_DIR}/bin

../configure CFLAGS="-Wall -O3" CXXFLAGS="-Wall -O3" CC=${LEDEBIN_DIR}/mipsel-openwrt-linux-musl-gcc CXX=${LEDEBIN_DIR}/mipsel-openwrt-linux-musl-g++ --host=mips-openwrt-linux-uclibc --prefix=${TOOLCHAIN_DIR}/ --build=mips-openwrt-linux-gnu


TEST (for python in the cross compile tree):

  - ref: https://github.com/OnionIoT/OpenWRT-Packages/wiki/Setting-Up-the-Cross-Compile-Environment
  
  $ scripts/feeds update -a
  $ scripts/feeds install python
  $ make menuconfig
    -> Languages
       -> Python
          -> python (y)
          -> python-devel (y)
    <save>
  $ vi packages/feeds/python/Makefile
    add --enable-optimizations \ to configure options (avoids _gcov* errors)
  $ make -j10


NOTICE:

Cross compiling Python 2.7.x using LEDE project tool chain (and then
adding it to the tool chain install.)

extract python-x.y.tar.gz and cd into the source directory

$ mkdir build; cd build

$ ../configure \
            CC=${LEDEBIN_DIR}/mipsel-openwrt-linux-musl-gcc \
            CXX=${LEDEBIN_DIR}/mipsel-openwrt-linux-musl-g++ \
	    RANLIB=${LEDEBIN_DIR}/mipsel-openwrt-linux-musl-ranlib \
	    AR=${LEDEBIN_DIR}/mipsel-openwrt-linux-musl-ar \
            --host=mips-openwrt-linux-uclibc \
            --prefix=${TOOLCHAIN_DIR} \
            --build=mips-openwrt-linux-gnu \
            --disable-ipv6 \
            --enable-optimizations \
	    --enable-shared \
            ac_cv_file__dev_ptmx=no \
            ac_cv_file__dev_ptc=no \
            ac_cv_have_long_long_format=yes

$ make -j8

- Note 1: make seems to succeed in compiling libpython2.7.a, but fails
  to produce the .so version

- Note 2: make install fails miserably because of some pc host vs. omega
  stuff that us beyond me to figure out.

- manually copied the Include tree from python src to
  source/staging_dir/toolchain-mipsel_24kc_gcc-5.4.0_musl/include/python2.7/
  and manually copied the pyconfig.h file as well.

  $ mkdir ../source/staging_dir/toolchain-mipsel_24kc_gcc-5.4.0_musl-1.1.16/include/python2.7
  $ cp ${python-src}/Include/* ../source/staging_dir/toolchain-mipsel_24kc_gcc-5.4.0_musl-1.1.16/include/python2.7/
  $ cp ${python-src}/pyconfig.h ../source/staging_dir/toolchain-mipsel_24kc_gcc-5.4.0_musl-1.1.16/include/python2.7/

- manually copied the libpython2.7.a file to
  ...source/staging_dir/toolchain-mipsel_24kc_gcc-5.4.0_musl/lib/

  $ cp ${python-build}/libpython2.7.a ../source/staging_dir/toolchain-mipsel_24kc_gcc-5.4.0_musl-1.1.16/lib/

- fiddled with the build system in my own software because autoconf
  couldn't run python-config and get the right answer.

- compiled my code successfully.