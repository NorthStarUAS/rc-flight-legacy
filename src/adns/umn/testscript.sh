#!/bin/sh

BASEDIR=`pwd`

TESTDIR=data_test
REFDIR=data_reference
PROG=${BASEDIR}/nexttest

for i in simulation fgfs1 mnav cut_storch storch; do
    echo Processing $i
    # ( cd ${BASEDIR}/${TESTDIR}; time ${PROG} $i > /dev/null 2>&1 )
    ( cd ${BASEDIR}/${TESTDIR}; time ${PROG} $i )
    if [ "x$1" != "x--no-diff" ]; then
        # diff ${TESTDIR}/${i}.navstate ${REFDIR}/${i}.navstate
        # diff ${TESTDIR}/${i}.cov ${REFDIR}/${i}.cov
        ./navdiff.pl ${TESTDIR}/${i}.navstate ${REFDIR}/${i}.navstate
        ./navdiff.pl ${TESTDIR}/${i}.cov ${REFDIR}/${i}.cov
        ./navdiff.pl ${TESTDIR}/${i}.filter ${REFDIR}/${i}.filter
    fi
done
