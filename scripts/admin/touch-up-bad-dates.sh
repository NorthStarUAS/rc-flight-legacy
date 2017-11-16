#!/bin/sh

if [ "$#" -ne 1 ]; then
    echo "usage: $0 <path>"
    exit
fi

CUTOFFDATE="2017-06-23"

echo "Scanning $1 for files older than $CUTOFFDATE"
echo "Current date is:"
date
echo "sleeping for 5 seconds ..."
sleep 5

echo "This is the 'fix' list..."
find $1 -not -newermt $CUTOFFDATE -print | less

echo "about to 'touch' all the files with bad dates ..."
sleep 5

find $1 -not -newermt $CUTOFFDATE -exec touch {} \;
