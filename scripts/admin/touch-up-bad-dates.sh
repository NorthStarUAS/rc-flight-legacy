#!/bin/sh

echo "Current date is:"
date

sleep 5

# find files older than 3653 days (approx 10 years)

echo "This is the 'fix' list..."
find ~/Source/ -mtime +3653 -print | less

echo "about to 'touch' all the files with bad dates ..."
sleep 5

find ~/Source/ -mtime +3653 -exec touch {} \;
