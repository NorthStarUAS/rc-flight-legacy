#!/bin/sh

sudo date -s "`ssh curt@192.168.7.1 date -u`"
