#!/bin/sh 
module=serp

insmod -f ./${module}.ko || exit 1
major=`cat /proc/devices | awk "\\$2==\"$module\" {print \\$1}"| head -n 1`
echo major number is $major
mknod /dev/${module} c $major 0
chmod a+rw /dev/${module}
exit 0
