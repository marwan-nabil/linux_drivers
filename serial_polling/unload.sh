#!/bin/sh
module=serp

rmmod $module || exit 1
# remove nodes
rm -f /dev/${module} /dev/${module}?
