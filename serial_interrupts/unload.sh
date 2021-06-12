#!/bin/sh
module=seri

rmmod $module || exit 1
# remove nodes
rm -f /dev/${module} /dev/${module}?
