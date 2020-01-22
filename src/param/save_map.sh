#!/bin/sh

param=`rospack find navi_param`/map
cd $param

if [ -f default.yaml ]; then
    /bin/mv default.yaml default.yaml.bak
fi
if [ -f default.pgm ]; then
    /bin/mv default.pgm default.pgm.bak
fi

rosrun map_server map_saver -f default
