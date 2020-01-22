#!/bin/bash

source /home/ubuntu/catkin_ws/devel/setup.bash

case $1 in
    start)
	roslaunch rulo joy_F710.launch &
	echo $! > /tmp/joy_F710.pid
	;;
    stop)
        kill -TERM `cat /tmp/joy_F710.pid`
	rm /tmp/joy_F710.pid
	;;
    *)
	;;
esac

        

