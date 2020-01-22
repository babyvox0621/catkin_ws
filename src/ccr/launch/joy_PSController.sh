#!/bin/bash

source /home/ubuntu/catkin_ws/devel/setup.bash

case $1 in
    start)
	roslaunch rulo joy_PSController.launch &
	echo $! > /tmp/joy_PSController.pid
	;;
    stop)
        kill -TERM `cat /tmp/joy_PSController.pid`
	rm /tmp/joy_PSController.pid
	;;
    *)
	;;
esac

        

