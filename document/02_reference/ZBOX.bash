#!/bin/bash
gnome-terminal --geometry=80x20+0+0 --window -e 'bash -c "rosrun TOP TOP;exec bash"'
gnome-terminal --geometry=80x20+500+0 --window -e 'bash -c "rosrun merge_coordinate merge image_transport:=compressed;exec bash"'
gnome-terminal --geometry=80x20+1000+0 --window -e 'bash -c "rosrun dummy_voice dummy_voice;exec bash"'
gnome-terminal --geometry=80x20+0+400 --window -e 'bash -c "roslaunch darknet_ros yolo_v3.launch;exec bash"'
gnome-terminal --geometry=80x20+500+400 --window -e 'bash -c "rosrun voice monitoring.py;exec bash"'
