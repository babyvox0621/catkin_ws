#!/bin/bash
gnome-terminal --geometry=80x20+0+0 --window -e 'bash -c "roslaunch navi_param ccr.launch;exec bash"'
sleep 5s
gnome-terminal --geometry=80x20+1000+0 --window -e 'bash -c "roslaunch move tf_rs.launch;exec bash"'
sleep 5s
gnome-terminal --geometry=80x20+500+0 --window -e 'bash -c "roslaunch navi_param create_map_hokuyo.launch;exec bash"'
gnome-terminal --geometry=80x20+0+400 --window -e 'bash -c "rosrun move rulo.py;exec bash"'
gnome-terminal --geometry=80x20+500+400 --window -e 'bash -c "roslaunch crane_plus_src controller_manager.launch ;exec bash"'
sleep 5s
gnome-terminal --geometry=80x20+1000+400 --window -e 'bash -c "roslaunch crane_plus_src start_tilt_controller.launch;exec bash"'
sleep 5s
gnome-terminal --geometry=80x20+0+800 --window -e 'bash -c "rosrun crane_plus_src arm.py;exec bash"'
gnome-terminal --geometry=80x20+500+800 --window -e 'bash -c "roslaunch realsense2_camera rs_rgbd.launch;exec bash"'
