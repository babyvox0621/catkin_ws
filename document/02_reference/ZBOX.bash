#!/bin/bash
rosrun TOP TOP &

#!/bin/bash
rosrun dummy_voice dummy_voice &

#!/bin/bash
rosrun merge_coordinate merge image_transport:=compressed

#!/bin/bash
roslaunch darknet_ros yolo_v3.launch &





