#!/usr/bin/env python
import rospy
import subprocess
import time
from move.msg import commond

cmd = 'totem cigarette_cm.mp4'

#subprocess.call('totem cigarette_cm.mp4', shell=True)
subprocess.call(['totem', '/home/ubuntu/catkin_ws/src/voice/scripts/cigarette_cm.mp4'])

