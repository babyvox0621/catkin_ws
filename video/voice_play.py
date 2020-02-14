#!/usr/bin/env python
import subprocess
import time
import sys

args = sys.argv
print(args[1])
filename = args[1]
wait_time = float(args[2])

totem = subprocess.Popen(['totem', filename])
time.sleep(wait_time)
totem.kill()

