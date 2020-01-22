#! /usr/bin/env python

import sys
sys.path.append("/home/ubuntu/crane_plus_ws/src/dynamixel_motor/dynamixel_controllers/src/dynamixel_controllers")

from dynamixel_driver import dynamixel_io
import joint_torque_controller as tc

dxl_io = dynamixel_io.DynamixelIO('/dev/ttyUSB0',1000000)

jtc=[0,0,0,0,0,0]
jtc[1]=tc.JointTorqueController(dxl_io,"tilt1_controller","pan_tilt_port")
jtc[2]=tc.JointTorqueController(dxl_io,"tilt2_controller","pan_tilt_port")
jtc[3]=tc.JointTorqueController(dxl_io,"tilt3_controller","pan_tilt_port")
jtc[4]=tc.JointTorqueController(dxl_io,"tilt4_controller","pan_tilt_port")
jtc[5]=tc.JointTorqueController(dxl_io,"tilt5_controller","pan_tilt_port")

for i in range(1,6):
    jtc[i].initialize()

print(jtc[1].MAX_VELOCITY)
print(jtc[1].MIN_VELOCITY)


for i in range(1,6):
    jtc[i].set_compliance_margin(0)
    jtc[i].set_compliance_slope(32)
    jtc[i].set_compliance_punch(0)

    #to default
    #jtc[i].set_compliance_margin(1)
    #jtc[i].set_compliance_slope(32)
    #jtc[i].set_compliance_punch(32)

#(memo)
#    def __init__(self, dxl_io, controller_namespace, port_namespace):
#    def initialize(self):
#    def spd_rad_to_raw(self, spd_rad):
#    def set_torque_enable(self, torque_enable):
#    def set_speed(self, speed):
#    def set_compliance_slope(self, slope):
#    def set_compliance_margin(self, margin):
#    def set_compliance_punch(self, punch):
#    def set_torque_limit(self, max_torque):
#    def process_motor_states(self, state_list):
#    def process_command(self, msg):

