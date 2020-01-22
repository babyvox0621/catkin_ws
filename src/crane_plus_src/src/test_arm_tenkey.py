#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState


class test_arm:
    def __init__(self):
        rospy.init_node('test_arm', anonymous=True)
        self.tilt1_pub = rospy.Publisher('tilt1_controller/command', Float64, queue_size=1000)
        self.tilt2_pub = rospy.Publisher('tilt2_controller/command', Float64, queue_size=1000)
        self.tilt3_pub = rospy.Publisher('tilt3_controller/command', Float64, queue_size=1000)
        self.tilt4_pub = rospy.Publisher('tilt4_controller/command', Float64, queue_size=1000)
        self.tilt5_pub = rospy.Publisher('tilt5_controller/command', Float64, queue_size=1000)
        rospy.Subscriber('tilt1_controller/state',JointState, self.tilt1_Callback)
        rospy.Subscriber('tilt2_controller/state',JointState, self.tilt2_Callback)
        rospy.Subscriber('tilt3_controller/state',JointState, self.tilt3_Callback)
        rospy.Subscriber('tilt4_controller/state',JointState, self.tilt4_Callback)
        rospy.Subscriber('tilt5_controller/state',JointState, self.tilt5_Callback)

        self.tilt1_state=JointState()
        self.tilt2_state=JointState()
        self.tilt3_state=JointState()
        self.tilt4_state=JointState()
        self.tilt5_state=JointState()

        self.tilt_moving=[0,1,1,1,1,1]
        self.rate = rospy.Rate(100)

        #init
        t1=0
        t2=0.8
        t3=-0.7
        t4=0.5
        t5=0.7
        
        self.move_arm([0,t1,t2,t3,t4,t5],10)
        self.rate.sleep()
        self.wait_move()

        ##loop
        while(1):
            key = raw_input('key=> ')

            if key=="q":
                break
            elif key=="8": #forward
                t2=t2+0.05
                t3=t3+0.10
                t4=t4-0.05
            elif key=="2": #back
                t2=t2-0.05
                t3=t3-0.10
                t4=t4+0.05
            elif key=="6": #right
                t1=t1-0.05
            elif key=="4": #left
                t1=t1+0.05
            elif key=="5": #tilt2 up
                t2=t2-0.05
            elif key=="0": #tilt2 down
                t2=t2+0.05
            elif key=="9": #tilt4 up
                t4=t4-0.05
            elif key=="3": #tilt4 down
                t4=t4+0.05
            elif key=="7": #tilt5 close
                t5=t5+0.05
            elif key=="1": #tilt5 open
                t5=t5-0.05

            #ARM移動
            t1=round(t1,3)
            t2=round(t2,3)
            t3=round(t3,3)
            t4=round(t4,3)
            t5=round(t5,3)
            self.move_arm([0,t1,t2,t3,t4,t5],5)
            
        exit()

    #ARM移動関数
    def move_arm(self, tilt, step):
        t1_0 = self.tilt1_state.current_pos
        t2_0 = self.tilt2_state.current_pos
        t3_0 = self.tilt3_state.current_pos
        t4_0 = self.tilt4_state.current_pos
        t5_0 = self.tilt5_state.current_pos
        t1=(tilt[1] - t1_0) /step
        t2=(tilt[2] - t2_0) /step
        t3=(tilt[3] - t3_0) /step
        t4=(tilt[4] - t4_0) /step
        t5=(tilt[5] - t5_0) /step
        rospy.loginfo("Current:[%6.3f,%6.3f,%6.3f,%6.3f,%6.3f]", t1_0,t2_0,t3_0,t4_0,t5_0)
        rospy.loginfo("Goal   :[%6.3f,%6.3f,%6.3f,%6.3f,%6.3f], Step:%d", tilt[1],tilt[2],tilt[3],tilt[4],tilt[5],step)

        for i in range(1,step+1):
            self.tilt1_pub.publish(t1_0 + t1*i)
            self.tilt2_pub.publish(t2_0 + t2*i)
            self.tilt3_pub.publish(t3_0 + t3*i)
            self.tilt4_pub.publish(t4_0 + t4*i)
            self.tilt5_pub.publish(t5_0 + t5*i)
            self.rate.sleep()

            self.tilt_moving[1] = 1
            self.tilt_moving[2] = 1
            self.tilt_moving[3] = 1
            self.tilt_moving[4] = 1
            self.tilt_moving[5] = 1
            self.rate.sleep()

            self.wait_move()

        t1_1 = self.tilt1_state.current_pos
        t2_1 = self.tilt2_state.current_pos
        t3_1 = self.tilt3_state.current_pos
        t4_1 = self.tilt4_state.current_pos
        t5_1 = self.tilt5_state.current_pos
        rospy.loginfo("Finish :[%6.3f,%6.3f,%6.3f,%6.3f,%6.3f]", t1_1,t2_1,t3_1,t4_1,t5_1)


    #is_movingがFalseになるまでWAIT
    def wait_move(self):
        while (1):
            move_check = 0
            for i in range(1,5):
                move_check = move_check | self.tilt_moving[i]
            if self.tilt_moving[1] ==  0:
                self.rate.sleep()
                break


    #tilt情報を定期的に更新
    def tilt1_Callback(self, tilt1_state):
        self.tilt1_state=tilt1_state
        if tilt1_state.is_moving == True:
            self.tilt_moving[1] = 1
        else:
            self.tilt_moving[1] = 0
    def tilt2_Callback(self, tilt2_state):
        self.tilt2_state=tilt2_state
        if tilt2_state.is_moving == True:
            self.tilt_moving[2] = 1
        else:
            self.tilt_moving[2] = 0
    def tilt3_Callback(self, tilt3_state):
        self.tilt3_state=tilt3_state
        if tilt3_state.is_moving == True:
            self.tilt_moving[3] = 1
        else:
            self.tilt_moving[3] = 0
    def tilt4_Callback(self, tilt4_state):
        self.tilt4_state=tilt4_state
        if tilt4_state.is_moving == True:
            self.tilt_moving[4] = 1
        else:
            self.tilt_moving[4] = 0
    def tilt5_Callback(self, tilt5_state):
        self.tilt5_state=tilt5_state
        if tilt5_state.is_moving == True:
            self.tilt_moving[5] = 1
        else:
            self.tilt_moving[5] = 0
        

if __name__ == '__main__':
 
    try:
        ts = test_arm()
        rospy.spin()
    except rospy.ROSInterruptException: pass
