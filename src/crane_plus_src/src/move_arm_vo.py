#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

class move_arm:
    ########################################
    #INIT
    def __init__(self):
        ####################
        #外部制御変数定義
        #座標変数
        self.x=0.00
        self.y=0.10
        self.z=0.20
        M_PI = math.pi  #3.14
        self.wrist_arg = M_PI/2

        #動作モードオプション
        self.Step=1  #移動量分割数
        self.WaitMove=True  #移動後の安定待ちON/OFF

        #エラーフラグ
        self.Error_Coordinate=False
        
        ####################
        #内部制御変数定義
        #SERVO制御変数(TILT)
        self.Tilt=np.zeros(6,dtype=float)
        #移動中監視
        self.Tilt_Moving=np.zeros(6,dtype=int)

        
        ####################
        #ROS設定
        rospy.init_node('move_arm', anonymous=True)
        self.RosRate = rospy.Rate(100)

        self.Tilt_Pub=[0,0,0,0,0,0]
        self.Tilt_Pub[1] = rospy.Publisher('tilt1_controller/command', Float64, queue_size=1000)
        self.Tilt_Pub[2] = rospy.Publisher('tilt2_controller/command', Float64, queue_size=1000)
        self.Tilt_Pub[3] = rospy.Publisher('tilt3_controller/command', Float64, queue_size=1000)
        self.Tilt_Pub[4] = rospy.Publisher('tilt4_controller/command', Float64, queue_size=1000)
        self.Tilt_Pub[5] = rospy.Publisher('tilt5_controller/command', Float64, queue_size=1000)
        rospy.Subscriber('tilt1_controller/state',JointState, self.Tilt1_Callback)
        rospy.Subscriber('tilt2_controller/state',JointState, self.Tilt2_Callback)
        rospy.Subscriber('tilt3_controller/state',JointState, self.Tilt3_Callback)
        rospy.Subscriber('tilt4_controller/state',JointState, self.Tilt4_Callback)
        rospy.Subscriber('tilt5_controller/state',JointState, self.Tilt5_Callback)
 
        self.Tilt_State=[0,0,0,0,0,0]
        self.Tilt_State[1]=JointState()
        self.Tilt_State[2]=JointState()
        self.Tilt_State[3]=JointState()
        self.Tilt_State[4]=JointState()
        self.Tilt_State[5]=JointState()

        #初回Topic購読を待ってInit完了(安定待ち)
        self.wait_move()
        for ch in range(1,6):
            self.Tilt[ch]=self.Tilt_State[ch].current_pos


    ########################################
    #GRIP制御動関数(%指定)
    def grip(self,per):
        if per<0:
            per=0
        if per>100:
            per=100
        self.Tilt[5]=-1.4*per/100+0.7
        self.move_tilt()

    ########################################
    #ARM移動関数（座標指定)
    def move_xyz(self):
        #座標->tilt変換してErrorでなければ移動させる
        self.xyz2tilt()
        if self.Error_Coordinate==0:
            self.move_tilt()

            
    ########################################
    #ARM移動関数（TILT指定)
    def move_tilt(self):
        #ローカル変数準備
        Step=self.Step
        t_Goal=self.Tilt
        t_Start=np.zeros(6)
        t_Delta=np.zeros(6)
        t_tmp=np.zeros(6)

        #現在のTilt情報と目標Tilt情報から、1STEPあたりの移動量算出
        for ch in range(1,6):
            t_Start[ch]=self.Tilt_State[ch].current_pos
            t_Delta[ch]=(t_Goal[ch]-t_Start[ch]) / Step

        rospy.loginfo("Current:[%6.3f,%6.3f,%6.3f,%6.3f,%6.3f]", t_Start[1],t_Start[2],t_Start[3],t_Start[4],t_Start[5])
        rospy.loginfo("Goal   :[%6.3f,%6.3f,%6.3f,%6.3f,%6.3f], Step:%d", t_Goal[1],t_Goal[2],t_Goal[3],t_Goal[4],t_Goal[5],Step)

        #Tilt Topicの発行
        #  - 1STEP分ずつ移動させる
        #  - 5ch分のTopicを発行したら目標Tiltに移動完了するまで待つ
        for i in range(1,Step+1):
            for ch in range(1,6):
                t_tmp[ch]=t_Start[ch] + t_Delta[ch]*i
                self.Tilt_Pub[ch].publish(t_tmp[ch])
            rospy.loginfo("Step%03d:[%6.3f,%6.3f,%6.3f,%6.3f,%6.3f]", i,t_tmp[1],t_tmp[2],t_tmp[3],t_tmp[4],t_tmp[5])
            self.RosRate.sleep()
            self.wait_move()
 
        #移動後のTilt情報取得
        t1=np.zeros(6)
        for ch in range(1,6):
            t1[ch]=self.Tilt_State[ch].current_pos
        rospy.loginfo("Finish :[%6.3f,%6.3f,%6.3f,%6.3f,%6.3f]", t1[1],t1[2],t1[3],t1[4],t1[5])

        
    ########################################
    #xyz座標->Tilt(SERVO角度)変換関数
    def xyz2tilt(self):
        #crane2+ ARM距離パラメータ
        L_B2 = 0.0710	# base_link〜Shoulder(id2)のLink長さ
        L_23 = 0.0830	# Shoulder(id2)～Elbow(id3)のLink長さ
        L_34 = 0.0935	# Elbow(id3)〜Wrist(id4)のLink長さ
        L_4G = 0.1250	# Wrist(id4)〜Gripper_link先のLink長さ
        M_PI = math.pi  #3.14

        #ローカル変数準備
        x=self.x
        y=self.y
        z=self.z
        wrist_arg=self.wrist_arg
        #wrist_arg = M_PI/2
        err_flg = False
 
        rospy.loginfo("x,y,z,wrist_arg : [%6.3f,%6.3f,%6.3f,%6.3f]", x, y, z, wrist_arg)

        #角度計算
        #  参考:http://ogimotokin.hatenablog.com/entry/2018/04/06/001340
        R_24 = math.sqrt(math.pow(x,2)+math.pow(y,2)) -L_4G*math.cos(-wrist_arg);
        Z_24 = z + L_4G*math.sin(-wrist_arg) - L_B2
 
        theta_1d = math.atan2(x,y)
        #theta_1d = math.atan2(y,x) - M_PI/2
        #if theta_1d < -M_PI:
        #    theta_1d+=M_PI*2

        theta_2d_tmp0 = math.pow(L_23,2)-math.pow(L_34,2)+math.pow(R_24,2)+math.pow(Z_24,2)
        #theta_2d_tmp0 = math.pow(L_23,2)-math.sqrt(math.pow(L_34,2)+math.pow(R_24,2))-math.pow(L_34,2)
        theta_2d_tmp1 = 2*L_23*math.sqrt(math.pow(R_24,2)+math.pow(Z_24,2))
        if -1 <= (theta_2d_tmp0/theta_2d_tmp1) <= 1 :
            theta_2d = math.acos( theta_2d_tmp0 / theta_2d_tmp1 )
        else:
            err_flg=True
 
        theta_3d_tmp0 = math.pow(L_34,2)-math.pow(L_23,2)+math.pow(R_24,2)+math.pow(Z_24,2)
        #theta_3d_tmp0 = math.pow(L_34,2)-math.sqrt(math.pow(R_24,2)+math.pow(Z_24,2))-math.pow(L_23,2)
        theta_3d_tmp1 = 2*L_34*math.sqrt(math.pow(R_24,2)+math.pow(Z_24,2))
        if -1 <= (theta_3d_tmp0/theta_3d_tmp1) <= 1 :
            theta_3d = math.acos( theta_3d_tmp0 / theta_3d_tmp1 )
        else:
            err_flg=True
 
        theta_4d = math.atan2(Z_24, R_24);

        #計算ErrorがなければSERVO角度を更新
        if err_flg==False:
            self.Error_Coordinate=False
            #self.Tilt[1] = math.atan2(y,x) -M_PI/2
            self.Tilt[1] = theta_1d
            self.Tilt[2] = -(theta_2d + math.atan2(Z_24, R_24)-M_PI/2)
            self.Tilt[3] = -(theta_2d + theta_3d)
            self.Tilt[4] = -(theta_4d - theta_3d - wrist_arg)
            self.Tilt[5] = self.Tilt[5]
        #計算Errorの場合はエラーフラグを立てる
        else:
            self.Error_Coordinate=True
            rospy.logerr("trans error")


    ########################################
    #移動完了待ち関数
    def wait_move(self):
        #移動中監視変数をセット
        self.Tilt_Moving=np.ones(6,dtype=int)

        #全chの移動が完了したら関数を抜ける
        #(memo)Tilt_Moving[ch] は Tilt*_Callback関数で割込み処理的に更新される
        while (1):
            self.RosRate.sleep()
            MoveFlg = False
            for ch in range(1,6):
                MoveFlg |= self.Tilt_Moving[ch]
            if(MoveFlg==False or self.WaitMove==False):
                break


    ########################################
    #tilt情報を定期的に更新(Topicを購読したら実行される)
    def Tilt1_Callback(self, Tilt_State):
        ch=1
        self.Tilt_State[ch]=Tilt_State
        self.Tilt_Moving[ch]=Tilt_State.is_moving
    def Tilt2_Callback(self, Tilt_State):
        ch=2
        self.Tilt_State[ch]=Tilt_State
        self.Tilt_Moving[ch]=Tilt_State.is_moving
    def Tilt3_Callback(self, Tilt_State):
        ch=3
        self.Tilt_State[ch]=Tilt_State
        self.Tilt_Moving[ch]=Tilt_State.is_moving
    def Tilt4_Callback(self, Tilt_State):
        ch=4
        self.Tilt_State[ch]=Tilt_State
        self.Tilt_Moving[ch]=Tilt_State.is_moving
    def Tilt5_Callback(self, Tilt_State):
        ch=5
        self.Tilt_State[ch]=Tilt_State
        self.Tilt_Moving[ch]=Tilt_State.is_moving


########################################
#おまじない 
if __name__ == '__main__':
    try:
        ts = move_arm()
        rospy.spin()
    except rospy.ROSInterruptException: pass
