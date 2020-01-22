#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import math
import numpy as np
from time import sleep
import cv2


#ARMパラメータ
L_B2 = 0.0760	# base_link〜Shoulder(id2)のLink長さ
L_23 = 0.0825	# Shoulder(id2)～Elbow(id3)のLink長さ
L_34 = 0.0940	# Elbow(id3)〜Wrist(id4)のLink長さ
L_4G = 0.1000	# Wrist(id4)〜Gripper_link先のLink長さ
M_PI = math.pi  #3.14

#座標変数
x=0.00
y=0.15
z=0.15
x0=x
y0=y
z0=z
wrist_arg = 0

#SERVO制御変数
t1=0
t2=0
t3=0
t4=0
t5=0


 
class test_arm:
    def __init__(self):
        global x,y,z,t1,t2,t3,t4,t5,wrist_arg,M_PI
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


        #描画サイズの定義
        CampusSizeX=0.20
        CampusSizeY=0.14
        CampusOffsetX=-0.10
        CampusOffsetY=+0.14
        FontSize=0.04

        #高さ設定
        height_down=0.100
        height_up  =height_down + 0.020

        #画面描画用の設定
        Ratio=2000
        ImageSizeX=int(CampusSizeX*Ratio)
        ImageSizeY=int(CampusSizeY*Ratio)
        img = np.zeros((ImageSizeY, ImageSizeX, 3), np.uint8)
        img[:,:,:]=(255,255,255)
        color = (0, 0, 0)
        thickness = 2
        cv2.namedWindow("img")

        #筆跡データ読込（ファイル名は固定の暫定対応）
        data = np.load("/home/ubuntu/moji.npy")

        ########################################
        #メイン処理
        z=height_up
        x0=0
        y0=0

        ####################
        #一筆分のデータ取得
        for data0 in data:
            #NULLデータの場合は処理スキップ
            if data0[0][0]==-100:
                continue
            print data0
                        
            ####################
            #一筆分のデータ取得
            for data1 in data0: 
                #NULLデータの場合は処理スキップ
                if data1[0]==-100:
                    continue
                print data1


                #座標算出
                x= (data1[0]/100.0)*FontSize + CampusOffsetX
                y=-(data1[1]/100.0)*FontSize + CampusOffsetY +FontSize

                #移動量が小さい場合はスキップ
                if abs(x-x0)+abs(y-y0)<0.002:
                    continue

                #アームを動かす
                self.trans_pos()
                self.move_arm([0,t1,t2,t3,t4,t5],1)

                #高さがdownモードの時は、画面にもライン描画
                if z==height_down :
                    #描画用座標算出してラインを引く
                    lx0=int((x0-CampusOffsetX)*Ratio)
                    lx =int((x -CampusOffsetX)*Ratio)
                    ly0=ImageSizeY-int((y0-CampusOffsetY)*Ratio)
                    ly =ImageSizeY-int((y -CampusOffsetY)*Ratio)
                    cv2.line(img, (lx0,ly0), (lx,ly), color, thickness)
                    cv2.imshow("img", img)
                    cv2.waitKey(1)
                    print(lx0,lx,ly0,ly)

                #高さがupモードの時は、x,y移動後にペンを下げる
                if z==height_up :
                    sleep(1.0)
                    z=height_down
                    self.trans_pos()
                    self.move_arm([0,t1,t2,t3,t4,t5],10)

                #参照用のx,y座標を退避
                x0=x
                y0=y

            #一筆分が終了したら、ペンを上げる
            sleep(1.0)
            z=height_up
            self.trans_pos()
            self.move_arm([0,t1,t2,t3,t4,t5],10)

        #処理が終ったら、キー入力を待って終了
        print("Please turn key to finish.")
        cv2.waitKey(0)
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

    #座標->SERVO角度変換関数
    def trans_pos(self):
        global x,y,z,t1,t2,t3,t4,t5,wrist_arg,x0,y0,z0,wrist_arg0
        err_flg = 0

        rospy.loginfo("x,y,z,wrist_arg : [%6.3f,%6.3f,%6.3f,%6.3f]", x, y, z, wrist_arg)

        #角度計算
        #http://ogimotokin.hatenablog.com/entry/2018/04/06/001340
        R_24 = math.sqrt(math.pow(x,2)+math.pow(y,2)) -L_4G*math.cos(wrist_arg);
        Z_24 = z + L_4G*math.sin(wrist_arg)- L_B2

        theta_2d_tmp0 = math.pow(L_23,2)-math.pow(L_34,2)+math.pow(R_24,2)+math.pow(Z_24,2)
        theta_2d_tmp1 = 2*L_23*math.sqrt(math.pow(R_24,2)+math.pow(Z_24,2))
        if -1 <= (theta_2d_tmp0/theta_2d_tmp1) <= 1 :
            theta_2d = math.acos( theta_2d_tmp0 / theta_2d_tmp1 )
        else:
            err_flg=1

        theta_3d_tmp0 = math.pow(L_34,2)-math.pow(L_23,2)+math.pow(R_24,2)+math.pow(Z_24,2)
        theta_3d_tmp1 = 2*L_34*math.sqrt(math.pow(R_24,2)+math.pow(Z_24,2))
        if -1 <= (theta_2d_tmp0/theta_2d_tmp1) <= 1 :
            theta_3d = math.acos( theta_3d_tmp0 / theta_3d_tmp1 )
        else:
            err_flg=1

        theta_4d = math.atan2(Z_24, R_24);

        #計算ErrorがなければSERVO角度を更新
        if err_flg==0:
            t1 = math.atan2(y,x)-M_PI/2
            t2 = -(theta_2d + math.atan2(Z_24, R_24)-M_PI/2)
            t3 = -(theta_2d + theta_3d)
            t4 = -(theta_4d - theta_3d + wrist_arg)
            t5=t5
            x0=x
            y0=y
            z0=z
            wrist_arg0=wrist_arg
        #計算Errorの場合は座標もひとつ前の状態に戻す
        else:
            rospy.logerr("trans error")
            x=x0
            y=y0
            z=z0
            wrist_arg=wrist_arg0


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
