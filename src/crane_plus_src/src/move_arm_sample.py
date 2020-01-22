#! /usr/bin/env python
# -*- coding: utf-8 -*-

#move_arm制御プログラム実装例
#  試してみたい項目の "if(0):" を "if(1)" に書き換えてから実行してください
 
import rospy
import move_arm_v1 as move_arm


####################
#move_arm呼出し
arm=move_arm.move_arm()

####################
#xyz座標で移動例(単位:m)
#- 原点はアーム本体の場所（根元）
arm.z=+0.10

#x,y 4象限移動例
if(0):
    arm.x=+0.10
    arm.y=+0.20
    arm.move_xyz()
    
    arm.x=-0.10
    arm.y=+0.20
    arm.move_xyz()
    
    arm.x=+0.10
    arm.y=-0.20
    arm.move_xyz()
    
    arm.x=-0.10
    arm.y=-0.20
    arm.move_xyz()
        
#z 高さ調整例
if(0):
    arm.x=+0.00
    arm.y=+0.20
    arm.z=+0.10
    arm.move_xyz()

    arm.z=+0.20
    arm.move_xyz()

    arm.z=+0.15
    arm.move_xyz()

####################
#手首角度調整例(単位:rad)
#- 手首の角度を変化させてもグリップの高さはzで一定
#- wrist_arg=0で水平、+方向で上向き、-方向で下向き
if(0):
    arm.x=+0.00
    arm.y=+0.20
    arm.z=+0.15
    arm.wrist_arg=+0.5
    arm.move_xyz()

    arm.wrist_arg=-0.5
    arm.move_xyz()

    arm.wrist_arg=+0.0
    arm.move_xyz()

####################
#GRIPの開閉例(単位:%)
if(0):
    arm.x=+0.00
    arm.y=+0.20
    arm.z=+0.15
    arm.move_xyz()

    arm.grip(0)
    arm.grip(100)
    arm.grip(50)
    arm.grip(80)
    arm.grip(0)


####################
#TILT直接指定で移動例(単位:rad)
#- サーボ5chのTILT量を直接指定する
if(0):
    arm.Tilt[1]=0.00
    arm.Tilt[2]=0.00
    arm.Tilt[3]=0.00
    arm.Tilt[4]=0.00
    arm.Tilt[5]=0.00
    arm.move_tilt()

    arm.Tilt[1]=+1.00
    arm.move_tilt()

    arm.Tilt[1]=-1.00
    arm.move_tilt()

####################
#移動量分割例(単位:回)
#- 指定座標までの移動量が大きい時にゆっくり移動させたい場合
#  (一気に移動することはなくなるが、カクカクした動きになるかも)
if(0):
    arm.x=+0.00
    arm.y=+0.20
    arm.z=+0.10
    arm.move_xyz()

    arm.Step=20
    arm.z=+0.20
    arm.move_xyz()

    arm.Step=5
    arm.z=+0.10
    arm.move_xyz()

    arm.Step=1


####################
#移動後の安定待ちON/OFF例(True/False)
#- 移動コマンドが連続発行される場合に、
#  WaitMove=Trueだと、移動が完了するまで次のコマンドを発行されない
#  WaitMove=Falseだと、移動が完了する前でも次のコマンドが発行される
#- 1回の移動量が大きい場合はTrueにしておかないと確実に移動されない
#- 小さい移動を繰返し処理する場合はFalseの方がスムーズに動く

#移動量が大きい場合の比較
if(0):
    arm.x=+0.00
    arm.y=+0.20

    arm.WaitMove=True
    arm.z=+0.20
    arm.move_xyz()
    arm.z=+0.10
    arm.move_xyz()

    arm.WaitMove=False
    arm.z=+0.20
    arm.move_xyz()
    arm.z=+0.10
    arm.move_xyz()

#小さい移動を繰返し処理する場合の比較
if(0):
    arm.y=+0.20
    arm.z=+0.10

    arm.WaitMove=True
    for i in range(-10,+10,1):
        arm.x=i/100.0
        arm.move_xyz()
        
    arm.WaitMove=False
    for i in range(+10,-10,-1):
        arm.x=i/100.0
        arm.move_xyz()


exit()

