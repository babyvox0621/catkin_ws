#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from time import sleep
import cv2

import move_arm_v1 as move_arm


class write_char:
    ########################################
    #INIT
    def __init__(self,ARM_ON,GRAPH_ON):
        #ARM制御ONモード[True/False]
        self.ARM_ON=ARM_ON
        #画面描画ONモード[True/False]
        self.GRAPH_ON=GRAPH_ON

        #ARM制御準備
        if self.ARM_ON==True:
            self.arm=move_arm.move_arm()
            self.arm.Step=100
            self.arm.WaitMove=False

        #パラメータ初期設定
        self.setup_param()

        if self.ARM_ON==True:
            self.arm.Step=100
            self.arm.x=(self.x_Left+self.x_Right)/2
            self.arm.y=(self.y_Top+self.y_Bottom)/2
            self.arm.z=self.HeightDown+0.02
            self.arm.grip(0)
            self.arm.move_xyz()

        
    ########################################
    #パラメータ初期設定
    def setup_param(self):
        #描画サイズの定義
        self.CampusSizeX=0.20
        self.CampusSizeY=0.14
        self.CampusOffsetX=-0.10
        self.CampusOffsetY=+0.14
        self.x_Left   = self.CampusOffsetX
        self.x_Right  = self.CampusOffsetX + self.CampusSizeX
        self.y_Top    = self.CampusOffsetY + self.CampusSizeY
        self.y_Bottom = self.CampusOffsetY

        #文字制御関連
        self.FontSize=0.02
        self.Rotate=0 #文字回転(0-3)
        self.DeltaTh=0.002 #移動処理スキップの閾値

        #ペン高さ設定
        self.HeightDown=0.100
        self.HeightDelta=0.02

        #画面描画用の設定
        self.Ratio=2000  #[dot/m]
        self.ImageSizeX=int(self.CampusSizeX * self.Ratio)
        self.ImageSizeY=int(self.CampusSizeY * self.Ratio)
        self.Img = np.zeros((self.ImageSizeY, self.ImageSizeX, 3), np.uint8)
        self.Img[:,:,:]=(255,255,255)
        self.Color = (0, 0, 0)
        self.Thickness = 2
        if self.GRAPH_ON==True:
            cv2.namedWindow("img")
            cv2.imshow("img", self.Img)
            cv2.waitKey(10)
        
    ########################################
    #1文字分の文字書き関数
    def write_char(self,VectData,OffsetX,OffsetY):

        ########################################
        #初期設定
        self.HeightUp  = self.HeightDown + self.HeightDelta
        z=self.HeightUp
        x0=0
        y0=0

        #Step数制御
        Step_atFirst=100 #初期移動
        Step_atJump=10 #一筆間の移動
        Step_atWrite=1 #筆記
        Step_atPenUp=50 #PEN上げ動作
        Step_atPenDown=50 #PEN下げ動作
        if self.ARM_ON==True:
            self.arm.Step=Step_atFirst

        ####################
        #一筆分ずつデータ取得
        for VectData0 in VectData:
            #NULLデータの場合は処理スキップ
            if VectData0[0][0]==-100: continue
            print VectData0
                        
            ####################
            #サンプリング単位のデータ取得
            for VectData1 in VectData0: 
                #NULLデータの場合は処理スキップ
                if VectData1[0]==-100: continue
                print VectData1

                #座標算出
                #rotate 0
                x= (VectData1[0]/100.0) * self.FontSize + OffsetX
                y=-(VectData1[1]/100.0) * self.FontSize + OffsetY
                #rotate 90
                if self.Rotate==1:
                    x= (VectData1[1]/100.0) * self.FontSize + OffsetX
                    y= (VectData1[0]/100.0) * self.FontSize + OffsetY
                #rotate 180
                if self.Rotate==2:
                    x=-(VectData1[0]/100.0) * self.FontSize + OffsetX
                    y= (VectData1[1]/100.0) * self.FontSize + OffsetY
                #rotate 270
                if self.Rotate==3:
                    x=-(VectData1[1]/100.0) * self.FontSize + OffsetX
                    y=-(VectData1[0]/100.0) * self.FontSize + OffsetY

                #移動量が小さい場合はスキップ
                if abs(x-x0)+abs(y-y0)<self.DeltaTh: continue

                #アームを動かす
                if self.ARM_ON==True:
                    self.arm.x=x
                    self.arm.y=y
                    self.arm.z=z
                    self.arm.move_xyz()

                #高さがdownモードの時は、画面にもライン描画
                if z==self.HeightDown :
                    #描画用座標算出してラインを引く
                    lx0=int((x0-self.CampusOffsetX)*self.Ratio)
                    lx =int((x -self.CampusOffsetX)*self.Ratio)
                    ly0=self.ImageSizeY-int((y0-self.CampusOffsetY)*self.Ratio)
                    ly =self.ImageSizeY-int((y -self.CampusOffsetY)*self.Ratio)
                    if self.GRAPH_ON==True:
                        cv2.line(self.Img, (lx0,ly0), (lx,ly), self.Color, self.Thickness)
                        cv2.imshow("img", self.Img)
                        cv2.waitKey(1)

                #高さがupモードの時は、x,y移動後にペンを下げる
                if z==self.HeightUp :
                    sleep(0.5)
                    z=self.HeightDown
                    if self.ARM_ON==True:
                        self.arm.Step=Step_atPenDown
                        self.arm.z=z
                        self.arm.move_xyz()
                        self.arm.Step=Step_atWrite
                        sleep(0.5)
                        self.arm.WaitMove=True

                #次回参照用のx,y座標を退避
                x0=x
                y0=y

            #一筆分が終了したら、ペンを上げる
            sleep(0.5)
            z=self.HeightUp
            if self.ARM_ON==True:
                self.arm.WaitMove=False
                self.arm.z=z
                self.arm.Step=Step_atPenDown
                self.arm.move_xyz()
                self.arm.Step=Step_atJump
                sleep(0.5)

        #一文字書き終わり
        if self.ARM_ON==True:
            self.arm.Step=Step_atFirst

        
if __name__ == '__main__':
 
    try:
        ts = write_char()
        rospy.spin()
    except rospy.ROSInterruptException: pass
