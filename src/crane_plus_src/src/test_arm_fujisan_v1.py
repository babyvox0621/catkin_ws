#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import write_char_v1 as write_char


class write_text:
    ########################################
    #INIT
    def __init__(self):
        ARM_ON=False
        GRAPH_ON=True

        self.wc=write_char.write_char(ARM_ON,GRAPH_ON)
        self.setup_param()
        self.main_loop()

    ########################################
    #パラメータ初期設定
    def setup_param(self):
        self.wc.HeightDown=0.090
        self.wc.FontSize=0.07
        self.wc.Rotate=0

    ########################################
    #メイン処理
    def main_loop(self):

        #富士山(文字)
        self.wc.FontSize=0.04
        x=self.wc.x_Right-self.wc.FontSize

        y=self.wc.y_Top-0.01
        data = np.load("./fontdata/sample1/fu.npy")
        self.wc.write_char(data,x,y)
        
        y=y-self.wc.FontSize
        data = np.load("./fontdata/sample1/ji.npy")
        self.wc.write_char(data,x,y)

        y=y-self.wc.FontSize
        data = np.load("./fontdata/sample1/san.npy")
        self.wc.write_char(data,x,y)

        #富士山(絵)
        self.wc.FontSize=0.10
        x=x-self.wc.FontSize

        y=self.wc.y_Top-0.04
        data = np.load("./fontdata/sample1/fujisan.npy")
        self.wc.write_char(data,x,y)

        #チーム浅草
        self.wc.FontSize=0.03
        x=x-self.wc.FontSize

        y=self.wc.y_Top-0.03
        data = np.load("./fontdata/sample1/team.npy")
        self.wc.write_char(data,x,y)
        
        y=y-self.wc.FontSize
        data = np.load("./fontdata/sample1/asa.npy")
        self.wc.write_char(data,x,y)

        data = np.load("./fontdata/sample1/kusa.npy")
        y=y-self.wc.FontSize
        self.wc.write_char(data,x,y)

        key = raw_input('Please enter to finish.')
        exit()

       
if __name__ == '__main__':
 
    try:
        ts = write_text()
        rospy.spin()
    except rospy.ROSInterruptException: pass
