#! /usr/bin/env python
# -*- coding: utf-8 -*-
import twitter
import datetime
import os

auth = twitter.OAuth(consumer_key="75xfwFi6WN04n6yeD6FaWic4Z",
consumer_secret="7rrWJTnigZ5hiEIJQZSkMfp06abPgb3yxXlqUGzOD9AlSne7yQ",
token="1229336994158768128-eGt4RvEQuh4vurRCBMuortV7uuZiyn",
token_secret="Xz6ahyqDlKQXsLwwnK1JEq5SAFWEavIEwYBVSe6Zlp3Q9")

t = twitter.Twitter(auth=auth)
dt_now = datetime.datetime.now()

#ツイートのみ
message = "禁酒大失敗。。。"
status= message + "----" + dt_now.strftime('%Y-%m-%d %H:%M:%S')#投稿するツイート
t.statuses.update(status=status) #Twitterに投稿

cwd = os.getcwd()

#画像付きツイート
#pic=cwd + "/" + "lena.png"#画像を投稿するなら画像のパス
pic = "/home/ubuntu/catkin_ws/video/drink.jpg"
with open(pic,"rb") as image_file:
	image_data=image_file.read()
#print(image_data.type)
pic_upload = twitter.Twitter(domain='upload.twitter.com',auth=auth)
id_img1 = pic_upload.media.upload(media=image_data)["media_id_string"]
t.statuses.update(status=status,media_ids=id_img1)
#t.statuses.update(status=status,media_ids=",".join([id=img1]))
