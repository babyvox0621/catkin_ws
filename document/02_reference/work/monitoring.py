#!/usr/bin/env python
import paho.mqtt.client as mqtt
import json
import subprocess
#https://qiita.com/msquare33/items/9f0312585bb4707c686b

TOKEN = "token_c0XbHqVV7YsANJjN"
HOSTNAME = "mqtt.beebotte.com"
PORT = 8883
TOPIC = "Digital2019/VoiceRecognition"
CACERT = "mqtt.beebotte.com.pem"

def run_rosscript(pkg,filename):
    subprocess.Call(['source', '/home/catkin/devel/setup.bash']) # source workspace path
    run = subprocess.Popen(['rosrun', pkg,filename])


def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    data = json.loads(msg.payload.decode("utf-8"))["data"][0]
    data = {key:value.strip() for key, value in data.items()}
    print(data)
    #if "beer" in data.value():
    run_rosscript('voice', 'test.py')
    print('action : beer')
        

client = mqtt.Client()
client.username_pw_set("token:%s"%TOKEN)
client.on_connect = on_connect
client.on_message = on_message
client.tls_set(CACERT)
client.connect(HOSTNAME, port=PORT, keepalive=60)
client.loop_forever()
