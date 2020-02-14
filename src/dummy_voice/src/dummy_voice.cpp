//dummy code for voice
#include <math.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <actionlib/server/simple_action_server.h>

#include <TOP/commond.h>

class dummy_voice{
public:
    dummy_voice();
    ~dummy_voice();
    void TOPCallback(const TOP::commond::ConstPtr& msg);
    void GOOGLECallback(const TOP::commond::ConstPtr& msg);
public:
    bool flag_1;
    bool flag_2;
    bool flag_3;
    bool flag_4;
    int topic_id;
    ros::NodeHandle nh;
    ros::Subscriber sub_voice, sub_google;
    ros::Publisher pub_voice;
};

dummy_voice::dummy_voice(){
    sub_voice = nh.subscribe<TOP::commond>("Commond_TOP", 1, &dummy_voice::TOPCallback, this);
    sub_google = nh.subscribe<TOP::commond>("results_GOOGLE", 1, &dummy_voice::GOOGLECallback, this);
    pub_voice = nh.advertise<TOP::commond>("results_VOICE", 1);
    flag_1 = false;
    flag_2 = false;
    flag_3 = false;
    flag_4 = false;
    topic_id = -1;
}

dummy_voice::~dummy_voice(){

}
void dummy_voice::GOOGLECallback(const TOP::commond::ConstPtr& msg){
    if(msg->node == 4 || msg->node == 0){
        if(msg->msg.id == 1){//beer
            flag_1 = true;
            flag_2 = false;
        }
        else if(msg->msg.id == 2){//drink
            flag_1 = false;
            flag_2 = true;
        }
        else{
            ROS_INFO("wrong msg id from google");
        }
    }
    else{
        ROS_INFO("google published a wrong node number");
    }
}

void dummy_voice::TOPCallback(const TOP::commond::ConstPtr& msg){
    //ROS_INFO("callback dummy_voice");
    if(msg->node == 4 || msg->node == 0){//commond for voice
        topic_id = msg->topic_id;
        if(msg->msg.id == 0){//waiting
            flag_3 = true;
            flag_4 = false;
        }
        else if(msg->msg.id == 1){
            flag_3 = false;
            flag_4 = true;
        }
        else{
            ROS_INFO("wrong msg id from TOP");
        }
    }
    else{//commond for other nodes
        ROS_INFO("this topic is not for voice");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dummy_voice");
    ros::NodeHandle nh("~");
    dummy_voice dummy_voice;
    ROS_INFO("runing...");
    ros::Rate loop_rate(5);
    TOP::commond send;
    while(ros::ok())
    {
        ros::spinOnce();
        if(dummy_voice.flag_1&&dummy_voice.flag_3&&(!dummy_voice.flag_2)&&(!dummy_voice.flag_4)){
            send.topic_id = dummy_voice.topic_id;
            send.ret = 0;
            dummy_voice.flag_1 = false;
            dummy_voice.flag_2 = false;
            dummy_voice.flag_3 = false;
            dummy_voice.flag_4 = false;
            dummy_voice.pub_voice.publish(send);
        }
        else if(dummy_voice.flag_2&&dummy_voice.flag_4&&(!dummy_voice.flag_1)&&(!dummy_voice.flag_3)){
            send.topic_id = dummy_voice.topic_id;
            send.ret = 0;
            dummy_voice.flag_1 = false;
            dummy_voice.flag_2 = false;
            dummy_voice.flag_3 = false;
            dummy_voice.flag_4 = false;
            dummy_voice.pub_voice.publish(send);
        }
        else{
            ROS_INFO("no topic received");
        }
        loop_rate.sleep();
    }

    return 0;
}