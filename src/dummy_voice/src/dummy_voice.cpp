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

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_voice;
    ros::Publisher pub_voice;
};

dummy_voice::dummy_voice(){
    sub_voice = nh.subscribe<TOP::commond>("Commond_TOP", 1, &dummy_voice::TOPCallback, this);
    pub_voice = nh.advertise<TOP::commond>("results_VOICE", 1);
}

dummy_voice::~dummy_voice(){

}

void dummy_voice::TOPCallback(const TOP::commond::ConstPtr& msg){
    //ROS_INFO("callback dummy_voice");
    if(msg->node == 4 || msg->node == 0){//commond for voice
        //ROS_INFO("topic_id confirmed dummy_voice");
        TOP::commond send;
        send.topic_id = msg->topic_id;
        ROS_INFO("%d", msg->topic_id);
        //----------- voice main process -------------
        //----------- replace the source here --------
        send.ret = 0;//sucess or failed
        int cnt = 20;
        while(!cnt){//wait for voice message
            ROS_INFO("%d", cnt);
            cnt--;
        }
        //--------------------------------------------
        ROS_INFO("voice message received");
        pub_voice.publish(send);
    }
    else{//commond for other nodes
        ROS_INFO("this topic is not for rulo");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dummy_voice");
    ros::NodeHandle nh("~");
    dummy_voice dummy_voice;
    ROS_INFO("runing...");
    ros::spin();
    return 0;
}