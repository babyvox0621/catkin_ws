//dummy code for yolo
#include <math.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <actionlib/server/simple_action_server.h>

#include <TOP/commond.h>

class dummy_yolo{
public:
    dummy_yolo();
    ~dummy_yolo();
    void TOPCallback(const TOP::commond::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_yolo;
    ros::Publisher pub_yolo;
};

dummy_yolo::dummy_yolo(){
    sub_yolo = nh.subscribe<TOP::commond>("Commond_TOP", 1, &dummy_yolo::TOPCallback, this);
    pub_yolo = nh.advertise<TOP::commond>("results_YOLO", 1);
}

dummy_yolo::~dummy_yolo(){

}

void dummy_yolo::TOPCallback(const TOP::commond::ConstPtr& msg){
    //ROS_INFO("callback dummy_yolo");
    if(msg->node == 1 || msg->node == 0){//commond for yolo
        //ROS_INFO("topic_id confirmed dummy_yolo");
        TOP::commond send;
        send.topic_id = msg->topic_id;
        ROS_INFO("%d", msg->topic_id);
        //----------- yolo main process ----------
        send.ret = 0;//success or failed
        if(msg->msg.id == 0){
            send.msg.x3d = 1.f;
            send.msg.y3d = 2.f;
            send.msg.z3d = 3.f;
            send.msg.x2d = 4;
            send.msg.y2d = 5;
        }
        else{
            send.msg.x3d = -1.f;
            send.msg.y3d = -2.f;
            send.msg.z3d = -3.f;
            send.msg.x2d = -4;
            send.msg.y2d = -5;
        }
        //-----------------------------------------
        ROS_INFO("%f %f %f %d %d", send.msg.x3d, send.msg.y3d, send.msg.z3d, send.msg.x2d, send.msg.y2d);
        pub_yolo.publish(send);
    }
    else{//commond for other nodes
        ROS_INFO("this topic is not for yolo");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dummy_yolo");
    ros::NodeHandle nh("~");
    dummy_yolo dummy_yolo;
    ROS_INFO("runing...");
    ros::spin();
    return 0;
}