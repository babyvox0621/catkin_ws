//dummy code for arm
#include <math.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <actionlib/server/simple_action_server.h>

#include <TOP/commond.h>

class dummy_arm{
public:
    dummy_arm();
    ~dummy_arm();
    void TOPCallback(const TOP::commond::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_arm;
    ros::Publisher pub_arm;
};

dummy_arm::dummy_arm(){
    sub_arm = nh.subscribe<TOP::commond>("Commond_TOP", 1, &dummy_arm::TOPCallback, this);
    pub_arm = nh.advertise<TOP::commond>("results_ARM", 1);
}

dummy_arm::~dummy_arm(){

}

void dummy_arm::TOPCallback(const TOP::commond::ConstPtr& msg){
    //ROS_INFO("callback dummy_arm");
    if(msg->node == 3 || msg->node == 0){//commond for arm
        //ROS_INFO("topic_id confirmed dummy_arm");
        TOP::commond send;
        send.topic_id = msg->topic_id;
        ROS_INFO("%d", msg->topic_id);
        //-------- arm main process ---------
        //------ add source here ------------
        send.ret = 0;//sucess or failed
        //-----------------------------------
        ROS_INFO("arm moved");
        pub_arm.publish(send);
    }
    else{// commond for other nodes
        ROS_INFO("this topic is not for arm");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dummy_arm");
    ros::NodeHandle nh("~");
    dummy_arm dummy_arm;
    ROS_INFO("runing...");
    ros::spin();
    return 0;
}