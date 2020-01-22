//dummy code for rulo
#include <math.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <actionlib/server/simple_action_server.h>

#include <TOP/commond.h>

class dummy_rulo{
public:
    dummy_rulo();
    ~dummy_rulo();
    void TOPCallback(const TOP::commond::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rulo;
    ros::Publisher pub_rulo;
};

dummy_rulo::dummy_rulo(){
    sub_rulo = nh.subscribe<TOP::commond>("Commond_TOP", 1, &dummy_rulo::TOPCallback, this);
    pub_rulo = nh.advertise<TOP::commond>("results_RULO", 1);
}

dummy_rulo::~dummy_rulo(){

}

void dummy_rulo::TOPCallback(const TOP::commond::ConstPtr& msg){
    //ROS_INFO("callback dummy_rulo");
    if(msg->node == 2 || msg->node == 0){// commond for rulo
        //ROS_INFO("topic_id confirmed dummy_rulo");
        TOP::commond send;
        send.topic_id = msg->topic_id;
        ROS_INFO("%d", msg->topic_id);
        //-------- rulo main process ------------
        send.ret = 0;//sucess or failed
        switch (msg->msg.id){
          case 0://ランダム移動
            //-------- main source --------------
            // add source here
            //-----------------------------------
            ROS_INFO("moving without goal");
            break;
          case 1://目的地に移動
            //-------- main source --------------
            // add source here
            //-----------------------------------
            ROS_INFO("moving with goal %f %f %f", msg->msg.x3d, msg->msg.y3d, msg->msg.z3d);
            break;
          case 2://微調整
            //-------- main source --------------
            // add source here
            //-----------------------------------
            ROS_INFO("position adjustment %d %d", msg->msg.x2d, msg->msg.y2d);
            break;
          default://invalid move mode
            ROS_INFO("invalid move mode @RULO");
            break;
        }
        //---------------------------------------
        pub_rulo.publish(send);
    }
    else{// commond for other nodes
        ROS_INFO("this topic is not for rulo");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "dummy_rulo");
    ros::NodeHandle nh("~");
    dummy_rulo dummy_rulo;
    ROS_INFO("runing...");
    ros::spin();
    return 0;
}