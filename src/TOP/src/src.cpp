#include <math.h>
#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <actionlib/server/simple_action_server.h>

#include <TOP/commond.h>
#include <stdlib.h>
//check distance to tell whether trans status or not
#define THRESH_DIST 0.5f
#define THRESH_CENTER 100

class obj_TOP{
public:
    obj_TOP();
    ~obj_TOP();
    void YoloCallback(const TOP::commond::ConstPtr& msg);
    void RuloCallback(const TOP::commond::ConstPtr& msg);
    void ArmCallback(const TOP::commond::ConstPtr& msg);
    void VoiceCallback(const TOP::commond::ConstPtr& msg);
    void SendFirstMsg(void);
 
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_yolo, sub_rulo, sub_arm, sub_voice;
    ros::Publisher pub_commond;
    int topic_id;
    int status;
    int object;
};
//constructor
//init
obj_TOP::obj_TOP(){
	sub_yolo = nh.subscribe<TOP::commond>("results_YOLO", 1, &obj_TOP::YoloCallback, this);
	sub_rulo = nh.subscribe<TOP::commond>("results_RULO", 1, &obj_TOP::RuloCallback, this);
	sub_arm = nh.subscribe<TOP::commond>("results_ARM", 1, &obj_TOP::ArmCallback, this);
	sub_voice = nh.subscribe<TOP::commond>("results_VOICE", 1, &obj_TOP::VoiceCallback, this);
	pub_commond = nh.advertise<TOP::commond>("Commond_TOP",1);
  //init
	topic_id = 0;
	status = 0;
	object = 39;

	//create the msg for publish
	//TOP::commond send;
	//status 0 初期
	status = 2; //trans to 探索
	//send.node = 1; //commond for yolo
	//send.msg.id = object;
	//send.topic_id = topic_id;
	//pub_commond.publish(send);
  ROS_INFO("init done");
}
//deconstructor
//nothing to do
obj_TOP::~obj_TOP(){

}

//callback founction
// yolo callback
void obj_TOP::YoloCallback(const TOP::commond::ConstPtr& msg){
	//create the msg for publish
	TOP::commond send;
  //check topic id
  if(msg->topic_id == topic_id){
  	switch (status){
  		case 0: //初期
  	  case 1: //待機
  	    ROS_INFO("invalid status = %d @Yolo", status);
  	    break;
  	  case 2: //探索
  	  case 3: //移動
  	    //receive and check the results from yolo
  	    if(msg->ret == 0){ //found
          //set commond info
          if(status == 2){
            send.node = 2; //commond for rulo
          	send.msg.id = 1; //moving with goal
          	status = 3; //trans to 移動
            topic_id++;
            send.topic_id = topic_id; //set topic id
            send.msg.x3d = msg->msg.x3d;
            send.msg.y3d = msg->msg.y3d;
            send.msg.z3d = msg->msg.z3d; //3d position
            send.msg.x2d = msg->msg.x2d;
            send.msg.y2d = msg->msg.y2d; //2d position
            if(object == 39){
              system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/found.wav 3");
              //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/going.wav 4");
            }
            pub_commond.publish(send);
          }
          else if(status == 3){
            if(object == 39){
              send.node = 2; //commond for rulo
              if(msg->msg.z3d <= THRESH_DIST){ //close to goal enough
                send.msg.id = 2; //pose adujstment
                status = 4; //trans to 微調整
                //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/tweak.wav 5");
                ROS_INFO("close to goal -> adjustment");
              }
              else{ //still far from goal
                send.msg.id = 1; //moving with goal
                status = 3; //keep status 3
                //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/going.wav 4");
                ROS_INFO("still far -> moving with goal");
              }
              topic_id++;
              send.topic_id = topic_id; //set topic id
              if(object == 39){
                send.msg.key = 0;
              }
              else if(object == 0){
                send.msg.key = 1;
              }
              send.msg.x3d = msg->msg.x3d;
              send.msg.y3d = msg->msg.y3d;
              send.msg.z3d = msg->msg.z3d; //3d position
              send.msg.x2d = msg->msg.x2d;
              send.msg.y2d = msg->msg.y2d; //2d position
              pub_commond.publish(send);
            }
            else if(object == 0){
              status = 6; //trans to PR if object is person
              send.node = 4;
              topic_id++;
              send.topic_id = topic_id;
              send.msg.id = 1;
              system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/advice.wav 6");
              pub_commond.publish(send);
            }
          }
  	    }
  	    else if(msg->ret == 1){ //not found
          status = 2; //trans to 探索
          //set commond info
          send.node = 2; //commond for rulo
          topic_id++;
          send.topic_id = topic_id;
          send.msg.id = 0; //moving without goal (random move)
          ROS_INFO("object lost");
          if(status == 2){
            if(object == 39){
              //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/search.wav 3");
            }
            else if(object == 0){
              //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/people.wav 6");
            }
          }
          else if(status == 3){
            system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/lost.wav 3");
          }
          pub_commond.publish(send);
  	    }
  	    else{ //error code
          ROS_INFO("Yolo: error code = %d  ||  status = %d", msg->ret, status);
  	    }
  	    break;
  	  case 4: //微調整
        //receive and check the results from yolo
        if(msg->ret == 0){ //found
          //set commond info
          if(abs(msg->msg.x2d - 320) <= 100){ //close to center -> adjustment finished
            status = 5; //trans to 把持 if object is タバコ
            //set commond info
            send.node = 3; //commond for arm
            topic_id++;
            send.topic_id = topic_id;
            send.msg.id = 0; //catch
            system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/conceal.wav 3");
            ROS_INFO("adjustment finished");
            pub_commond.publish(send);
          }
          else{ //not close to center still need adjustment
            ROS_INFO("%d adjustment continue...", abs(msg->msg.x2d-320));
            //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/tweak.wav 5");
            send.msg.id = 2; //pose adjustment
            status = 4; //keep status 3
            send.node = 2;
            topic_id++;
            send.topic_id = topic_id; //set topic id
            if(object == 39){
              send.msg.key = 0;
            }
            else if(object == 0){
              send.msg.key = 1;
            }
            send.msg.x3d = msg->msg.x3d;
            send.msg.y3d = msg->msg.y3d;
            send.msg.z3d = msg->msg.z3d; //3d position
            send.msg.x2d = msg->msg.x2d;
            send.msg.y2d = msg->msg.y2d; //2d position
            pub_commond.publish(send);
          }
        }
        else if(msg->ret == 1){ //not found
          ROS_INFO("object lost");
          system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/lost.wav 3");
          status = 2; //trans to 探索
          //set commond info
          send.node = 2; //commond for rulo
          topic_id++;
          send.topic_id = topic_id;
          send.msg.id = 0; //moving without goal (random move)
          if(object == 39){
            //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/search.wav 3");
          }
          else if(object == 0){
            //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/people.wav 6");
          }
          pub_commond.publish(send);
        }
        else{ //error code
          ROS_INFO("Yolo: error code = %d  ||  status = %d", msg->ret, status);
        }
        break;
  	  case 5: //把持
  	  case 6: //PR
  	    ROS_INFO("invalid status = %d @Yolo", status);
  	    break;
  	  default:
  	    ROS_INFO("not exist status @Yolocallback");
  	    break;
  	}
  }
  else{
  	ROS_INFO("wrong topic id @YoloCallback");
  }
}

// rulo callback
void obj_TOP::RuloCallback(const TOP::commond::ConstPtr& msg){
  ros::Rate loop_rate(1);
	//create the msg for publish
	TOP::commond send;
  //check topic id
  if(msg->topic_id == topic_id){
  	switch (status){
  		case 0: //初期
        ROS_INFO("invalid status = %d @Rulo", status);
  	    break;
  	  case 1: //待機
  	  	if(msg->ret == 0){ //done
  	  		status = 1;
  	  		//set commond info
  	  		send.node = 4; //commond for voice
  	  		topic_id++;
  	  		send.topic_id = topic_id;
  	  		send.msg.id = 0; //20s wait...
          ROS_INFO("waiting for voice message...");
  	  		pub_commond.publish(send);
  	  	}
  	  	else{
  	  		ROS_INFO("Rulo: error code = %d  ||  status = %d", msg->ret, status);
  	  	}
  	    break;
  	  case 2: //探索
  	  case 3: //移動
      case 4: //微調整
  	    if(msg->ret == 0){ //done
  	    	//set commond info
  	    	send.node = 1; //commond for yolo
  	    	topic_id++;
  	    	send.topic_id = topic_id;
  	    	send.msg.id = object;
          loop_rate.sleep();
  	    	pub_commond.publish(send);
  	    }
  	    else{ //error code
  	    	ROS_INFO("Rulo: error code = %d  ||  status = %d", msg->ret, status);
  	    }
  	    break;
  	  case 5: //把持
  	  case 6: //PR
        ROS_INFO("invalid status = %d @Rulo", status);
  	    break;
  	  default:
  	    ROS_INFO("not exist status @RuloCallback");
  	    break;
  	}
  }
  else{
  	ROS_INFO("wrong topic id @RuloCallback");
  }
}

// arm callback
void obj_TOP::ArmCallback(const TOP::commond::ConstPtr& msg){
	//create the msg for publish
	TOP::commond send;
  //check topic id
  if(msg->topic_id == topic_id){
  	switch (status){
  		case 0: //初期
  	  case 1: //待機
  	  case 2: //探索
  	  case 3: //移動
  	  case 4: //微調整
  	    ROS_INFO("invalid status = %d @Arm", status);
  	    break;
  	  case 5: //把持
  	    if(msg->ret == 0){
  	    	status = 1;
  	    	//set commond info
  	    	send.node = 2; //commond for rulo
  	    	topic_id++;
  	    	send.topic_id = topic_id;
  	    	send.msg.id = 3; //moving with goal(world coordinate) -> move to the initial position
  	    	send.msg.x3d = 0.f;//orgin.x;
  	    	send.msg.y3d = 0.f;//orgin.y;
  	    	send.msg.z3d = 0.f;//orgin.z; //3d position
  	    	pub_commond.publish(send);
  	    }
  	    else{
  	    	ROS_INFO("Arm: error code = %d  || status = %d", msg->ret, status);
  	    }
  	    break;
  	  case 6: //PR
  	    if(msg->ret == 0){
          system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/sns.wav 6");
          ROS_INFO("demo over...");
        }
        else{
          ROS_INFO("Arm: error code = %d || status = %d", msg->ret, status);
        }
  	    break;
  	  default:
  	    ROS_INFO("not exist status @ArmCallback");
  	    break;
  	}
  }
  else{
  	ROS_INFO("wrong topic id @ArmCallback");
  }
}

// voice callback
void obj_TOP::VoiceCallback(const TOP::commond::ConstPtr& msg){
	//create the msg for publish
	TOP::commond send;
  //check topic id
  if(msg->topic_id == topic_id){
  	switch (status){
  		case 0: //初期
        ROS_INFO("invalid status = %d @Voice", status);
  	    break;
  	  case 1: //待機
  	    if(msg->ret == 0){
  	    	status = 2; //trans to 探索
  	    	object = 0; //set object as person
  	    	//set commond info
  	    	send.node = 1; //commond for yolo
  	    	topic_id++;
  	    	send.topic_id = topic_id;
  	    	send.msg.id = object;
          system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/people.wav 6");
  	    	pub_commond.publish(send);
  	    }
  	    else{
  	    	ROS_INFO("Voice: error code = %d  || status = %d", msg->ret, status);
  	    }
  	    break;
  	  case 2: //探索
  	  case 3: //移動
  	  case 4: //微調整
  	  case 5: //把持
  	    ROS_INFO("invalid status = %d @Voice", status);
  	    break;
  	  case 6: //PR
        if(msg->ret == 0){
          status = 6;
          send.node = 3; //arm
          topic_id++;
          send.topic_id = topic_id;
          send.msg.id = 1; //release
          pub_commond.publish(send);
        }
        else{
          ROS_INFO("Voice: error code = %d || status = %d", msg->ret, status);
        }
  	    break;
  	  default:
  	    ROS_INFO("not exist status @VoiceCallback");
  	    break;
  	}
  }
  else{
  	ROS_INFO("wrong topic id @VoiceCallback");
  }
}

void obj_TOP::SendFirstMsg(){
  TOP::commond send;
  send.node = 1; //commond for yolo
  send.msg.id = object;
  send.topic_id = topic_id;
  //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/search.wav 3");
  pub_commond.publish(send);
  ROS_INFO("send done %d", topic_id);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "TOP");
	ros::NodeHandle nh("~");
	obj_TOP obj_TOP;
  ROS_INFO("runing...");
  ros::Rate loop_rate(0.5);
  loop_rate.sleep();
  ROS_INFO("press g then press enter");
  getchar();
  ROS_INFO("OK");
  //system("python /home/ubuntu/catkin_ws/video/voice_play.py /home/ubuntu/catkin_ws/video/start.wav 3");
  obj_TOP.SendFirstMsg();  
	ros::spin();
	return 0;
}