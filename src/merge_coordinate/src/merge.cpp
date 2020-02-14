//c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//subscribe msg
 //realsen
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
 // darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <cv_bridge/cv_bridge.h>
#include <TOP/commond.h>

#define WIDTH   30
#define HEIGHT  30
#define FOCUS   619.5469360351562f
#define C_X     328.6131591796875f
#define C_Y     235.708740234375f
class obj_recognition{
public:
    obj_recognition();
    ~obj_recognition();
    void rgbImageCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void TOPCallback(const TOP::commond::ConstPtr& msg);
 
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb, sub_depth, sub_TOP;
    ros::Publisher pub_pos, pub_run;
    //Fenny test
    //image_transport::ImageTransport imageTransport_;
    //image_transport::Subscriber imageSubscriber_;
    float obj_pos_x;
    float obj_pos_y;
    float obj_pos_z;
    bool runing;
    bool detected;
    int topic_id;
    int object_id;
    int xmin,ymin,xmax,ymax;
    //int inter_flag=0;
};
 
obj_recognition::obj_recognition()
    //:imageTransport_(nh)
{
    sub_rgb = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 1, &obj_recognition::rgbImageCallback, this);
    sub_depth = nh.subscribe<sensor_msgs::Image>("camera/aligned_depth_to_color/image_raw", 1, &obj_recognition::depthImageCallback, this);
    //Fenny test
    //imageSubscriber_ = imageTransport_.subscribe("camera/aligned_depth_to_color/image_raw", 1, &obj_recognition::depthImageCallback, this, image_transport::TransportHints("compressed"));
    sub_TOP = nh.subscribe<TOP::commond>("Commond_TOP", 1, &obj_recognition::TOPCallback, this);
    pub_pos = nh.advertise<TOP::commond>("results_YOLO", 1);
    pub_run = nh.advertise<std_msgs::Int32>("YOLO_GO", 1);
    obj_pos_x = 0.f;
    obj_pos_y = 0.f;
    obj_pos_z = 0.f;
    detected = false;
    runing = false;
    topic_id=-1;
    object_id=-1;
}
 
obj_recognition::~obj_recognition(){

}
void obj_recognition::TOPCallback(const TOP::commond::ConstPtr& msg){
    if(msg->node == 1 || msg->node == 0){
        topic_id = msg->topic_id;
        object_id = msg->msg.id;
        std_msgs::Int32 send;
        send.data = object_id;
        pub_run.publish(send);
        runing = true;
        ROS_INFO("%d", msg->topic_id);
    }
    else{
        ROS_INFO("this tipic is not for yolo");
    }
}
void obj_recognition::rgbImageCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
   float maxf=0.f;
   int size;
   int index;
   size = msg->bounding_boxes.size();
   if(runing == true)
   	ROS_INFO("yolo results received...");
   if(size){
   	//ROS_INFO("deteted num %d", size);
    for(int i=0;i<size;++i)
    {
        if(msg->bounding_boxes[i].probability>maxf)
        {
            maxf=msg->bounding_boxes[i].probability;
            index=i;
        }
    }

    xmin = msg->bounding_boxes[index].xmin;
    ymin = msg->bounding_boxes[index].ymin;
    xmax = msg->bounding_boxes[index].xmax;
    ymax = msg->bounding_boxes[index].ymax;

    obj_pos_x = xmin + (xmax-xmin)/2;
    obj_pos_y = ymin + (ymax-ymin)/2;
    //ROS_INFO("xmin: %d , xmax: %d, ymin: %d, ymax: %d, x=%d, y=%d",xmin,xmax,ymin,ymax, obj_pos_x, obj_pos_y);
    detected=true;
   }
   else
   {
   	//ROS_INFO("detected num 0");
    obj_pos_x=-1;
    obj_pos_y=-1;
    detected=false;
   }
   //runing=true;
}
 
void obj_recognition::depthImageCallback(const sensor_msgs::ImageConstPtr& msg){
 
    if(runing == false){
        //ROS_INFO("not runing yet...");
        return;
    }
    //ROS_INFO("start...");
    int x1, x2, y1, y2;
    int i, j, k;
    int width = WIDTH;
    int height = HEIGHT;
    float sum = 0.0;
    float ave;
    cv_bridge::CvImagePtr cv_ptr;
    TOP::commond send;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//TYPE_32FC1
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
 
    cv::Mat depth(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

    cv::Mat &mat = cv_ptr->image;
    ROS_INFO("depth image received");
#if 1  //debug
    send.topic_id = topic_id;
    if(detected){
        send.ret = 0;
        x1 = obj_pos_x - width;
        if(x1<0)
            x1=0;
        if(x1<xmin)
        	x1=xmin;
        x2 = obj_pos_x + width;
        if(x2>=639)
            x2=639;
        if(x2>=xmax)
        	x2=xmax;
        y1 = obj_pos_y - height;
        if(y1<0)
            y1=0;
        if(y1<ymin)
        	y1=ymin;
        y2 = obj_pos_y + height;
        if(y2>=479)
            y2=479;
        if(y2>=ymax)
        	y2=ymax;
    
    
        int counter=0;
        for(i = 0; i < cv_ptr->image.rows;i++){
            float* Dimage = cv_ptr->image.ptr<float>(i);
            float* Iimage = depth.ptr<float>(i);
            char* Ivimage = img.ptr<char>(i);
            for(j = 0 ; j < cv_ptr->image.cols; j++){
                if(Dimage[j] > 0.0){
                    Iimage[j] = Dimage[j];
                    Ivimage[j] = (char)(255*(Dimage[j]/5.5));
                }else{
                }
 
                if(i > y1 && i < y2){
                    if(j > x1 && j < x2){
                        if(Dimage[j] > 0.0){
                            sum += Dimage[j];
                            counter++;
                        }
                    }
                }
            }
        }
        if(counter==0)
        {
            send.ret = 1;
            ROS_INFO("no depth info");
        }
        else
        {
            ave = sum / (float)counter;
        
            float xx,yy;
            xx = (float)obj_pos_x - C_X;
            yy = (float)obj_pos_y - C_Y;

            //2d position
            send.msg.x2d = (int)obj_pos_x;
            send.msg.y2d = (int)obj_pos_y;
            //3d position
            send.msg.x3d = (xx*ave)/FOCUS/1000.0;
            send.msg.y3d = (yy*ave)/FOCUS/1000.0;
            send.msg.z3d = ave/1000.0;
            ROS_INFO("results %f %f %f", send.msg.x3d, send.msg.y3d, send.msg.z3d);
        }        
    }
    else
    {
        send.ret = 1;
        ROS_INFO("Nothing");
    }
    
    
    //image.width=obj_width;
    //image.height=obj_height;
    //ROS_INFO("(%d, %d, %d, %d, %d)", image.x, image.y, image.distance, image.width, image.height);
    pub_pos.publish(send);
    ROS_INFO("publish done");
    runing = false;
#endif
}

int main(int argc, char **argv){
    ros::init(argc, argv, "merge_coordinate");
    ros::NodeHandle nh("~");
    obj_recognition object_recognition;
    ROS_INFO("runing...");
    ros::spin();
    return 0;
}