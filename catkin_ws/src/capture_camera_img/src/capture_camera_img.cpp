/*
#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <strings.h>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2//core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
*/

#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <strings.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2//core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tbb/tbb.h>

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int16.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


int darknet_cnt = 0;

void msgCallback_rgbimage(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    static int lastdarknetcnt = 0;
    
    std::cout<<"rgb_callback"<<std::endl;
    
    if (darknet_cnt!=lastdarknetcnt)
    {
        std::cout<<"image saved"<<std::endl;
        cv::Mat test = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::resize(test, test, cv::Size(), 0.1, 0.1);        
        cv::imwrite(patch::to_string(darknet_cnt)+".png", test);   
        lastdarknetcnt = darknet_cnt;
    }
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("error");
        exit(-1);
    }
    cv::Mat rgb_im = cv_ptr->image;
    //cv::imshow("RGB capture image", cv_ptr->image);
    //cv::waitKey(10);
}

void msgCallback_capture_cnt(const std_msgs::Int16::ConstPtr& msg)
{   
    std::cout<<"darknet_cnt_callback"<<std::endl;
    darknet_cnt = msg->data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "capture_camera_img");
  ros::NodeHandle n;

  ros::Subscriber sub_img = n.subscribe("/camera/rgb/image_rect_color", 1, msgCallback_rgbimage);
  ros::Subscriber sub_cnt = n.subscribe("/image_capture_cnt", 1, msgCallback_capture_cnt);
  
  ros::spin();

  return 0;
}



/*
void syncMsgsCB(const sensor_msgs::ImageConstPtr& rgb, const std_msgs::Int16::ConstPtr& cnt)
{ 
    //std::cout<<"cnt->data:"<<cnt->data<<std::endl;
    std::cout<<"test"<<std::endl;
   
}
    
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, std_msgs::Int16> SyncPolicy;
    
int main(int argc, char *argv[]){
    ros::init(argc, argv, "capture_camera_img");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_img_sub(nh,"/camera/rgb/image_rect_color", 1);
    message_filters::Subscriber<std_msgs::Int16> capture_cnt_sub(nh,"/image_capture_cnt", 1);
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_img_sub, capture_cnt_sub);
    sync.registerCallback(boost::bind(&syncMsgsCB, _1, _2));
    
    ros::spin();
    
    return 0;
}
*/