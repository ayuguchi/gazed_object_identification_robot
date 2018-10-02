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
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14159265


template <typename T> std::string tostr(const T& t)
{
    std::ostringstream os; os<<t; return os.str();
}

int framenum = 0;

std::vector<std::string>classname;
std::vector<int>detectedtimes;
std::vector<int>detectedtimesfull;

class DarknetTest
{
public:
    DarknetTest(ros::NodeHandle nh);
    //DarknetTest();
    ~DarknetTest();
    void msgCallback_ObjectRecognition(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    ros::Subscriber ros_object_sub;
    ros::Subscriber sub_rgb;
    ros::Publisher velocity_pub;
    //ros::Publisher ros_text_pub;
    ros::NodeHandle nh1;
};

DarknetTest::DarknetTest(ros::NodeHandle nh):
nh1(nh)
//DarknetTest::DarknetTest()
{
        ros_object_sub = nh1.subscribe("/darknet_ros/bounding_boxes",1, &DarknetTest::msgCallback_ObjectRecognition, this);
        //sub_rgb = nh1.subscribe<sensor_msgs::Image>("/darknet_ros/detection_image", 1, &DarknetTest::rgbImageCallback, this);
        velocity_pub=nh1.advertise<geometry_msgs::Twist>("cmd_vel",1);
   
        //ros_text_pub = nh1.advertise<visualization_msgs::Marker>("target_text", 20);
}

DarknetTest::~DarknetTest()
{
}

void DarknetTest::msgCallback_ObjectRecognition(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg )
{
    std::cout<<"object callback"<<std::endl;
    
    std::vector<darknet_ros_msgs::BoundingBox> detectedobjects = msg->boundingBoxes;

    int cnt = 0;     
    for(std::vector<darknet_ros_msgs::BoundingBox>::iterator itr = detectedobjects.begin(); itr != detectedobjects.end() ; ++itr)
    {
        std::cout<<"framenum:"<<framenum<<",cnt:"<<cnt<<",class:"<<(*itr).Class<<std::endl;

        if(classname.empty())
        {
            std::cout<<"add first class:"<<(*itr).Class<<std::endl;
            classname.push_back((*itr).Class);
            detectedtimes.push_back(0);
            detectedtimesfull.push_back(0);
        }
        else
        {
            std::vector<std::string>::iterator cniter = std::find(classname.begin(),classname.end(),(*itr).Class);

            if(cniter == classname.end())
            {
                std::cout<<"add new class:"<<(*itr).Class<<std::endl;
                classname.push_back((*itr).Class);
                detectedtimes.push_back(0);
                detectedtimesfull.push_back(0);
            }                              
        }

        std::vector<std::string>::iterator citer2 = std::find(classname.begin(),classname.end(),(*itr).Class);
        int index = std::distance(classname.begin(), citer2);
        detectedtimes.at(index) += 1;
    
        for(int i=0;i<detectedtimes.size();i++) 
            std::cout << detectedtimes.at(i) << " ";
        std::cout << std::endl;

        cnt += 1;
    }

    for(int i=0;i<detectedtimesfull.size();i++)
        detectedtimesfull.at(i) += detectedtimes.at(i);

    for(int i=0;i<detectedtimes.size();i++)
        detectedtimes.at(i) = 0;

    std::cout<<"classnames"<<std::endl;
    for(int i=0;i<classname.size();i++)
        std::cout << classname.at(i) << " ";
    std::cout << std::endl;
    
    std::cout<<"detectedtimesfull"<<std::endl;
    for(int i=0;i<detectedtimesfull.size();i++)
        std::cout << detectedtimesfull.at(i) << " ";
    std::cout << std::endl;
    std::cout<<""<<std::endl;

    framenum += 1;
}

//RGB
void DarknetTest::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //std::cout<<"object image callback"<<std::endl;

    cv_bridge::CvImagePtr rgb_ptr;

    try
    {
        rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
        catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR("error");
        exit(-1);
    }
    cv::Mat rgb_im = rgb_ptr->image;
    cv::imshow("RGB darknet image", rgb_im);
    cv::waitKey(10);

}

// 購読者ノードのメイン関数
int main(int argc, char **argv)
{
    // ノード名の初期化
    ros::init(argc, argv, "test_darknet");
    //DarknetTest DarknetTest;
    
    ros::NodeHandle nh;
    DarknetTest pt(nh);
    ros::spin();
    return 0;
}
