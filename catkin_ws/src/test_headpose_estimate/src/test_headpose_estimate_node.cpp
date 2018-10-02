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

#include <tbb/tbb.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "test_headpose_estimate/ActionUnit.h"
#include "test_headpose_estimate/Face.h"
#include "test_headpose_estimate/Faces.h"

#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>

#include <openface/LandmarkCoreIncludes.h>
#include <openface/Face_utils.h>
#include <openface/FaceAnalyser.h>
#include <openface/GazeEstimation.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define PI 3.14159265

#define FaceYawLimMin -110
#define FaceYawLimMax -70
#define FaceYawOffset 0

#define WIDTH   50
#define HEIGHT  75
#define X1 270
#define Y1 250
#define X2 370
#define Y2 400
#define PersonBoxScale 4

#define ResizeSize 0.5

#define AngleofView 58

template <typename T> std::string tostr(const T& t)
{
    std::ostringstream os; os<<t; return os.str();
}

int frame_num = 0;
int modify_yaw_cnt = 0;
int darknet_cnt = 0;
int depth_cnt = 0;
int face_cnt = 0;

std::ofstream headposedata("headposedata.csv", std::ios::trunc);
std::ofstream facefeatures("facefeatures.csv", std::ios::trunc);

int time_init;

class TestHeadposeEstimate
{
public:
    TestHeadposeEstimate(ros::NodeHandle nh);
    ~TestHeadposeEstimate();
    void msgCallback_FaceRecognition(const test_headpose_estimate::Faces::ConstPtr& msg );
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ModifyHeadOrientation();
    void PublishPersonMarker(double theta,double distance);

private:
    ros::Subscriber ros_object_sub;
    ros::Subscriber ros_face_sub;
    ros::Subscriber rgb_img_sub;
    ros::Publisher velocity_pub;
    ros::Publisher arrow_marker_pub;
    ros::Publisher person_marker_pub;

    ros::NodeHandle nh1;

    std::vector<std::string>classnames;
    std::vector<int>detectedtimes;
    std::vector<int>detectedtimesfull;

    std::vector<std::string>classname;
    std::vector<int>boxxmin;
    std::vector<int>boxymin;
    std::vector<int>boxxmax;
    std::vector<int>boxymax;

    std::vector<int>personbox; 
    Eigen::MatrixXd rotmat;
    Eigen::MatrixXd rotmat2;

    std::vector<double>headorientation;   
    std::vector<int>nose_tip;
    std::vector<int>chin;
    std::vector<int>left_eye;
    std::vector<int>right_eye;
    std::vector<int>left_mouth;
    std::vector<int>right_mouth;

    std::vector<int>nose_end_point2D_drawtmp;
    std::vector<int>nose_end_point2D_draw;
    std::vector<int>nose_end_point2D_draw2;
    std::vector<int>nose_end_point2D_draw3;
    std::vector<int>nose_end_point2D_draw4;
 
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;
    std::vector<double>lastrotation_value;   
    std::vector<double>lasttranslation_value;   
    
};

TestHeadposeEstimate::TestHeadposeEstimate(ros::NodeHandle nh):
nh1(nh)
{
        ros_face_sub = nh1.subscribe("faces",1, &TestHeadposeEstimate::msgCallback_FaceRecognition, this);
        rgb_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &TestHeadposeEstimate::rgbImageCallback, this);
      
        arrow_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_arrow_marker", 1);
        person_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_person_marker", 1);
        velocity_pub=nh1.advertise<geometry_msgs::Twist>("cmd_vel",1);
}

TestHeadposeEstimate::~TestHeadposeEstimate()
{
}

void TestHeadposeEstimate::msgCallback_FaceRecognition(const  test_headpose_estimate::Faces::ConstPtr& msg )
{
    std::vector<test_headpose_estimate::Face> detectedfaces = msg->faces;
    if(!detectedfaces.empty())
    {
        face_cnt += 1;
        std::cout<<"face_callback:"<<face_cnt<<std::endl;
    }
    
    int i = 0;
    for(std::vector<test_headpose_estimate::Face>::iterator itr = detectedfaces.begin(); itr != detectedfaces.end() ; ++itr)
    {
        double headroll,headpitch,headyaw;
        double headori[3] = {0.0,0.0,0.0};

        nose_tip.clear();
        chin.clear();
        left_eye.clear();
        right_eye.clear(); 
        left_mouth.clear();
        right_mouth.clear();

        headorientation.clear();
        
        static ros::Time firsttime = ros::Time::now();
        ros::Time nowtime = ros::Time::now();

        double firsttimesec,nowtimesec,currenttimesec;

        firsttimesec = firsttime.toSec();
        nowtimesec = nowtime.toSec();

        currenttimesec = nowtimesec-firsttimesec;
     
        //std::cout<<"header:"<<(*itr).header.stamp<<std::endl;
        
        tf::Quaternion quat((*itr).head_pose.orientation.x,(*itr).head_pose.orientation.y,(*itr).head_pose.orientation.z,(*itr).head_pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(headroll, headpitch, headyaw);

        //std::cout<<"head_pose:"<<(*itr).head_pose.position.x<<", "<<(*itr).head_pose.position.y<<", "<<(*itr).head_pose.position.z<<std::endl;
        std::cout<<"landmarks_2d.size:"<<(*itr).landmarks_2d.size()<<std::endl;
        if((*itr).landmarks_2d.size())
        {
            //std::cout<<"x:"<<(*itr).landmarks_2d[33].x<<", "<<"y:"<<(*itr).landmarks_2d[33].y<<", "<<"z:"<<(*itr).landmarks_2d[33].z<<std::endl; 
            std::vector<cv::Point2f> image_points;
            std::vector<cv::Point3f> model_points;
            
            vector<cv::Point3f> nose_end_point3D;
            vector<cv::Point2f> nose_end_point2D;

            //11/18
            cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 541.20870062659242, 0, 318.78756964392710, 0 ,  540.20435182225424, 236.43301053278904, 0, 0, 1);
            cv::Mat dist_coeffs = (cv::Mat_<double>(4,1) << 0.06569569924719, -0.25862424608946, 0.00010394071172, -0.00024019257963);

            rotation_vector  = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation in axis-angle form
            translation_vector = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::Mat rotation_matrix  = cv::Mat::zeros(3, 3, CV_64FC1); // Rotation in axis-angle form
            
            nose_tip.push_back(std::round((*itr).landmarks_2d[33].x));
            nose_tip.push_back(std::round((*itr).landmarks_2d[33].y));
            chin.push_back(std::round((*itr).landmarks_2d[8].x));
            chin.push_back(std::round((*itr).landmarks_2d[8].y));
            left_eye.push_back(std::round((*itr).landmarks_2d[36].x));
            left_eye.push_back(std::round((*itr).landmarks_2d[36].y));
            right_eye.push_back(std::round((*itr).landmarks_2d[45].x));
            right_eye.push_back(std::round((*itr).landmarks_2d[45].y));
            left_mouth.push_back(std::round((*itr).landmarks_2d[48].x));
            left_mouth.push_back(std::round((*itr).landmarks_2d[48].y));
            right_mouth.push_back(std::round((*itr).landmarks_2d[54].x));
            right_mouth.push_back(std::round((*itr).landmarks_2d[54].y));

            image_points.push_back( cv::Point2d(nose_tip[0], nose_tip[1])) ; // Nose tip
            image_points.push_back( cv::Point2d(chin[0], chin[1])) ; // Chin
            image_points.push_back( cv::Point2d(left_eye[0], left_eye[1])) ; // Left eye left corner
            image_points.push_back( cv::Point2d(right_eye[0], right_eye[1])) ;// Right eye left corner
            image_points.push_back( cv::Point2d(left_mouth[0], left_mouth[1])) ;// Left Mouth corner
            image_points.push_back( cv::Point2d(right_mouth[0], right_mouth[1])) ; // Right Mouth corner

            float facemodelscale = 0.225;
            model_points.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));// Nose tip
            model_points.push_back(cv::Point3f(0.0f, -330.0*facemodelscale, -65.0*facemodelscale));// Chin
            model_points.push_back(cv::Point3f(-225.0*facemodelscale, 170.0*facemodelscale, -135.0*facemodelscale));// Left eye left corner
            model_points.push_back(cv::Point3f(225.0*facemodelscale, 170.0*facemodelscale, -135.0*facemodelscale));// Right eye right corner
            model_points.push_back(cv::Point3f(-150.0*facemodelscale, -150.0*facemodelscale, -125.0*facemodelscale));// Left Mouth corner
            model_points.push_back(cv::Point3f(150.0*facemodelscale, -150.0*facemodelscale, -125.0*facemodelscale));// Right mouth corner
           
            
            // Solve for pose
            cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector,CV_ITERATIVE);
            std::cout << "rotation_vector: " << rotation_vector.at<double>(0,0)<< " " <<rotation_vector.at<double>(0,1)<< " "<<rotation_vector.at<double>(0,2)<<std::endl;
            
            cv::Mat rotation_vector2  = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation in axis-angle form
            cv::Mat translation_vector2 = cv::Mat::zeros(3, 1, CV_64FC1);
            rotation_vector2 = rotation_vector;
            translation_vector2 = translation_vector;
            
            cv::Rodrigues(rotation_vector2, rotation_matrix); 
            /*
            std::cout << "Translation Vector2_before: " << translation_vector2.at<double>(0,0)<< " " <<translation_vector2.at<double>(0,1)<< " "<<translation_vector2.at<double>(0,2)<<std::endl;
            std::cout << "camera_matrix_before " << std::endl << camera_matrix << std::endl; 
            std::cout << "dist_coeffs_before" << std::endl << dist_coeffs << std::endl; 
            */
            
            cv::Mat rotMatrixX,rotMatrixY,rotMatrixZ;
            cv::Vec3d eulerAngles;
            double* _r = rotation_matrix.ptr<double>();
            double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                                                        _r[3],_r[4],_r[5],0,
                                                        _r[6],_r[7],_r[8],0};
            cv::decomposeProjectionMatrix( cv::Mat(3,4,CV_64FC1,projMatrix),
            camera_matrix,
            rotation_matrix,
            translation_vector2,
            rotMatrixX,
            rotMatrixY,
            rotMatrixZ,
            eulerAngles);
            headorientation.push_back(eulerAngles[2]);//roll
            headorientation.push_back(eulerAngles[0]);//pitch
            headorientation.push_back(eulerAngles[1]);//yaw
                                                                                                    
            cout << "headorientation:" << headorientation[0] <<", "<< headorientation[1] << ", "<< headorientation[2] << endl;

            camera_matrix = (cv::Mat_<double>(3,3) << 541.20870062659242, 0, 318.78756964392710, 0 ,  540.20435182225424, 236.43301053278904, 0, 0, 1);
            dist_coeffs = (cv::Mat_<double>(4,1) << 0.06569569924719, -0.25862424608946, 0.00010394071172, -0.00024019257963);
            
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point2D_drawtmp.clear();
            
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,250.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].y));
        
            std::cout << "rotation_vector: " << rotation_vector.at<double>(0,0)<< " " <<rotation_vector.at<double>(0,1)<< " "<<rotation_vector.at<double>(0,2)<<std::endl;
            std::cout << "nose_end_point2D_drawtmp:"<<nose_end_point2D_drawtmp[0] <<", "<<nose_end_point2D_drawtmp[1]<< std::endl;

            TestHeadposeEstimate::ModifyHeadOrientation();
            
            nose_end_point2D_drawtmp.clear();
            nose_end_point2D_draw.clear();
            nose_end_point2D_draw2.clear();
            nose_end_point2D_draw3.clear();
            
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,250.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].y));
        
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,500.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].y));
 
            nose_end_point2D.clear();
            nose_end_point3D.clear();        
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,750.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].y));
                        
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3f(0,0,1000.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_draw3.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw3.push_back(std::round(nose_end_point2D[0].y));
            
            std::cout << "nose_end_point2D_drawtmp:"<<nose_end_point2D_drawtmp[0] <<", "<<nose_end_point2D_drawtmp[1]<< std::endl;
            std::cout << "nose_end_point2D_draw:"<<nose_end_point2D_draw[0] <<", "<<nose_end_point2D_draw[1]<< std::endl;
            std::cout << "nose_end_point2D_draw2:"<<nose_end_point2D_draw2[0] <<", "<<nose_end_point2D_draw2[1]<< std::endl;
            std::cout << "nose_end_point2D_draw3:"<<nose_end_point2D_draw3[0] <<", "<<nose_end_point2D_draw3[1]<< std::endl;
            
            std::cout<<""<<std::endl;
        }
       }    
}


void TestHeadposeEstimate::ModifyHeadOrientation()
{
    static int init = 0;

    static double lastheadori[3];
    static double lastnose_end_point2D_drawtmptheta;
    double nose_end_point2D_drawtmptheta;
    
    if(!init)
    {
        std::cout<<"init"<<std::endl;    
        lastheadori[2] =  headorientation[2];
        for(int i=0;i<3;i++)
        {
            lastrotation_value.push_back(rotation_vector.at<double>(0,i));
            lasttranslation_value.push_back(translation_vector.at<double>(0,i));
        }
        init = 1;
        lastnose_end_point2D_drawtmptheta = atan2(nose_end_point2D_drawtmp.at(0)-nose_tip[0],nose_end_point2D_drawtmp.at(1)-nose_tip[1])* 180 / PI;
    }

    nose_end_point2D_drawtmptheta = atan2(nose_end_point2D_drawtmp.at(0)-nose_tip[0],nose_end_point2D_drawtmp.at(1)-nose_tip[1])* 180 / PI;
    
    std::cout<<nose_end_point2D_drawtmptheta<<","<<lastnose_end_point2D_drawtmptheta<<","<<lastnose_end_point2D_drawtmptheta-nose_end_point2D_drawtmptheta<<std::endl;
    
    /*
    if(abs(lastnose_end_point2D_drawtmptheta-nose_end_point2D_drawtmptheta)>75)
    {
        modify_yaw_cnt += 1;
        std::cout<<"modify angle"<<std::endl;
        //std::cout<<"modify_yaw_cnt:"<<modify_yaw_cnt<<std::endl;
        for(int i=0;i<3;i++)
        {
            headorientation[i] = lastheadori[i];
            rotation_vector.at<double>(0,i) = lastrotation_value.at(i);
            translation_vector.at<double>(0,i) = lasttranslation_value.at(i);
        }
    }
    */
    //else if(!(((nose_end_point2D_drawtmp[0]>0)&&(nose_end_point2D_drawtmp[0]<640))&&((nose_end_point2D_drawtmp[1]>0)&&(nose_end_point2D_drawtmp[1]<480))))
    if(!(((nose_end_point2D_drawtmp[0]>0)&&(nose_end_point2D_drawtmp[0]<640))&&((nose_end_point2D_drawtmp[1]>0)&&(nose_end_point2D_drawtmp[1]<480))))
    {
        std::cout<<"modify drawpoint"<<std::endl;
        for(int i=0;i<3;i++)
        {
            headorientation[i] = lastheadori[i];
            rotation_vector.at<double>(0,i) = lastrotation_value.at(i);
            translation_vector.at<double>(0,i) = lasttranslation_value.at(i);
        }
    }
    else
    {
        lastrotation_value.clear();
        lasttranslation_value.clear();
        std::cout<<"save last data"<<std::endl;
        for(int i=0;i<3;i++)
        {   
            lastheadori[i] = headorientation[i];    
            lastrotation_value.push_back(rotation_vector.at<double>(0,i));
            lasttranslation_value.push_back(translation_vector.at<double>(0,i));
        }
        lastnose_end_point2D_drawtmptheta = atan2(nose_end_point2D_drawtmp.at(0)-nose_tip[0],nose_end_point2D_drawtmp.at(1)-nose_tip[1])* 180 / PI;
    }
}

//RGB
void TestHeadposeEstimate::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    int i, j;
    int x1, x2, y1, y2;
    int width = WIDTH;
    int height = HEIGHT;
    cv_bridge::CvImagePtr cv_ptr;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    cv::Mat &mat = cv_ptr->image;

    //cv::putText(cv_ptr->image, std::to_string(face_cnt), cv::Point(550,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA); 
    
    //std::cout<<"RGB image_captute"<<std::endl;

    if(!nose_tip.empty())
    {
        /*
        cv::circle(cv_ptr->image, cv::Point(nose_tip[0], nose_tip[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(chin[0], chin[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_eye[0], left_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_eye[0], right_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_mouth[0], left_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_mouth[0], right_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);
        */

        //cout << "nose_end_point2D_draw:"<<nose_end_point2D_draw[0] <<", "<<nose_end_point2D_draw[1]<< endl;
    
      //250
      cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_drawtmp[0], nose_end_point2D_drawtmp[1]), 5, cv::Scalar(0,0,255), 5);
      
      /*
      //500
      cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw[0], nose_end_point2D_draw[1]), 5, cv::Scalar(255, 255,0), 3);
      //750        
      cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw2[0], nose_end_point2D_draw2[1]), 5, cv::Scalar(0,255,255), 3);
      //1000
      cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw3[0], nose_end_point2D_draw3[1]), 5, cv::Scalar(255,255,255), 3);
      */
      /*
      if(((nose_end_point2D_draw3[0]>0)&&(nose_end_point2D_draw3[0]<640))&&((nose_end_point2D_draw3[1]>0)&&(nose_end_point2D_draw3[1]<480)))
      {
          cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw3[0],nose_end_point2D_draw3[1]), cv::Scalar(255,255,225), 2);
      }
      else if(((nose_end_point2D_draw2[0]>0)&&(nose_end_point2D_draw2[0]<640))&&((nose_end_point2D_draw2[1]>0)&&(nose_end_point2D_draw2[1]<480)))
      {    
          cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw2[0],nose_end_point2D_draw2[1]), cv::Scalar(0,255,225), 2);
      }
      else
      {
          cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw[0],nose_end_point2D_draw[1]), cv::Scalar(255,255,0), 2);
      }
      */

      cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_drawtmp[0],nose_end_point2D_drawtmp[1]), cv::Scalar(0,255,0), 2);
    }
    //std::cout<< " " <<std::endl;
    //cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), ResizeSize, ResizeSize); 
    cv::imshow("RGB image", cv_ptr->image);
    cvMoveWindow("RGB image",1000,350);
    cv::waitKey(10);
}

void TestHeadposeEstimate::PublishPersonMarker(double theta, double distance)
{
    visualization_msgs::Marker arrowmarker;
    
    arrowmarker.header.frame_id = "/base_link";
    arrowmarker.header.stamp = ros::Time::now();
    arrowmarker.ns = "basic_shapes";
    arrowmarker.type = visualization_msgs::Marker::ARROW;
    arrowmarker.action = visualization_msgs::Marker::ADD;
   
    arrowmarker.pose.orientation=tf::createQuaternionMsgFromYaw(theta/180.0*M_PI);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    arrowmarker.scale.x = 1.0;
    arrowmarker.scale.y = 0.1;
    arrowmarker.scale.z = 0.1;
    // Set the color -- be sure to set alpha to something non-zero!
    arrowmarker.color.r = 0.0f;
    arrowmarker.color.g = 1.0f;
    arrowmarker.color.b = 0.0f;
    arrowmarker.color.a = 1.0;

    arrowmarker.lifetime = ros::Duration();
    arrow_marker_pub.publish(arrowmarker);

    visualization_msgs::Marker personmarker;

    personmarker.header.stamp = ros::Time::now();
    personmarker.header.frame_id = "/base_link";
    personmarker.pose.position.x = distance * cos(theta/180.0*M_PI);
    personmarker.pose.position.y = distance * sin(theta/180.0*M_PI);
    personmarker.lifetime = ros::Duration(1);
    personmarker.scale.x = .4;
    personmarker.scale.y = .4;
    personmarker.scale.z = .4;
    personmarker.color.a = 0.5;
    personmarker.color.r = 0.5;
    personmarker.type = personmarker.SPHERE;
    person_marker_pub.publish(personmarker);
}

// 購読者ノードのメイン関数
int main(int argc, char **argv)
{
    // ノード名の初期化
    ros::init(argc, argv, "test_headpose_estimate");

    headposedata<<"time"<<","
    <<"head_pose_x"<<","
    <<"head_pose_y"<<","
    <<"head_pose_z"<<","
    <<"head_roll"<<","
    <<"head_pitch"<<","
    <<"head_yaw"<<","
    <<"quaternion_x"<<", "
    <<"quaternion_y"<<", "
    <<"quaternion_z"<<", "
    <<"quaternion_w"<<std::endl;

    facefeatures<<"time"<<","
    <<"x"<<","
    <<"y"<<","
    <<"z"<<std::endl;
  

    ros::NodeHandle nh;
    TestHeadposeEstimate pt(nh);
    
    ros::spin();
    return 0;
}
