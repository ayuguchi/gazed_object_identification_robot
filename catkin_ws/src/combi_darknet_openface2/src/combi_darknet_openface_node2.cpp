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

#include "combi_darknet_openface2/ActionUnit.h"
#include "combi_darknet_openface2/Face.h"
#include "combi_darknet_openface2/Faces.h"

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

#define AngleofView 58


#define ResizeSize 0.75

template <typename T> std::string tostr(const T& t)
{
    std::ostringstream os; os<<t; return os.str();
}

int frame_num = 0;

int modify_yaw_cnt = 0;
int modify_distance_cnt = 0;

int kf_failed_cnt = 0;
int face_cnt = 0;
int darknet_cnt = 0;
int rgb_cnt = 0;
int depth_cnt = 0;
int kf_cnt = 0;
int area_error = 0;

int robotstop = 0;
int personstop = 0;
int personstop_cnt = 0;

double lastcurrenttimevelocity  = 0;

std::ofstream headposedata("headposedata.csv", std::ios::trunc);
std::ofstream facefeatures("facefeatures.csv", std::ios::trunc);
std::ofstream timeusedata("timeusedata.csv", std::ios::trunc);
std::ofstream activityscorefacedata("activityscorefacedata.csv", std::ios::trunc);
std::ofstream activityscoreobjectdata("activityscoreobjectdata.csv", std::ios::trunc);
std::ofstream scorefacedata("scorefacedata.csv", std::ios::trunc);
std::ofstream scoreobjectdata("scoreobjectdata.csv", std::ios::trunc);
std::ofstream scorefacelabel("scorefacelabel.csv", std::ios::trunc);
std::ofstream scoreobjectlabel("scoreobjectlabel.csv", std::ios::trunc);
std::ofstream robotcmdveldata("robotcmdveldata.csv", std::ios::trunc);
std::ofstream personvelocitydata("personvelocitydata.csv", std::ios::trunc);
std::ofstream depthdata("depthdata.csv", std::ios::trunc);
std::ofstream alltimerecord("alltimerecord.csv", std::ios::trunc);


int time_init;

class CombiDarknetOpenface
{
public:
    CombiDarknetOpenface(ros::NodeHandle nh);
    ~CombiDarknetOpenface();
    void msgCallback_cmdvel(const geometry_msgs::Twist& msg );//cmd_vel
    void msgCallback_FaceRecognition(const combi_darknet_openface2::Faces::ConstPtr& msg );
    void ModifyHeadOrientation();
    void ModifyPersonDistance(double* distance);
    void Publish_Velocity();
    void Calculate_TimeUse(double currenttimesec);
    void msgCallback_ObjectRecognition(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void rgbObjectImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);//face_feature
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void msgCallback_FilterMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void PublishPersonMeasurement(double measurementx, double measurementy);//input for KF
    void PublishPersonMarker(double theta, double measurementx, double measurementy);//input for KF
    void PublishHeadposeArrow();//input for KF
        

private:
    ros::Subscriber ros_object_sub;
    ros::Subscriber rgb_object_sub;
    ros::Subscriber cmd_vel_sub;
    
    ros::Subscriber ros_face_sub;
    ros::Subscriber rgb_img_sub;
    ros::Subscriber depth_img_sub;

    ros::Subscriber ros_filtered_sub;
    ros::Publisher measurement_pub;
    
    ros::Publisher headpose_arrow_pub;

    ros::Publisher origin_marker_pub;
    ros::Publisher person_arrow_pub;
    ros::Publisher person_marker_pub;

    ros::Publisher velocity_pub;
    

    ros::NodeHandle nh1;

    std::vector<std::string>classnames;
    std::vector<std::string>classname;
    std::vector<int>boxxmin;
    std::vector<int>boxymin;
    std::vector<int>boxxmax;
    std::vector<int>boxymax;
    std::vector<int>boxxmin2;
    std::vector<int>boxymin2;
    std::vector<int>boxxmax2;
    std::vector<int>boxymax2;
    std::vector<int>boxcenterx;
    std::vector<int>boxcentery;
    std::vector<int>lastboxcenterx;
    std::vector<int>lastboxcentery;
    
    std::vector<int>activityscoreface;
    std::vector<int>activityscoreobject;


    std::vector<double>estimateposition;
    std::vector<double>lastestimateposition;
    std::vector<double>personvelocity;
    
    std::vector<int>personbox; 
    
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
            
    /*
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;

    cv::Mat rotation_vectortmp; // Rotation in axis-angle form
    cv::Mat translation_vectortmp;
    */
};

CombiDarknetOpenface::CombiDarknetOpenface(ros::NodeHandle nh):
nh1(nh)
{
        ros_object_sub = nh1.subscribe("/darknet_ros/bounding_boxes",1, &CombiDarknetOpenface::msgCallback_ObjectRecognition, this);
        rgb_object_sub = nh1.subscribe<sensor_msgs::Image>("/darknet_ros/detection_image", 1, &CombiDarknetOpenface::rgbObjectImageCallback, this);
        
        
        ros_face_sub = nh1.subscribe("faces",1, &CombiDarknetOpenface::msgCallback_FaceRecognition, this);
        /*
        rgb_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &CombiDarknetOpenface::rgbImageCallback, this);
        depth_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &CombiDarknetOpenface::depthImageCallback, this);

        ros_filtered_sub= nh1.subscribe("estimate_pos",1, &CombiDarknetOpenface::msgCallback_FilterMsg, this);
        measurement_pub = nh1.advertise<geometry_msgs::PoseStamped>("filter_measurement", 1);
        
        cmd_vel_sub= nh1.subscribe("cmd_vel",1, &CombiDarknetOpenface::msgCallback_cmdvel, this);
        
        headpose_arrow_pub = nh.advertise<visualization_msgs::Marker>("/visualization_headpose_arrow", 1);

        origin_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_origin_marker", 1);
        person_arrow_pub = nh.advertise<visualization_msgs::Marker>("/visualization_person_arrow", 1);
        person_marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_person_marker", 1);
        
        velocity_pub=nh1.advertise<geometry_msgs::Twist>("cmd_vel",1);
        */
}

CombiDarknetOpenface::~CombiDarknetOpenface()
{
}

void CombiDarknetOpenface::msgCallback_cmdvel(const geometry_msgs::Twist& msg)
{
    static ros::Time firsttime = ros::Time::now();
    ros::Time nowtime = ros::Time::now();

    double firsttimesec,nowtimesec,currenttimesec;
    firsttimesec = firsttime.toSec();
    nowtimesec = nowtime.toSec();
    currenttimesec = nowtimesec-firsttimesec;

    std::cout<<"cmd_vel_callback"<<std::endl;

    if(!(msg.linear.x==0)&&(msg.linear.y)&&(msg.angular.z==0))
    {
        robotstop = 1;
    }
    else
    {
        robotstop = 0;
    }

    std::cout<<"personstop:"<<personstop<<std::endl;
        

    robotcmdveldata<<darknet_cnt<<","
    <<currenttimesec<<","
    <<msg.linear.x<<","
    <<msg.linear.y<<","
    <<msg.angular.z<<","
    <<robotstop<<std::endl;
    
}

void CombiDarknetOpenface::msgCallback_FaceRecognition(const  combi_darknet_openface2::Faces::ConstPtr& msg )
{
    std::vector<combi_darknet_openface2::Face> detectedfaces = msg->faces;
    
    if(!detectedfaces.empty())
    {
        face_cnt += 1;
        std::cout<<"face_callback:"<<face_cnt<<std::endl;
    }

    nose_tip.clear();
    chin.clear();
    left_eye.clear();
    right_eye.clear(); 
    left_mouth.clear();
    right_mouth.clear();
    nose_end_point2D_drawtmp.clear();
    headorientation.clear(); 

    /*
    nose_end_point2D_draw.clear();
    nose_end_point2D_draw2.clear();
    nose_end_point2D_draw3.clear();
    nose_end_point2D_draw4.clear();
    */

    for(std::vector<combi_darknet_openface2::Face>::iterator itr = detectedfaces.begin(); itr != detectedfaces.end() ; ++itr)
    {   
        static ros::Time firsttime = ros::Time::now();
        ros::Time nowtime = ros::Time::now();

        double firsttimesec,nowtimesec,currenttimesec;

        firsttimesec = firsttime.toSec();
        nowtimesec = nowtime.toSec();

        currenttimesec = nowtimesec-firsttimesec;

        /*
        std::cout<<"head_orientation:"<<(*itr).head_pose.orientation.x<<", "<<(*itr).head_pose.orientation.y<<", "<<(*itr).head_pose.orientation.z<<", "<<(*itr).head_pose.orientation.w<<std::endl;
        //std::cout<<"head_orientation:"<<headroll*180/PI<<", "<<headpitch*180/PI<<", "<<headyaw*180/PI<<std::endl;
        std::cout<<"left_gaze:"<<(*itr).left_gaze.x<<", "<<(*itr).left_gaze.y<<", "<<(*itr).left_gaze.y<<std::endl;
        std::cout<<"right_gaze:"<<(*itr).right_gaze.x<<", "<<(*itr).right_gaze.y<<", "<<(*itr).right_gaze.y<<std::endl;
        //std::cout<<"header:"<<(*itr).header.stamp<<std::endl;
        */

        int i = 0;
        //std::cout<<"landmarks_2d.size:"<<(*itr).landmarks_2d.size()<<std::endl;
        if((*itr).landmarks_2d.size())
        {
            std::cout<<"x:"<<(*itr).landmarks_2d[33].x<<", "<<"y:"<<(*itr).landmarks_2d[33].y<<", "<<"z:"<<(*itr).landmarks_2d[33].z<<std::endl; 
            /*
            std::vector<cv::Point2f> image_points;
            std::vector<cv::Point3f> model_points;
            
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

            model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Nose tip
            model_points.push_back(cv::Point3d(0.0f, -330.0f, -65.0f));          // Chin
            model_points.push_back(cv::Point3d(-225.0f, 170.0f, -135.0f));       // Left eye left corner
            model_points.push_back(cv::Point3d(225.0f, 170.0f, -135.0f));        // Right eye right corner
            model_points.push_back(cv::Point3d(-150.0f, -150.0f, -125.0f));      // Left Mouth corner
            model_points.push_back(cv::Point3d(150.0f, -150.0f, -125.0f));       // Right mouth corner
            
            cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 570.3422241210938, 0, 319.5, 0 ,  570.3422241210938, 239.5, 0, 0, 1);
            cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion

            rotation_vector  = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation in axis-angle form
            translation_vector = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::Mat rotation_matrix  = cv::Mat::zeros(3, 3, CV_64FC1); // Rotation in axis-angle form
            
            cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector,CV_ITERATIVE);
            
            vector<cv::Point3d> nose_end_point3D;
            vector<cv::Point2d> nose_end_point2D;
            nose_end_point3D.push_back(cv::Point3d(0,0,1000.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].y));
            */

            /*
            //std::cout << "Rotation Vector: " << rotation_vector.at<double>(0,0)<< " " <<rotation_vector.at<double>(0,1)<< " "<<rotation_vector.at<double>(0,2)<<std::endl;
            //std::cout << "Translation Vector: " << translation_vector.at<double>(0,0)<< " " <<translation_vector.at<double>(0,1)<< " "<<translation_vector.at<double>(0,2)<<std::endl;
            //cout << "nose_tip:"<<nose_tip[0] << ", "<< nose_tip[1] << endl;
            //cout << "nose_end_point2D:"<<nose_end_point2D[0] << endl;
            */
            
            /*
            std::cout << "Rotation Vector: " << rotation_vector.at<double>(0,0)<< " " <<rotation_vector.at<double>(0,1)<< " "<<rotation_vector.at<double>(0,2)<<std::endl;
            std::cout << "Translation Vector: " << translation_vector.at<double>(0,0)<< " " <<translation_vector.at<double>(0,1)<< " "<<translation_vector.at<double>(0,2)<<std::endl;
            cout << "nose_end_point2D_drawtmp  :"<<nose_end_point2D_drawtmp[0]<<","<<nose_end_point2D_drawtmp[1]<< endl;
            
            cv::Mat translation_vectortmp = translation_vector;
            cv::Rodrigues(rotation_vector, rotation_matrix); 
            //cout << "Rotation Matrix " << endl << rotation_matrix << endl; 

            cv::Mat rotMatrixX,rotMatrixY,rotMatrixZ;
            cv::Vec3d eulerAngles;
            double* _r = rotation_matrix.ptr<double>();
            double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                                                    _r[3],_r[4],_r[5],0,
                                                    _r[6],_r[7],_r[8],0};

            cv::decomposeProjectionMatrix( cv::Mat(3,4,CV_64FC1,projMatrix),
            camera_matrix,
            rotation_matrix,
            translation_vectortmp,
            rotMatrixX,
            rotMatrixY,
            rotMatrixZ,
            eulerAngles);

            headorientation.push_back(eulerAngles[2]);//roll
            headorientation.push_back(eulerAngles[0]);//pitch
            headorientation.push_back(eulerAngles[1]);//yaw
            cout << "headorientation:" << headorientation[0] <<", "<< headorientation[1] << ", "<< headorientation[2] << endl;     
            CombiDarknetOpenface::ModifyHeadOrientation();
            CombiDarknetOpenface::PublishHeadposeArrow();
            */
            
        }
         
        /*
        if((*itr).landmarks_2d.size())
        {
            for(i=0;i<(*itr).landmarks_2d.size();i++)
            {
                facefeatures<<currenttimesec<<", "
                <<(*itr).landmarks_2d[i].x<<", "
                <<(*itr).landmarks_2d[i].y<<", "
                <<(*itr).landmarks_2d[i].z<<std::endl;
            }
        }
        else
        {
            std::cout<<"failed face detect"<<std::endl;
        }

        headposedata<<currenttimesec<<", "
        <<(*itr).head_pose.position.x<<", "
        <<(*itr).head_pose.position.y<<", "
        <<(*itr).head_pose.position.z<<","
        <<headorientation[0]<<", "
        <<headorientation[1]<<", "
        <<headorientation[2]<<std::endl;
        */
    } 

    std::cout<<"face detect finish"<<std::endl;
    std::cout<< " " <<std::endl;
}

void CombiDarknetOpenface::ModifyHeadOrientation()
{
     static double lastheadori[3];
     static int init = 0;

     //std::cout<<"lastheadori[2]_before:"<<lastheadori[2]<<","<<"headori[2]:"<<headori[2]<<","<<"diff:"<<lastheadori[2]-headori[2]<<std::endl;
 
     if(!init)
     {
        lastheadori[2] =  headorientation[2];
        init = 1;
     }

     if(abs(lastheadori[2]-headorientation[2])>50)
     {
        //std::cout<<"modify"<<std::endl;
        modify_yaw_cnt += 1;

        //std::cout<<"modify_yaw:"<<lastheadori[2]-headorientation[2]<<std::endl;
        //std::cout<<"modify_yaw_cnt:"<<modify_yaw_cnt<<std::endl;
        headorientation[2] = lastheadori[2];
     }
     else
     {
        lastheadori[2] = headorientation[2];   
     }
}


void CombiDarknetOpenface::Publish_Velocity()
{
        geometry_msgs::Twist twist;
        double sigyaw = 0.0;

        int faceyawlimmin = FaceYawLimMin + FaceYawOffset;//-90:neutra
        int faceyawlimmax = FaceYawLimMax + FaceYawOffset;

        if((faceyawlimmin<=headorientation[2])&&(headorientation[2]<=faceyawlimmax))
        {
               sigyaw = 0;
               std::cout<<"front"<<std::endl;
        }
        else if(headorientation[2]<faceyawlimmin)
        {
               sigyaw = 0.3;
               std::cout<<"right"<<std::endl;
        }
        else if(faceyawlimmax<headorientation[2])
        {
               sigyaw = -0.3;
               std::cout<<"left"<<std::endl;
        }

        twist.angular.z = sigyaw;

        //std::cout<<"sigyaw:"<<sigyaw<<std::endl;
        std::cout<<""<<std::endl;    

        //velocity_pub.publish(twist); 
}

void CombiDarknetOpenface::msgCallback_ObjectRecognition(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg )
{
    
    static ros::Time firsttime = ros::Time::now();
    ros::Time nowtime = ros::Time::now();

    double firsttimesec,nowtimesec,currenttimesec;
    firsttimesec = firsttime.toSec();
    nowtimesec = nowtime.toSec();
    currenttimesec = nowtimesec-firsttimesec;
    std::vector<darknet_ros_msgs::BoundingBox> detectedobjects = msg->boundingBoxes;

    classname.clear();
    boxxmin.clear();
    boxymin.clear();
    boxxmax.clear();
    boxymax.clear();
    boxxmin2.clear();
    boxymin2.clear();
    boxxmax2.clear();
    boxymax2.clear();
    boxcenterx.clear();
    boxcentery.clear();
    
    activityscoreface.clear();
    activityscoreobject.clear();

    personbox.clear();
        
    int objectcnt = 0;
    int personindex = 0; 

    darknet_cnt += 1;
    std::cout<<"darknet_callback:"<<darknet_cnt<<std::endl;
    
    if(classnames.empty())
    {   
        classnames.push_back("none");
    }

    boxxmin.push_back(0);
    boxymin.push_back(0);
    boxxmax.push_back(0);
    boxymax.push_back(0);
    classname.push_back("none");
 
    for(std::vector<darknet_ros_msgs::BoundingBox>::iterator itr = detectedobjects.begin(); itr != detectedobjects.end() ; ++itr)
    {
        classname.push_back((*itr).Class);
        if((*itr).Class=="person")
        {
           // personindex = objectcnt+1;
             personindex = objectcnt+1;
             std::cout<<"personindex:"<<personindex<<std::endl;
        }

        boxxmin.push_back((*itr).xmin);
        boxymin.push_back((*itr).ymin);
        boxxmax.push_back((*itr).xmax);
        boxymax.push_back((*itr).ymax);

        //std::cout<<"class:"<<(*itr).Class<<std::endl;
        if(classnames.size()==1)
        {
            std::cout<<"add first class:"<<(*itr).Class<<std::endl;
            classnames.push_back((*itr).Class);
        }
        else
        {
            std::vector<std::string>::iterator cniter = std::find(classnames.begin(),classnames.end(),(*itr).Class);
            if(cniter == classnames.end())
            {
                //std::cout<<"add new class:"<<(*itr).Class<<std::endl;
                classnames.push_back((*itr).Class);
            }                              
        }

        objectcnt += 1;
    }
    std::cout<<"objectcnt:"<<objectcnt<<std::endl;
                
    /*
    if(personindex)
    {
        //std::cout<<"personindex:"<<personindex-1<<std::endl;
        personbox.push_back(boxxmin.at(personindex));
        personbox.push_back(boxymin.at(personindex));
        personbox.push_back(boxxmax.at(personindex));
        personbox.push_back(boxymax.at(personindex));

        //std::cout<<"box_darknet:"<<personbox.at(0)<<","<<personbox.at(1)<<","<<personbox.at(2)<<","<<personbox.at(3)<<std::endl;
    }
    else
    {
        std::cout<<"person not found"<<std::endl;   
    } 
    
    std::cout<<"classname :";   
    for(int i=0;i<classname.size();i++)
        std::cout << classname.at(i) << " ";
    std::cout <<""<< std::endl;
    std::cout<<"classnames :";  
    for(int i=0;i<classnames.size();i++)
        std::cout << classnames.at(i) << " ";
    std::cout <<""<< std::endl;
    
    boxxmin2.resize(classnames.size());
    boxymin2.resize(classnames.size());
    boxxmax2.resize(classnames.size());
    boxymax2.resize(classnames.size());
    boxcenterx.resize(classnames.size());
    boxcentery.resize(classnames.size());
    activityscoreface.resize(classnames.size());
    activityscoreobject.resize(classnames.size()); 
    
    fill(boxxmin2.begin(), boxxmin2.end(),0);
    fill(boxymin2.begin(), boxymin2.end(),0);
    fill(boxymin2.begin(), boxymin2.end(),0);
    fill(boxymin2.begin(), boxymin2.end(),0);
    fill(boxcenterx.begin(), boxcenterx.end(),0);
    fill(boxcentery.begin(), boxcentery.end(),0);
    fill(activityscoreface.begin(), activityscoreface.end(),0);
    fill(activityscoreobject.begin(), activityscoreobject.end(),0);

    for(int i=0;i<classname.size();i++)
    {
        std::vector<std::string>::iterator citer3 = std::find(classnames.begin(),classnames.end(),classname.at(i));
        
        int indexbox = std::distance(classnames.begin(), citer3);
        boxxmin2.at(indexbox)=boxxmin.at(i);
        boxymin2.at(indexbox)=boxymin.at(i);
        boxxmax2.at(indexbox)=boxxmax.at(i);
        boxymax2.at(indexbox)=boxymax.at(i);
    }
    for(int i=0;i<classnames.size();i++)
    {        
        boxcenterx.at(i) = (boxxmin2.at(i)+boxxmax2.at(i))/2;
        boxcentery.at(i) = (boxymin2.at(i)+boxymax2.at(i))/2;
    }

    //if((!personbox.empty())&&robotstop&&personstop)
    //if((!personbox.empty())&&personstop)
    if((!personbox.empty()))
    {
        std::cout<<"boxxmin    :"<<" ";
        for(int i=0;i<boxxmin.size();i++)
        std::cout << boxxmin.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"boxxmin2  :"<<" ";
        for(int i=0;i<boxxmin2.size();i++)
        std::cout << boxxmin2.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"boxcenterx:"<<" ";
        for(int i=0;i<boxcenterx.size();i++)
        std::cout << boxcenterx.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"boxcentery:"<<" ";
        for(int i=0;i<boxcentery.size();i++)
        std::cout << boxcentery.at(i) << " ";
        std::cout <<""<< std::endl;
    
        CombiDarknetOpenface::Calculate_TimeUse(currenttimesec);
    }
    alltimerecord<<darknet_cnt<<","
    <<currenttimesec<<","
    <<robotstop<<","
    <<personstop_cnt<<","
    <<personstop<<std::endl; 
    
    frame_num += 1;
    */
}

void CombiDarknetOpenface::Calculate_TimeUse(double currenttimesec)
{
    int minindex = 0;
    if((!nose_end_point2D_drawtmp.empty())&&(!personbox.empty()))
    {
        std::cout << "object and face orientation" << std::endl;
        
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 570.3422241210938, 0, 319.5, 0 ,  570.3422241210938, 239.5, 0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming no lens distortion

        nose_end_point2D_draw.clear();
        nose_end_point2D_draw2.clear();
        nose_end_point2D_draw3.clear();
        nose_end_point2D_draw4.clear();
    
        vector<cv::Point3d> nose_end_point3D;
        vector<cv::Point2d> nose_end_point2D;
            
        nose_end_point3D.push_back(cv::Point3d(0,0,1000.0));
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
        nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].x));
        nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].y));
        
        nose_end_point2D.clear();
        nose_end_point3D.clear();        
        nose_end_point3D.push_back(cv::Point3d(0,0,500));
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            
        nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].x));
        nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].y));

        nose_end_point2D.clear();
        nose_end_point3D.clear();
        nose_end_point3D.push_back(cv::Point3d(0,0,1500));
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
        nose_end_point2D_draw3.push_back(std::round(nose_end_point2D[0].x));
        nose_end_point2D_draw3.push_back(std::round(nose_end_point2D[0].y));

        nose_end_point2D.clear();
        nose_end_point3D.clear();
        nose_end_point3D.push_back(cv::Point3d(0,0,2000));
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
        nose_end_point2D_draw4.push_back(std::round(nose_end_point2D[0].x));
        nose_end_point2D_draw4.push_back(std::round(nose_end_point2D[0].y));

        cout << "nose_end_point2D_draw  :"<<nose_end_point2D_draw[0]<<","<<nose_end_point2D_draw[1]<< endl;
        cout << "nose_end_point2D_draw2:"<<nose_end_point2D_draw2[0]<<","<<nose_end_point2D_draw2[1]<< endl;
        cout << "nose_end_point2D_draw3:"<<nose_end_point2D_draw3[0]<<","<<nose_end_point2D_draw3[1]<< endl;
        cout << "nose_end_point2D_draw4:"<<nose_end_point2D_draw4[0]<<","<<nose_end_point2D_draw4[1]<< endl;
        

        std::vector<float> objectdistance;
        std::vector<float> objectdistancetmp;

        for(int i=0;i<classnames.size();i++)
        {
            objectdistance.push_back(std::sqrt(std::pow(boxcenterx.at(i)-nose_end_point2D_draw.at(0) , 2) + std::pow(boxcentery.at(i)-nose_end_point2D_draw.at(0), 2)));
            if(classnames.at(i)=="person")
            {
                objectdistance.at(i) = 0;   
            }
            if(classnames.at(i)=="dining table")
            {
                objectdistance.at(i) = 0;   
            }
            if((boxcenterx.at(i) ==0)&&(boxcentery.at(i) ==0))
            {
                objectdistance.at(i) = 0;
            }

            if(objectdistance.at(i)>0)
            {
                objectdistancetmp.push_back(objectdistance.at(i));
            }
        }

        std::cout<<"objectdistance  :"<<" ";
        for(int i=0;i<objectdistance.size();i++)
        std::cout << objectdistance.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"objectdistancetmp  :"<<" ";
        for(int i=0;i<objectdistancetmp.size();i++)
        std::cout << objectdistancetmp.at(i) << " ";
        std::cout <<""<< std::endl;
       
        auto minobjectdistance = std::min_element(std::begin(objectdistancetmp), std::end(objectdistancetmp));
        std::cout << "*minobjectdistance:"<< *minobjectdistance << std::endl;
        float minobjectdistancetmp = *minobjectdistance;
        std::vector<float>::iterator citerobdist =std::find(objectdistance.begin(), objectdistance.end(), minobjectdistancetmp);
        if (citerobdist != objectdistance.end()) 
        {
            minindex = std::distance(objectdistance.begin(), citerobdist);
        }
        std::cout << "minindex:"<< minindex << std::endl;
        activityscoreface.at(minindex) += 1;  
    }
    
    int maxindex = 0;
    if((!classnames.empty())&&(!personbox.empty()))
    {
        std::cout << "object movement" << std::endl;
        std::vector<float> objectmovement; 
        std::vector<float> objectmovementtmp;
 
        if(lastboxcenterx.empty())
        {
            std::cout << "object movement init" << std::endl;       
            for(int i=0;i<classnames.size();i++)
            {   
                lastboxcenterx.push_back(boxcenterx.at(i));
                lastboxcentery.push_back(boxcentery.at(i));
            }
        }
        else
        {
            std::cout << "object movement calculate" << std::endl;
            std::cout<<"lastboxcenterx:"<<" ";
            for(int i=0;i<lastboxcenterx.size();i++)
            std::cout << lastboxcenterx.at(i) << " ";
            std::cout <<""<< std::endl;
            std::cout<<"lastboxcentery:"<<" ";
            for(int i=0;i<lastboxcentery.size();i++)
            std::cout << lastboxcentery.at(i) << " ";
            std::cout <<""<< std::endl;

            if(lastboxcenterx.size()==boxcenterx.size())
            {
                for(int i=0;i<classnames.size();i++)
                {
                    objectmovement.push_back(std::sqrt(std::pow(boxcenterx.at(i)-lastboxcenterx.at(i), 2) + std::pow(boxcentery.at(i)-lastboxcentery.at(i), 2)));
                    if(classnames.at(i)=="person")
                    {
                        objectmovement.at(i) = 0;   
                    }
                    if(classnames.at(i)=="dining table")
                    {
                        objectmovement.at(i) = 0;   
                    }
                    if((boxcenterx.at(i) ==0)&&(boxcentery.at(i) ==0))
                    {
                       objectmovement.at(i) = 0;
                    }

                    if(objectmovement.at(i)>0)
                    {
                        objectmovementtmp.push_back(objectmovement.at(i));
                    }
                }
                std::cout<<"objectmovement:"<<" ";
                for(int i=0;i<objectmovement.size();i++)
                std::cout << objectmovement.at(i) << " ";
                std::cout <<""<< std::endl;
                
                if(objectmovementtmp.empty())
                {
                    std::cout << "all movements are zero" << std::endl;
                    maxindex = 0;
                } 
                else
                {
                    std::cout<<"objectmovementtmp:"<<" ";
                    for(int i=0;i<objectmovementtmp.size();i++)
                    std::cout << objectmovementtmp.at(i) << " ";
                    std::cout <<""<< std::endl;
                    
                    auto maxobjectmovement = std::max_element(std::begin(objectmovementtmp), std::end(objectmovementtmp));
                    std::cout << "*maxobjectmovement: "<< *maxobjectmovement << std::endl;
                    float maxobjectmovementtmp = *maxobjectmovement;
                    std::vector<float>::iterator citerobmove =std::find(objectmovement.begin(), objectmovement.end(), maxobjectmovementtmp);
                    if (citerobmove != objectmovement.end()) 
                    {
                        maxindex = std::distance(objectmovement.begin(), citerobmove);
                    }
                    if(objectmovement.at(maxindex)>15)
                    {
                        std::cout << "object moving" << std::endl;
                    }
                    else
                    {
                        std::cout << "object stopping" << std::endl;
                        maxindex = 0;
                    }

                    for(int i=0;i<classnames.size();i++)
                    {
                        lastboxcenterx.at(i) = boxcenterx.at(i);
                        lastboxcentery.at(i) = boxcentery.at(i);
                    }        
                }
                activityscoreobject.at(maxindex) += 1;   
            }   
            else
            {
                std::cout << "add new object movement " << std::endl;
                lastboxcenterx.clear();
                lastboxcentery.clear();
                lastboxcenterx.resize(boxcenterx.size());
                lastboxcentery.resize(boxcentery.size());
                std::cout << "boxcenterx.size(): "<< boxcenterx.size() << std::endl;
                for(int i=0;i<classnames.size();i++)
                {
                    lastboxcenterx.at(i) = boxcenterx.at(i);
                    lastboxcentery.at(i) = boxcentery.at(i);
                }
            }
            std::cout << "maxindex: "<< maxindex << std::endl;
        }
    }

    if((!personbox.empty())&&(!estimateposition.empty()))
    //if((!activityscoreface.empty())&&(!activityscoreobject.empty()))
    {
        std::cout<<"activityscoreface:"<<" ";
        for(int i=0;i<activityscoreface.size();i++)
        std::cout << activityscoreface.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"activityscoreobject:"<<" ";
        for(int i=0;i<activityscoreobject.size();i++)
        std::cout << activityscoreobject.at(i) << " ";
        std::cout <<""<< std::endl;
        
        activityscorefacedata<<darknet_cnt<<","
         <<currenttimesec<<", "
         <<minindex<<std::endl;
        scorefacedata<<darknet_cnt<<","<<currenttimesec<<",";
        scorefacelabel<<darknet_cnt<<","<<currenttimesec<<",";
        activityscoreobjectdata<<darknet_cnt<<","
        <<currenttimesec<<", "
        <<maxindex<<std::endl;
        scoreobjectdata<<darknet_cnt<<","<<currenttimesec<<",";
        scoreobjectlabel<<darknet_cnt<<","<<currenttimesec<<",";

        for(int i=0;i<classnames.size();i++)
        {    
                if(i==(classnames.size()-1))
                {
                    scorefacedata<< activityscoreface.at(i);
                    scoreobjectdata<< activityscoreobject.at(i);
                    scorefacelabel<<classnames.at(i);
                    scoreobjectlabel<<classnames.at(i);
                }
                else
                 {
                    scorefacedata<< activityscoreface.at(i)<<",";
                    scoreobjectdata<< activityscoreobject.at(i)<<",";
                    scorefacelabel<<classnames.at(i)<<",";
                    scoreobjectlabel<<classnames.at(i)<<",";
                }
        }
        scorefacedata<<std::endl;
        scoreobjectdata<<std::endl;  
        scorefacelabel<<std::endl;
        scoreobjectlabel<<std::endl;  

         timeusedata<<darknet_cnt<<","
        <<currenttimesec<<", "
        << personvelocity.at(0)<<", "
        << personvelocity.at(1)<<", "
        << estimateposition.at(0)<<", "
        << estimateposition.at(1)<<std::endl;  
    }
    std::cout <<"darknet callback end"<< std::endl;
    std::cout <<""<< std::endl;
}

//Darknet
void CombiDarknetOpenface::rgbObjectImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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

    cv::resize(rgb_im, rgb_im, cv::Size(), ResizeSize, ResizeSize);
    
    cv::imshow("RGB darknet image", rgb_im);
    //cvMoveWindow("RGB darknet image", 1000,0);
    cvMoveWindow("RGB darknet image", 990,0);
    cv::waitKey(10);
}

//RGB
void CombiDarknetOpenface::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    int i, j;
    int x1, x2, y1, y2;
    int width = WIDTH;
    int height = HEIGHT;
    static int lastdarknetcnt = 0;

    cv_bridge::CvImagePtr cv_ptr;

    /*
     if (darknet_cnt!=lastdarknetcnt)
    {
        std::cout<<"image saved"<<std::endl;
        cv::imwrite(std::to_string(darknet_cnt)+".png", cv_bridge::toCvShare(msg, "bgr8")->image);    
        lastdarknetcnt = darknet_cnt;
    }*/

    
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    //std::cout<<"RGB image_captute"<<std::endl;
    rgb_cnt += 1;
    std::cout<<"rgb_callback:"<<rgb_cnt<<std::endl;

    if(!nose_tip.empty())
    {
        cv::circle(cv_ptr->image, cv::Point(nose_tip[0], nose_tip[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(chin[0], chin[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_eye[0], left_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_eye[0], right_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_mouth[0], left_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_mouth[0], right_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);

        //cout << "nose_end_point2D_draw:"<<nose_end_point2D_draw[0] <<", "<<nose_end_point2D_draw[1]<< endl;
        
        //cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),nose_end_point2D_draw, cv::Scalar(255,0,0), 2);
    }
    //std::cout<< " " <<std::endl;

    if(!personbox.empty())
    {
        cv::putText(cv_ptr->image, "person found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }
    else
    {
        cv::putText(cv_ptr->image, "person not found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }
    
    if((!nose_end_point2D_draw.empty())&&(!personbox.empty())&&(!activityscoreface.empty()))
    {
        int zerocntface = std::count(activityscoreface.begin(), activityscoreface.end(), 0);
        int maxindexface = 0;
            
        if(!(zerocntface==activityscoreface.size()))
        {
            auto maxactivityscoreface = std::max_element(std::begin(activityscoreface), std::end(activityscoreface));
            float maxactivityscorefacetmp = *maxactivityscoreface;
            std::vector<int>::iterator citeracscoreface =std::find(activityscoreface.begin(), activityscoreface.end(), maxactivityscorefacetmp);
            if (citeracscoreface != activityscoreface.end()) 
            {
                maxindexface = std::distance(activityscoreface.begin(), citeracscoreface);
            }
        }
        else
        {
            maxindexface = 0;
        }
        //std::cout << "maxindexface:"<< maxindexface << std::endl;
        cv::putText(cv_ptr->image, classnames.at(maxindexface), cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
        cv::rectangle(cv_ptr->image, cv::Point(boxxmin2.at(maxindexface), boxymin2.at(maxindexface)), cv::Point(boxxmax2.at(maxindexface), boxymax2.at(maxindexface)), cv::Scalar(255, 0, 0), 3, 4);
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw[0], nose_end_point2D_draw[1]), 5, cv::Scalar(255, 255,0), 3);
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw2[0], nose_end_point2D_draw2[1]), 5, cv::Scalar(0,255,255), 3);
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw3[0], nose_end_point2D_draw3[1]), 5, cv::Scalar(255,0,255), 3);
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw4[0], nose_end_point2D_draw4[1]), 5, cv::Scalar(255,255,255), 3);
        cv::circle(cv_ptr->image, cv::Point(boxcenterx[maxindexface], boxcentery[maxindexface]), 5, cv::Scalar(255,0,0), 3);

        cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw[0],nose_end_point2D_draw[1]), cv::Scalar(255,0,0), 2);
    
    }
    
    
    /*
    if(((!classnames.empty())&&(!personbox.empty())&&(!activityscoreobject.empty())))
    {
        int zerocntobject = std::count(activityscoreobject.begin(), activityscoreobject.end(), 0);
        
        int maxindexobject = 0;
        if(!(zerocntobject==activityscoreobject.size()))
        {
            auto maxactivityscoreobject = std::max_element(std::begin(activityscoreobject), std::end(activityscoreobject));
            float maxactivityscoreobjecttmp = *maxactivityscoreobject;
            std::vector<int>::iterator citeracscoreobject =std::find(activityscoreobject.begin(), activityscoreobject.end(), maxactivityscoreobjecttmp);
            if (citeracscoreobject != activityscoreobject.end()) 
            {
                maxindexobject = std::distance(activityscoreobject.begin(), citeracscoreobject);
            }
        }
        else
        {
            maxindexobject = 0;
        }
        cv::putText(cv_ptr->image, classnames.at(maxindexobject), cv::Point(20,85), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,200,200), 2,CV_AA);
        cv::rectangle(cv_ptr->image, cv::Point(boxxmin2.at(maxindexobject), boxymin2.at(maxindexobject)), cv::Point(boxxmax2.at(maxindexobject), boxymax2.at(maxindexobject)), cv::Scalar(0, 200, 200), 3, 4);
        //cv::circle(cv_ptr->image, cv::Point(boxcenterx[maxindexobject], boxcentery[maxindexobject]), 5, cv::Scalar(0, 255,0), 3);    
    }
    */

    
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), ResizeSize, ResizeSize);

    cv::imshow("RGB image", cv_ptr->image);

    cvMoveWindow("RGB image", 990,500);
    cv::waitKey(10);


}

void CombiDarknetOpenface::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    int x1ori = 0;
    int x2ori = 0;
    int y1ori = 0;
    int y2ori = 0;
    int x1 = 0;
    int x2 = 0;
    int y1 = 0;
    int y2 = 0;
    int xc = 0;
    int yc = 0;

    double distancetmp = 0.0;
    
    int i, j, k;
    int width = WIDTH;
    int height = HEIGHT;
    double sum = 0.0;
    double distance;
    double theta,theta2;
    cv_bridge::CvImagePtr cv_ptr;

    static ros::Time firsttime = ros::Time::now();
     ros::Time nowtime = ros::Time::now();

     double firsttimesec,nowtimesec,currenttimesec;
     firsttimesec = firsttime.toSec();
     nowtimesec = nowtime.toSec();
     currenttimesec = nowtimesec-firsttimesec;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
    
    cv::Mat depth(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    
    //cv::normalize(depth, depth, 1, 0, cv::NORM_MINMAX);
    
    cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    cv::normalize(img, img, 1, 0, cv::NORM_MINMAX);

    
    if(!personbox.empty())
    {
        depth_cnt += 1;
        std::cout<<"depth_callback:"<<depth_cnt<<std::endl;
    
        x1ori = personbox.at(0);
        y1ori = personbox.at(1);
        x2ori = personbox.at(2);
        y2ori = personbox.at(3);

        xc = (x1ori+x2ori)/2;
        yc = (y1ori+y2ori)/2;

        x1 = xc-1;
        y1 = yc-1;
        x2 = xc+1;
        y2 = yc+1;
    
        int depth_sum_cnt = 0;
        for(i = 0; i < cv_ptr->image.rows;i++)
        {
            float* Dimage = cv_ptr->image.ptr<float>(i);
            float* Iimage = depth.ptr<float>(i);
            char* Ivimage = img.ptr<char>(i);
            for(j = 0 ; j < cv_ptr->image.cols; j++)
            {
                if(Dimage[j] > 0.0)
                {
                    Iimage[j] = Dimage[j];
                    Ivimage[j] = (char)(255*(Dimage[j]/5.5));
                }
                else
                {
                }

                if(i > y1 && i < y2)
                {
                    if(j > x1 && j < x2)
                    {
                        if(Dimage[j] > 0.0)
                        {
                            sum += Dimage[j];
                            depth_sum_cnt +=1;
                            std::cout<<"point:"<<j<<","<<i<<std::endl;
                            std::cout<<"depth_sum_cnt:"<<depth_sum_cnt<<std::endl;
                        }
                    }
                }
            }
        }  
        std::cout<<"box_center_depth:"<<xc<<","<<yc<<std::endl;
        
        //distance = sum / ((x2-x1)*(y2-y1));
        distance = sum ;
    
    
        //theta2 = ((double)xc/640)*AngleofView;
        //theta2 = (-1)*(theta2-AngleofView/2);
    
        theta = AngleofView/2-((double)xc/640)*AngleofView;
    
        /*
        floator(int i=0;i<classname.size();i++)
        std::cout << classname.at(i) << " ";
        std::cout << std::endl;
        
        //std::cout<<"box_depth_ori:"<<x1ori<<","<<y1ori<<","<<x2ori<<","<<y2ori<<std::endl;
        //std::cout<<"area:"<<(x2ori-x1ori)*(y2ori-y1ori)<<","<< (x2-x1)*(y2-y1)<<std::endl;
        
        //std::cout<<"distance:"<<distance<<std::endl;
        //std::cout<<"box:"<<x1<<", "<<y1<<", "<<x2<<", "<<y2<<std::endl;
        */

        distancetmp = distance;
        CombiDarknetOpenface::ModifyPersonDistance(&distance);
        
        double measurementx = distance * cos(theta/180.0*M_PI);
        double measurementy = distance * sin(theta/180.0*M_PI);
    
        CombiDarknetOpenface::PublishPersonMeasurement(measurementx,measurementy);
        CombiDarknetOpenface::PublishPersonMarker(theta,measurementx,measurementy);
        
        cv::rectangle(img, cv::Point(x1ori, y1ori), cv::Point(x2ori, y2ori), cv::Scalar(0, 0, 255), 5, 4);
        //cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 3, 4);
        cv::circle(img, cv::Point(xc, yc), 8, cv::Scalar(0, 0, 0), 3);
    }
    //std::cout<<"area_error:"<<area_error<<std::endl;
    
    cv::resize(img, img, cv::Size(), ResizeSize, ResizeSize);
    cv::imshow("Depth_Image", img);
    cvMoveWindow("Depth_Image", 1450,0);
    //cvMoveWindow("Depth_Image", 10,0);
    cv::waitKey(10);
    
     depthdata<<darknet_cnt<<","
     <<currenttimesec<<", "
     << distancetmp<<", "
     << distance<<", "
     << xc<<", "
     << yc<<", "
     <<modify_distance_cnt<<","
     <<personstop_cnt<<","
     << personstop<<std::endl;  

     std::cout<<""<<std::endl;   
}

void CombiDarknetOpenface::ModifyPersonDistance(double *distance)
{
     static double lastdistance;
     static int init = 0;

     //std::cout<<"lastheadori[2]_before:"<<lastheadori[2]<<","<<"headori[2]:"<<headori[2]<<","<<"diff:"<<lastheadori[2]-headori[2]<<std::endl;
 
     if(!init)
     {
        lastdistance =  *distance;
        init = 1;
     }

     //if(abs(lastdistance - *distance)>0.5)
     if (*distance == 0)
     {
        modify_distance_cnt += 1;
        //std::cout<<"modify_distance:"<<lastdistance - *distance<<std::endl;
        std::cout<<"modify_distance_cnt:"<<modify_distance_cnt<<std::endl;
        *distance = lastdistance;
     }
     else
     {
        lastdistance = *distance;   
     }
    /*else if(abs(lastdistance - *distance)>0.4)
    {
        modify_distance_cnt += 1;
        //std::cout<<"modify_distance:"<<lastdistance - *distance<<std::endl;
        std::cout<<"modify_distance_cnt:"<<modify_distance_cnt<<std::endl;
        *distance = lastdistance;
     }*/   
}

void CombiDarknetOpenface::msgCallback_FilterMsg(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        static ros::Time firsttime = ros::Time::now();
        ros::Time nowtime = ros::Time::now();

        double firsttimesec,nowtimesec,currenttimesec;
        firsttimesec = firsttime.toSec();
        nowtimesec = nowtime.toSec();
        currenttimesec = nowtimesec-firsttimesec;

        kf_cnt += 1;
        std::cout<<"KF_callback:"<<kf_cnt<<std::endl;
        lastestimateposition.clear();
        if(kf_cnt>1)
        {
            lastestimateposition.push_back(estimateposition[0]);
            lastestimateposition.push_back(estimateposition[1]);
        }
        else
        {
            lastestimateposition.push_back(msg->pose.position.x);
            lastestimateposition.push_back(msg->pose.position.y);
        }
        estimateposition.clear();
        estimateposition.push_back(msg->pose.position.x);
        estimateposition.push_back(msg->pose.position.y);

        personvelocity.clear();
        if(kf_cnt>1)
        {
            //personvelocity.push_back((estimateposition.at(0) - lastestimateposition.at(0))/(currenttimesec - lastcurrenttimevelocity));
            //personvelocity.push_back((estimateposition.at(1) - lastestimateposition.at(1))/(currenttimesec - lastcurrenttimevelocity));
            personvelocity.push_back(estimateposition.at(0) - lastestimateposition.at(0));
            personvelocity.push_back(estimateposition.at(1) - lastestimateposition.at(1));
        }
        else
        {
            personvelocity.push_back(0.0);
            personvelocity.push_back(0.0);
        }

        if(personbox.empty())
        {
            personstop = 0; 
        }
        else if((abs(personvelocity.at(0))>0.01)&&(abs(personvelocity.at(1))>0.01))
        {
            personstop_cnt += 1;
        }
        else if ((abs(personvelocity.at(0))>0.02)||(abs(personvelocity.at(1))>0.02))
        {
            personstop_cnt += 1;   
        }
        else
        {
            personstop = 1;
            personstop_cnt = 0;
        }

        if(personstop_cnt>4)
        {
            personstop=0;
        }
        
        lastcurrenttimevelocity  = currenttimesec;

        std::cout<<"personstop:"<<personstop<<std::endl;
        
        //std::cout<<"estimateposition:"<<estimateposition[0]<<","<<estimateposition[1]<<std::endl;
        //std::cout<<"lastestimateposition:"<<lastestimateposition[0]<<","<<lastestimateposition[1]<<std::endl;
         //std::cout<<"personvelocity:"<<personvelocity[0]<<","<<personvelocity[1]<<std::endl;
        personvelocitydata<<darknet_cnt<<","
        <<currenttimesec<<", "
        << personvelocity.at(0)<<", "
        << personvelocity.at(1)<<", "
        << personstop_cnt<<", "
        <<personstop<<std::endl;

        std::cout<<""<<std::endl;   
}

void CombiDarknetOpenface::PublishPersonMeasurement(double measurementx, double measurementy)
{
    geometry_msgs::PoseStamped inputpose;
 
    inputpose.pose.position.x = measurementx;
    inputpose.pose.position.y = measurementy;
    
    double disterror;
    if(estimateposition.empty())
    {       
        inputpose.pose.position.z = 1;
        measurement_pub.publish(inputpose);
        //std::cout <<"filter init" <<std::endl;
    }
    else    
    {
        disterror = std::sqrt(std::pow(measurementx-estimateposition[0] , 2) + std::pow(measurementy-estimateposition[1], 2));
        //std::cout <<"disterror: " << disterror<<std::endl;
     
        if(disterror<1.2)
        {
            kf_failed_cnt = 0;
            inputpose.pose.position.z = 0;
            measurement_pub.publish(inputpose);
            //std::cout <<"publish measurement" <<std::endl;
        }
        else
        {
            if(kf_failed_cnt == 5)
            {
                inputpose.pose.position.z = 1;
                measurement_pub.publish(inputpose);
                //std::cout <<"filter_reset" <<std::endl;
                kf_failed_cnt = 0;
            }
            else
            {
                kf_failed_cnt += 1;
                //std::cout <<"tracking failed: " << kf_failed_cnt<<std::endl;
            }
        }
    }   
}

void CombiDarknetOpenface::PublishHeadposeArrow()
{
    visualization_msgs::Marker headposearrow;
    double arrowtheta = 0;

    if((!estimateposition.empty())&&(!headorientation.empty()))
    {
        headposearrow.header.frame_id = "/base_link";
        headposearrow.header.stamp = ros::Time::now();
        headposearrow.ns = "basic_shapes";
        headposearrow.type = visualization_msgs::Marker::ARROW;
        headposearrow.action = visualization_msgs::Marker::ADD;
   
        headposearrow.pose.position.x = estimateposition[0];
        headposearrow.pose.position.y = estimateposition[1];
        //std::cout<<"headposearrow.pose:"<<headposearrow.pose.position.x<<","<<headposearrow.pose.position.y<<std::endl;
                
        if((0<=headorientation[2])&&(headorientation[2]<90))
        {
            arrowtheta = 180-headorientation[2];
            //std::cout<<"headposearrow.orientation:"<<headorientation[2]<<", "<<arrowtheta<<std::endl;    
      
        }
        else if((-90<=headorientation[2])&&(headorientation[2]<0))
        {
            arrowtheta = std::abs(headorientation[2])+180;
            //std::cout<<"headposearrow.orientation:"<<headorientation[2]<<", "<<arrowtheta<<std::endl;       
        }
        else
        {
            arrowtheta = headorientation[2];
            //std::cout<<"out of yaw range :"<<std::endl;
        }
        
        //headposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(-180/180.0*M_PI);

        headposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(arrowtheta/180.0*M_PI);
        //headposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(headorientation[2]/180.0*M_PI);


        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        headposearrow.scale.x = 0.3;
        headposearrow.scale.y = 0.1;
        headposearrow.scale.z = 0.1;
        // Set the color -- be sure to set alpha to something non-zero!
        headposearrow.color.r = 0.0f;
        headposearrow.color.g = 0.0f;
        headposearrow.color.b = 1.0f;
        headposearrow.color.a = 1.0f;

        headposearrow.lifetime = ros::Duration(0.5);
        headpose_arrow_pub.publish(headposearrow);
       
    }
    
}

void CombiDarknetOpenface::PublishPersonMarker(double theta, double measurementx, double measurementy)
{
    visualization_msgs::Marker personarrow;
    
    personarrow.header.frame_id = "/base_link";
    personarrow.header.stamp = ros::Time::now();
    personarrow.ns = "basic_shapes";
    personarrow.type = visualization_msgs::Marker::ARROW;
    personarrow.action = visualization_msgs::Marker::ADD;
    
    //std::cout<<"personarrow.pose.orientation:"<<theta<<std::endl;
    personarrow.pose.orientation=tf::createQuaternionMsgFromYaw(theta/180.0*M_PI);

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    personarrow.scale.x = 0.3;
    personarrow.scale.y = 0.1;
    personarrow.scale.z = 0.1;
    // Set the color -- be sure to set alpha to something non-zero!
    personarrow.color.r = 1.0f;
    personarrow.color.g = 0.0f;
    personarrow.color.b = 0.0f;
    personarrow.color.a = 1.0f;

    personarrow.lifetime = ros::Duration();
    person_arrow_pub.publish(personarrow);

    visualization_msgs::Marker personmarker;

    personmarker.header.stamp = ros::Time::now();
    personmarker.header.frame_id = "/base_link";
    personmarker.pose.position.x = measurementx;
    personmarker.pose.position.y = measurementy;
    //personmarker.lifetime = ros::Duration(1);
    personmarker.scale.x = 0.2;
    personmarker.scale.y = 0.2;
    personmarker.scale.z = 0.2;
    personmarker.color.a = 0.5f;
    personmarker.color.r = 1.0f;
    personmarker.type = personmarker.SPHERE;
    person_marker_pub.publish(personmarker);

    visualization_msgs::Marker originmarker;

    originmarker.header.stamp = ros::Time::now();
    originmarker.header.frame_id = "/base_link";
    originmarker.pose.position.x = 0.0;
    originmarker.pose.position.y = 0.0;
    //personmarker.lifetime = ros::Duration(1);
    originmarker.scale.x = 0.1;
    originmarker.scale.y = 0.1;
    originmarker.scale.z = 0.1;
    originmarker.color.a = 1.0f;
    originmarker.color.r = 1.0;
    originmarker.color.g = 1.0;
    originmarker.color.b = 1.0;
    
    originmarker.type = originmarker.SPHERE;
    origin_marker_pub.publish(originmarker);
}
// 購読者ノードのメイン関数
int main(int argc, char **argv)
{
    // ノード名の初期化
    ros::init(argc, argv, "combi_darknet_openface");

    headposedata<<"time"<<","
    <<"head_pose_x"<<","
    <<"head_pose_y"<<","
    <<"head_pose_z"<<","
    <<"head_roll"<<","
    <<"head_pitch"<<","
    <<"head_yaw"<<std::endl;

    timeusedata<<"cnt"<<","
    <<"time"<<","
    <<"velocity_x"<<","
    <<"velocity_y"<<","
    <<"position_x"<<","
    <<"position_y"<<","
    <<"class"<<std::endl;

    activityscorefacedata<<"cnt"<<","
    <<"time"<<","
    <<"class"<<std::endl;

    activityscoreobjectdata<<"cnt"<<","
    <<"time"<<","
    <<"class"<<std::endl;

    scorefacedata<<"cnt"<<","
    <<"time"<<","
    <<"score"<<std::endl;

    scoreobjectdata<<"cnt"<<","
    <<"time"<<","
    <<"score"<<std::endl;

    scorefacelabel<<"cnt"<<","
    <<"time"<<","
    <<"label"<<std::endl;

    scoreobjectlabel<<"cnt"<<","
    <<"time"<<","
    <<"label"<<std::endl;

    alltimerecord<<"cnt"<<","
    <<"time"<<","
    <<"robotstop"<<","
    <<"personstop_cnt"<<","
    <<"personstop"<<std::endl;

    personvelocitydata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"velocity_x"<<","
    <<"velocity_y"<<","
    <<"personstop_cnt"<<","
    <<"personstop"<<std::endl;

    robotcmdveldata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"msg.linear.x"<<","
    <<"msg.linear.y"<<","
    <<"msg.angular.z"<<","
    <<"robotstop"<<std::endl;

    depthdata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"distancetmp"<<","
    <<"distance"<<","
    <<"xc"<<","
    <<"yc"<<","
    <<"modify_distance_cnt"<<","
    <<"personstop_cnt"<<","
    <<"personstop"<<std::endl;

    

    ros::NodeHandle nh;
    CombiDarknetOpenface pt(nh);
    
    ros::spin();
    return 0;
}
