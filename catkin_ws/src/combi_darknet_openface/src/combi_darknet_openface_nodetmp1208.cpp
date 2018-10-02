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

//#include <naoqi_bridge_msgs/JointAnglesWithSpeedActionGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tbb/tbb.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "combi_darknet_openface/ActionUnit.h"
#include "combi_darknet_openface/Face.h"
#include "combi_darknet_openface/Faces.h"

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
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
#include <algorithm>

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

#define ResizeSize 0.5

#define OrientationInView 1
#define OrientationOutView 2
#define OrientationFront 3
#define RobotMoveCount 5
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
int robotpose_cnt = 0;

int area_error = 0;

int robotmove = 0;//0:stoping,1:moving:
int robotmove_cnt = 0;
int personmove = 1;//0:stoping,1:moving
int personmove_cnt = 0;

int display_num = 2;

int notmeasurement_cnt=0;
int notmeasurement=0;////0:stoping,1:ok_move
int movemode = 0;

double lastcurrenttimevelocity  = 0;

static std::string fixed_frame = "map";

std::ofstream headposedata("headposedata.csv", std::ios::trunc);
std::ofstream facefeatures("facefeatures.csv", std::ios::trunc);
std::ofstream timeusedata("timeusedata.csv", std::ios::trunc);
std::ofstream activityscorefacedata("activityscorefacedata.csv", std::ios::trunc);
std::ofstream activityscoreobjectdata("activityscoreobjectdata.csv", std::ios::trunc);
std::ofstream timerecordfacedata("timerecordfacedata.csv", std::ios::trunc);
std::ofstream scorefacedata("scorefacedata.csv", std::ios::trunc);
std::ofstream scoreobjectdata("scoreobjectdata.csv", std::ios::trunc);
std::ofstream scorefacelabel("scorefacelabel.csv", std::ios::trunc);
std::ofstream scoreobjectlabel("scoreobjectlabel.csv", std::ios::trunc);
std::ofstream robotcmdveldata("robotcmdveldata.csv", std::ios::trunc);
std::ofstream personvelocitydata("personvelocitydata.csv", std::ios::trunc);
std::ofstream robotvelocitydata("robotvelocitydata.csv", std::ios::trunc);
std::ofstream depthdata("depthdata.csv", std::ios::trunc);
std::ofstream alltimerecord("alltimerecord.csv", std::ios::trunc);
std::ofstream noseobjecttheta("noseobjecttheta.csv", std::ios::trunc);

class CombiDarknetOpenface
{
public:
    CombiDarknetOpenface(ros::NodeHandle nh);
    ~CombiDarknetOpenface();
    void msgCallback_FaceRecognition(const combi_darknet_openface::Faces::ConstPtr& msg );
    void ModifyHeadOrientation();
    void ModifyPersonDistance(double* distance);
    void Publish_Velocity(double currenttimesec);
    void Calculate_TimeUse(double currenttimesec);
    void msgCallback_ObjectRecognition(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void rgbObjectImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);//face_feature
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void msgCallback_FilterMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void msgCallback_RobotPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void PublishPersonMeasurement(double measurementx, double measurementy);//input for KF
    void PublishPersonMarker(double theta, double measurementx, double measurementy);//input for KF
    void PublishHeadposeArrow();//input for KF
    void ChangeViewPoint(double currenttimesec);
        

private:
    ros::Subscriber ros_object_sub;
    ros::Subscriber rgb_object_sub;
    ros::Subscriber cmd_vel_sub;
    
    ros::Subscriber ros_face_sub;
    ros::Subscriber rgb_img_sub;
    ros::Subscriber depth_img_sub;

    ros::Subscriber ros_filtered_sub;
    ros::Publisher measurement_pub;
    
    ros::Subscriber ros_robotpose_sub;
    
    ros::Publisher headpose_arrow_pub;

    ros::Publisher origin_marker_pub;
    ros::Publisher person_arrow_pub;
    ros::Publisher person_marker_pub;
    ros::Publisher estimate_marker_pub;
    ros::Publisher cnt_text_pub;
    ros::Publisher destination_marker_pub;

    ros::Publisher robotpose_arrow_pub;
    ros::Publisher target_robotpose_pub;

    ros::Publisher velocity_pub;
    ros::Publisher pepper_speech_pub;
    ros::Publisher capture_cnt_pub;

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
    std::vector<float>timerecordface;

    std::vector<double>estimateposition;
    std::vector<double>lastestimateposition;
    std::vector<double>personvelocity;

    std::vector<double>robotpose;
    double robotyaw;
    geometry_msgs::PoseStamped robotoriginpose;
    std::vector<double>lastrobotpose;
    std::vector<double>robotvelocity;

    std::vector<int>personbox; 
    
    std::vector<double>headorientation;    
    std::vector<int>nose_tip;
    std::vector<int>chin;
    std::vector<int>left_eye;
    std::vector<int>right_eye;
    std::vector<int>left_mouth;
    std::vector<int>right_mouth;
    double headarrowtheta;

    std::vector<int>nose_end_point2D_drawtmp;
    std::vector<int>nose_end_point2D_draw;
    std::vector<int>nose_end_point2D_draw2;
    std::vector<int>nose_end_point2D_draw3;
    std::vector<int>nose_end_point2D_draw4;
    std::vector<int>nose_end_point2D_drawmin;    

    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;
    std::vector<double>lastrotation_value;   
    std::vector<double>lasttranslation_value;   
};

CombiDarknetOpenface::CombiDarknetOpenface(ros::NodeHandle nh):
nh1(nh)
{
        ros_object_sub = nh1.subscribe("darknet_ros/bounding_boxes",1, &CombiDarknetOpenface::msgCallback_ObjectRecognition, this);
        rgb_object_sub = nh1.subscribe<sensor_msgs::Image>("/darknet_ros/detection_image", 1, &CombiDarknetOpenface::rgbObjectImageCallback, this);
        
        ros_face_sub = nh1.subscribe("faces",1, &CombiDarknetOpenface::msgCallback_FaceRecognition, this);
        rgb_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &CombiDarknetOpenface::rgbImageCallback, this);
        depth_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &CombiDarknetOpenface::depthImageCallback, this);

        ros_filtered_sub= nh1.subscribe("estimate_pos",1, &CombiDarknetOpenface::msgCallback_FilterMsg, this);
        measurement_pub = nh1.advertise<geometry_msgs::PoseStamped>("filter_measurement", 1);
        ros_robotpose_sub = nh1.subscribe("/current_robot_pose",1, &CombiDarknetOpenface::msgCallback_RobotPoseMsg, this);
    
        headpose_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_headpose_arrow", 1);
        
        origin_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_origin_marker", 1);
        person_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_person_arrow", 1);
        person_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_person_marker", 1);

        estimate_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_estimateperson_marker", 1);
        cnt_text_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_cnt_txt", 1);
        
        
        robotpose_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_robotpose_arrow", 1);

        target_robotpose_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_target_robotpose_arrow", 1);
        
        
        destination_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_destination_marker", 1);
        
        
        velocity_pub=nh1.advertise<geometry_msgs::Twist>("cmd_vel",1);
        capture_cnt_pub = nh1.advertise<std_msgs::Int16>("/image_capture_cnt", 1);
        pepper_speech_pub = nh1.advertise<std_msgs::String>("/speech", 1);
        
}

CombiDarknetOpenface::~CombiDarknetOpenface()
{
}

void CombiDarknetOpenface::msgCallback_FaceRecognition(const  combi_darknet_openface::Faces::ConstPtr& msg )
{
    std::vector<combi_darknet_openface::Face> detectedfaces = msg->faces;
    
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

    for(std::vector<combi_darknet_openface::Face>::iterator itr = detectedfaces.begin(); itr != detectedfaces.end() ; ++itr)
    {   
        static ros::Time firsttime = ros::Time::now();
        ros::Time nowtime = ros::Time::now();

        double firsttimesec,nowtimesec,currenttimesec;

        firsttimesec = firsttime.toSec();
        nowtimesec = nowtime.toSec();

        currenttimesec = nowtimesec-firsttimesec;

        int i = 0;
        //std::cout<<"landmarks_2d.size:"<<(*itr).landmarks_2d.size()<<std::endl;
        if((*itr).landmarks_2d.size())
        {
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

            image_points.push_back( cv::Point2f(nose_tip[0], nose_tip[1])) ; // Nose tip
            image_points.push_back( cv::Point2f(chin[0], chin[1])) ; // Chin
            image_points.push_back( cv::Point2f(left_eye[0], left_eye[1])) ; // Left eye left corner
            image_points.push_back( cv::Point2f(right_eye[0], right_eye[1])) ;// Right eye left corner
            image_points.push_back( cv::Point2f(left_mouth[0], left_mouth[1])) ;// Left Mouth corner
            image_points.push_back( cv::Point2f(right_mouth[0], right_mouth[1])) ; // Right Mouth corner

            float facemodelscale = 0.225;
            model_points.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));// Nose tip
            model_points.push_back(cv::Point3f(0.0f, -330.0*facemodelscale, -65.0*facemodelscale));// Chin
            model_points.push_back(cv::Point3f(-225.0*facemodelscale, 170.0*facemodelscale, -135.0*facemodelscale));// Left eye left corner
            model_points.push_back(cv::Point3f(225.0*facemodelscale, 170.0*facemodelscale, -135.0*facemodelscale));// Right eye right corner
            model_points.push_back(cv::Point3f(-150.0*facemodelscale, -150.0*facemodelscale, -125.0*facemodelscale));// Left Mouth corner
            model_points.push_back(cv::Point3f(150.0*facemodelscale, -150.0*facemodelscale, -125.0*facemodelscale));// Right mouth corner

            //11/18
            cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 541.20870062659242, 0, 318.78756964392710, 0 ,  540.20435182225424, 236.43301053278904, 0, 0, 1);
            cv::Mat dist_coeffs = (cv::Mat_<double>(4,1) << 0.06569569924719, -0.25862424608946, 0.00010394071172, -0.00024019257963);
        
            rotation_vector  = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation in axis-angle form
            translation_vector = cv::Mat::zeros(3, 1, CV_64FC1);
            cv::Mat rotation_matrix  = cv::Mat::zeros(3, 3, CV_64FC1); // Rotation in axis-angle form
            
            cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector,CV_ITERATIVE);
            
            vector<cv::Point3f> nose_end_point3D;
            vector<cv::Point2f> nose_end_point2D;
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,250.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].y));

            cv::Mat rotation_vector2; // Rotation in axis-angle form
            cv::Mat translation_vector2;
            rotation_vector2 = rotation_vector;
            translation_vector2 = translation_vector;

            cv::Rodrigues(rotation_vector2, rotation_matrix);
        
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
            CombiDarknetOpenface::ModifyHeadOrientation();
            
            camera_matrix = (cv::Mat_<double>(3,3) << 541.20870062659242, 0, 318.78756964392710, 0 ,  540.20435182225424, 236.43301053278904, 0, 0, 1);
            dist_coeffs = (cv::Mat_<double>(4,1) << 0.06569569924719, -0.25862424608946, 0.00010394071172, -0.00024019257963);
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point2D_drawtmp.clear();
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,250.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_drawtmp.push_back(std::round(nose_end_point2D[0].y));

            nose_end_point2D_draw.clear();
            nose_end_point2D_draw2.clear();
            nose_end_point2D_draw3.clear();
            nose_end_point2D_draw4.clear();
            
            //vector<cv::Point3f> nose_end_point3D;
            //vector<cv::Point2f> nose_end_point2D;
            nose_end_point2D.clear();
            nose_end_point3D.clear();        
            
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,250.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].y));
            
            nose_end_point2D.clear();
            nose_end_point3D.clear();        
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,500.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
                
            nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].y));
        
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3f(0,0,750.0));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_draw3.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw3.push_back(std::round(nose_end_point2D[0].y));
            
            /*
            //commentout(11/21)
            std::cout << "nose_end_point2D_drawtmp:"<<nose_end_point2D_drawtmp[0] <<", "<<nose_end_point2D_drawtmp[1]<< std::endl;    
            std::cout << "nose_end_point2D_draw:"<<nose_end_point2D_draw[0] <<", "<<nose_end_point2D_draw[1]<< std::endl;
            std::cout << "nose_end_point2D_draw2:"<<nose_end_point2D_draw2[0] <<", "<<nose_end_point2D_draw2[1]<< std::endl;
            std::cout << "nose_end_point2D_draw3:"<<nose_end_point2D_draw3[0] <<",  "<<nose_end_point2D_draw3[1]<< std::endl;
            */

            CombiDarknetOpenface::PublishHeadposeArrow();

            cout << "headorientation:" << headorientation[0] <<", "<< headorientation[1] << ", "<< headorientation[2] << endl;

            headposedata<<currenttimesec<<", "
            <<(*itr).head_pose.position.x<<", "
            <<(*itr).head_pose.position.y<<", "
            <<(*itr).head_pose.position.z<<","
            <<headorientation[0]<<", "
            <<headorientation[1]<<", "
            <<headorientation[2]<<std::endl;
        }
    } 
    std::cout<<"face detect finish"<<std::endl;
    std::cout<< " " <<std::endl;
}

void CombiDarknetOpenface::ModifyHeadOrientation()
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
        
        std::cout<<"nose_end_point2D_drawtmptheta:"<<nose_end_point2D_drawtmptheta<<","<<lastnose_end_point2D_drawtmptheta<<","<<lastnose_end_point2D_drawtmptheta-nose_end_point2D_drawtmptheta<<std::endl;
        
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
        }*/

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

    if(darknet_cnt==1)
    {
        std_msgs::String test;
        test.data = "pepper";
        pepper_speech_pub.publish(test);

    }
    
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
                
    if(personindex)
    {
        //std::cout<<"personindex:"<<personindex-1<<std::endl;
        personbox.push_back(boxxmin.at(personindex));
        personbox.push_back(boxymin.at(personindex));
        personbox.push_back(boxxmax.at(personindex));
        personbox.push_back(boxymax.at(personindex));

        std::cout<<"person:";   
        for(int i=0;i<personbox.size();i++)
            std::cout << personbox.at(i) << " ";
        std::cout <<""<< std::endl;
    }
    else
    {
        std::cout<<"person is not found"<<std::endl;   
    } 
    
    /*commentout(11/21)
    std::cout<<"classname :";   
    for(int i=0;i<classname.size();i++)
        std::cout << classname.at(i) << " ";
    std::cout <<""<< std::endl;
    */
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

    if(timerecordface.size()!=classnames.size())
    {
        int sizediff = classnames.size()-timerecordface.size();
        for(int i = 0; i < sizediff; i++)
        {
            timerecordface.push_back(0);
        }
    }

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

    for(int i=0;i<classnames.size();i++)
    {        
        if(classnames.at(i)=="person")
        {
            boxcenterx.at(i) = 0;
            boxcentery.at(i) = 0;
        }
        else if(classnames.at(i)=="dining table")
        {
            boxcenterx.at(i) = 0;
            boxcentery.at(i) = 0;   
        }
    }

    //if((!personbox.empty())&&personmove)
    //if((!personbox.empty()))
    std::cout<<"robotmove:"<<robotmove<<std::endl;
    std::cout<<"personmove:"<<personmove<<std::endl;
    std::cout<<"notmeasurement_cnt:"<<notmeasurement_cnt<<std::endl;
    if(!nose_end_point2D_drawtmp.empty())
    {
        std::cout<<"headorientation[2]:"<<headorientation[2]<<std::endl;
    }
    std::cout<<"headarrowtheta:"<<headarrowtheta<<std::endl;
    
    //if((!personbox.empty())&&(!robotmove)&&(!personmove))
    if(((!personbox.empty())&&(!robotmove)&&(!personmove))&&(notmeasurement_cnt<RobotMoveCount))
    {
        std::cout << "###########measure timeuse ############"<< std::endl;
        /*commentout(11/21)
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
        */
        CombiDarknetOpenface::Calculate_TimeUse(currenttimesec);
        if(!robotpose.empty())
        {
            robotoriginpose.pose.position.x = robotpose.at(0);
            robotoriginpose.pose.position.y = robotpose.at(1);
            robotoriginpose.pose.position.z = 0.0;
            robotoriginpose.pose.orientation=tf::createQuaternionMsgFromYaw(robotyaw/180.0*M_PI);
        }      
    }
    else if(notmeasurement_cnt==RobotMoveCount)
    {
        std::cout << "###########robot moving############"<< std::endl;
        CombiDarknetOpenface::ChangeViewPoint(currenttimesec);
    }
    else if(personmove)
    {
        std::cout << "###########person_moving############"<< std::endl;        
    }
    else if(personbox.empty())
    {
        std::cout << "###########person_detecting############"<< std::endl;
        geometry_msgs::Twist twist;
        
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;
        velocity_pub.publish(twist);
    }
    else
    {
        std::cout << "###########else############"<< std::endl;
    }

    alltimerecord<<darknet_cnt<<","
    <<currenttimesec<<","
    <<robotmove_cnt<<","
    <<robotmove<<","
    <<personmove_cnt<<","
    <<personmove<<","
    <<notmeasurement_cnt<<std::endl; 
    
    std::cout <<""<< std::endl;
    
    frame_num += 1;
}

void CombiDarknetOpenface::ChangeViewPoint(double currenttimesec)
{
    geometry_msgs::Twist twist;
    double disterror = 0.0;

    if((!robotpose.empty())&&(!nose_end_point2D_drawtmp.empty()))
    {
        if(movemode==OrientationInView)
        {
            std::cout<<"movemode==OrientationInView"<<std::endl;
        }
        else if(movemode==OrientationOutView)
        {
            std::cout<<"movemode==OrientationInView"<<std::endl;
            
            std::cout<<"headarrowtheta:"<<headarrowtheta<<std::endl;
            if((90<=headarrowtheta)&&(headarrowtheta<180))
            {
                std::cout<<"test1"<<std::endl; 
            }
            else if((180<=headarrowtheta)&&(headarrowtheta<270))
            {
                std::cout<<"test2"<<std::endl; 
            }
            
            visualization_msgs::Marker targetrobotposearrow;
            targetrobotposearrow.header.frame_id = fixed_frame;
            targetrobotposearrow.header.stamp = ros::Time::now();
            targetrobotposearrow.ns = "basic_shapes";
            targetrobotposearrow.type = visualization_msgs::Marker::ARROW;
            targetrobotposearrow.action = visualization_msgs::Marker::ADD;
           
            /*
            targetrobotposearrow.pose.position.x = msg->pose.position.x;
            targetrobotposearrow.pose.position.y = msg->pose.position.y;
            targetrobotposearrow.pose.position.z = 0;
            targetrobotposearrow.pose.orientation.x = msg->pose.orientation.x;
            targetrobotposearrow.pose.orientation.y = msg->pose.orientation.y;
            targetrobotposearrow.pose.orientation.z = msg->pose.orientation.z;
            targetrobotposearrow.pose.orientation.w = msg->pose.orientation.w;     
            */

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            targetrobotposearrow.scale.x = 0.3;
            targetrobotposearrow.scale.y = 0.1;
            targetrobotposearrow.scale.z = 0.1;
            // Set the color -- be sure to set alpha to something non-zero!
            targetrobotposearrow.color.r = 1.0f;
            targetrobotposearrow.color.g = 0.0f;
            targetrobotposearrow.color.b = 0.0f;
            targetrobotposearrow.color.a = 1.0f;

        }   
        else if(movemode==OrientationFront)
        {
            std::cout<<"movemode==OrientationInView"<<std::endl;                       
        }
        /*
        std::cout<<"twist.linear.x:"<<twist.linear.z<<std::endl;
        std::cout<<"twist.linear.y:"<<twist.linear.z<<std::endl;
        std::cout<<"twist.angular.z:"<<twist.angular.z<<std::endl;
        */
        //velocity_pub.publish(twist);
        if(disterror<100)
        {
            notmeasurement_cnt = 0;
        }   
    }
    else
    {
        std::cout << "robot position error"<< std::endl; 
    }
}

void CombiDarknetOpenface::Publish_Velocity(double currenttimesec)
{
        geometry_msgs::Twist twist;
        double sigyaw = 0.0;

        twist.angular.z = sigyaw;

        //std::cout<<"sigyaw:"<<sigyaw<<std::endl;
        std::cout<<""<<std::endl;    

        //velocity_pub.publish(twist); 
}

void CombiDarknetOpenface::Calculate_TimeUse(double currenttimesec)
{
    int noseendminindex = 0;
    int mindistanceindex = 0;
    static double lastcurrenttimesec;
    if(darknet_cnt ==1)
    {
        lastcurrenttimesec = currenttimesec;
    }
    if((!nose_end_point2D_draw.empty())&&(!personbox.empty()))
    {
        //11/18
        cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 541.20870062659242, 0, 318.78756964392710, 0 ,  540.20435182225424, 236.43301053278904, 0, 0, 1);
        cv::Mat dist_coeffs = (cv::Mat_<double>(4,1) << 0.06569569924719, -0.25862424608946, 0.00010394071172, -0.00024019257963);
        vector<cv::Point3f> nose_end_point3D;
        vector<cv::Point2f> nose_end_point2D;
        
        std::cout<<"#######face orientation variable distance########" << std::endl;
        std::vector<int> distancestep;
        int i=0;
        for(int j=0;j<75;j++)
        {
            i += 10;
            distancestep.push_back(i);
        }
        
        std::vector<float> noseenddistance;
        std::vector<float> noseenddistancetmp;
        std::vector<int> eachminindex;
        std::vector<float> eachminnoseenddistancetmp;
        std::vector<float> eachnoseend;
        int noseendminindextmp = 0;
        int minnoseend = 10000;
        float minnoseenddistancetmp = 100000;
        int mindistancestep = 0;
        for(int i=0;i<distancestep.size();i++)
        {
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3d(0,0,distancestep.at(i)));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            noseenddistance.clear();
            noseenddistancetmp.clear();
            for(int j=0;j<classnames.size();j++)
            {
                noseenddistance.push_back(std::sqrt(std::pow(boxcenterx.at(j)-nose_end_point2D[0].x, 2) + std::pow(boxcentery.at(j)-nose_end_point2D[0].y, 2)));
                
                if((boxcenterx.at(j) == 0)&&(boxcentery.at(j) == 0))
                {
                    noseenddistance.at(j) = 0;
                }
                if(noseenddistance.at(j)>0)
                {
                    noseenddistancetmp.push_back(noseenddistance.at(j));
                }
            }
            if(noseenddistancetmp.empty())
            {
                std::cout << "noseenddistancetmp is empty" << std::endl;
                noseendminindextmp = 0;
                movemode = OrientationInView;
            }
            else
            {
                auto minnoseenddistance = std::min_element(std::begin(noseenddistancetmp), std::end(noseenddistancetmp));
                //std::cout << "*minnoseenddistance:"<< *minnoseenddistance << std::endl;
                minnoseenddistancetmp = *minnoseenddistance;
                std::vector<float>::iterator citernoseenddist =std::find(noseenddistance.begin(), noseenddistance.end(), minnoseenddistancetmp);
                if (citernoseenddist != noseenddistance.end()) 
                {
                    noseendminindextmp = std::distance(noseenddistance.begin(), citernoseenddist);
                }
            }
            //std::cout << "noseendminindextmp:"<< noseendminindextmp << std::endl;
            eachminindex.push_back(noseendminindextmp);
            if(minnoseenddistancetmp<minnoseend)
            {
                minnoseend = minnoseenddistancetmp;
                noseendminindex = noseendminindextmp;
                nose_end_point2D_drawmin.clear();
                nose_end_point2D_drawmin.push_back(std::round(nose_end_point2D[0].x));
                nose_end_point2D_drawmin.push_back(std::round(nose_end_point2D[0].y));
                mindistancestep = distancestep.at(i);
            }
            eachminnoseenddistancetmp.push_back(minnoseenddistancetmp);
            eachnoseend.push_back(minnoseend);
        }
        if(minnoseend>65)
        {
            std::cout << "gaze is not Assigned" << std::endl;
            noseendminindex = 0;
            movemode = OrientationInView;
        }
        else if(!(((nose_end_point2D_drawmin[0]>0)&&(nose_end_point2D_drawmin[0]<640))&&((nose_end_point2D_drawmin[1]>0)&&(nose_end_point2D_drawmin[1]<480))))
        {
            std::cout << "gaze is not Measured" << std::endl;
            noseendminindex = 0;
            movemode = OrientationOutView;
        }
        if(!noseendminindex)
        {
            double nosetoendmin = std::sqrt(std::pow(nose_end_point2D_drawmin[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_drawmin[1]-nose_tip[1], 2));
            
            double nosetoendconst = std::sqrt(std::pow(nose_end_point2D_draw3[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_draw3[1]-nose_tip[1], 2));
            
            if(nosetoendconst<nosetoendmin)
            {
                nose_end_point2D_drawmin[0] = nose_end_point2D_draw3[0];
                nose_end_point2D_drawmin[1] = nose_end_point2D_draw3[1];
                std::cout << "nose_end_point2D_drawmin modified"<<std::endl;
            }

            double nosetoend = std::sqrt(std::pow(nose_end_point2D_drawmin[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_drawmin[1]-nose_tip[1], 2));

            if(nosetoend<50)
            {
                movemode = OrientationFront;         
            }
            std::cout << "nosetoend:" << nosetoend <<std::endl;
        }
        
        //commentout(11/21)
        std::cout << "minnoseend:"<< minnoseend << std::endl;
        std::cout << "noseendminindex:"<< noseendminindex << std::endl;
        cout << "nose_end_point2D_drawmin  :"<<nose_end_point2D_drawmin[0]<<","<<nose_end_point2D_drawmin[1]<< endl;
        cout << "boxcenter  :"<<boxcenterx[noseendminindex]<<","<<boxcentery[noseendminindex]<< endl;
        
        std::cout << "mindistancestep:"<< mindistancestep << std::endl;
        std::cout << "movemode:"<< movemode << std::endl;
        
        activityscoreface.at(noseendminindex) += 1;

        if(noseendminindex)
        {
            notmeasurement_cnt = 0;
            movemode = 0;    
        }
        else
        {
            notmeasurement_cnt += 1;
        }

        std::cout << "notmeasurement_cnt:"<< notmeasurement_cnt << std::endl;
        
        //calculate gaze theta
        std::vector<float> noseobjecttmpx;
        std::vector<float> noseobjecttmpy;        
        float noseobjecthetatrue;
        std::vector<float> noseobjecthetatmp;
        float gazex;
        float gazey;
        float gazetheta;
        int nosethetaminindex = 0;
        std::vector<float> gazethetadiff;
        noseobjecttmpx.resize(classnames.size());
        noseobjecttmpy.resize(classnames.size()); 
        noseobjecthetatmp.resize(classnames.size());
        gazethetadiff.resize(classnames.size());
        fill(noseobjecttmpx.begin(), noseobjecttmpx.end(),10000);
        fill(noseobjecttmpy.begin(), noseobjecttmpy.end(),10000);
        fill(noseobjecthetatmp.begin(), noseobjecthetatmp.end(),10000);
        fill(gazethetadiff.begin(), gazethetadiff.end(),10000);        
        if(noseendminindex)
        {
            std::cout<<"########calculate gaze theta#############" << std::endl; 
            for(int i=0;i<classnames.size();i++)
            {
                noseobjecttmpx.at(i)= boxcenterx.at(i)-nose_tip[0];
                noseobjecttmpy.at(i)= boxcentery.at(i)-nose_tip[1];    
      
                if((boxcenterx.at(i) == 0)&&(boxcentery.at(i) == 0))
                    noseobjecthetatmp.at(i) = 10000;
                else if((noseobjecttmpx.at(i) == 0)||(noseobjecttmpy.at(i) == 0))
                    noseobjecthetatmp.at(i) = 0;
                else
                    noseobjecthetatmp.at(i) = atan2(noseobjecttmpy.at(i),noseobjecttmpx.at(i))* 180 / PI;
            }
            gazex = nose_end_point2D_drawmin.at(0)-nose_tip[0];
            gazey = nose_end_point2D_drawmin.at(1)-nose_tip[1];
            gazetheta = atan2(gazey,gazex)* 180 / PI;
            for(int i=0;i<classnames.size();i++)
            {
                if(noseobjecthetatmp.at(i)==10000)
                    gazethetadiff.at(i) = 10000;
                else if(noseobjecthetatmp.at(i)==0)
                    gazethetadiff.at(i) = 0;
                else
                {
                    gazethetadiff.at(i) = gazetheta-noseobjecthetatmp.at(i);
                }
            }
 
            auto mingazethetadiff = std::min_element(std::begin(gazethetadiff), std::end(gazethetadiff));
            float mingazethetadifftmp = *mingazethetadiff;
            std::vector<float>::iterator citernoseob =std::find(gazethetadiff.begin(), gazethetadiff.end(), mingazethetadifftmp);
            if (citernoseob != gazethetadiff.end()) 
            {
                nosethetaminindex = std::distance(gazethetadiff.begin(), citernoseob);
            }

            /*
            //commentout(11/21)
            std::cout<<"gazetheta:"<< gazetheta << std::endl;
            std::cout<<"noseobjecthetatmp :"<<" ";
            for(int i=0;i<noseobjecthetatmp.size();i++)
            std::cout << noseobjecthetatmp.at(i) << " ";
            std::cout <<""<< std::endl;
            std::cout<<"gazethetadiff :"<<" ";
            for(int i=0;i<gazethetadiff.size();i++)
            std::cout << gazethetadiff.at(i) << " ";
            std::cout <<""<< std::endl;
            std::cout << "mingazethetadifftmp:"<< mingazethetadifftmp << std::endl;
            std::cout << "nosethetaminindex:"<< nosethetaminindex << std::endl;
            
            noseobjecttheta<<darknet_cnt<<","
            <<currenttimesec<<","
            <<gazetheta<<","
            <<nosethetaminindex<<std::endl;
            std::cout<<""<< std::endl;
            */ 
        }
    }
    //object movement
    int maxmoveindex = 0;
    if((!classnames.empty())&&(!personbox.empty()))
    {
        std::cout<<"########basic object movement#############" << std::endl; 
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
            /*
            //commentout(11/21)
            std::cout << "object movement calculate" << std::endl;
            std::cout<<"lastboxcenterx:"<<" ";
            for(int i=0;i<lastboxcenterx.size();i++)
            std::cout << lastboxcenterx.at(i) << " ";
            std::cout <<""<< std::endl;
            std::cout<<"lastboxcentery:"<<" ";
            for(int i=0;i<lastboxcentery.size();i++)
            std::cout << lastboxcentery.at(i) << " ";
            std::cout <<""<< std::endl;
            */

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
                /*
                //commentout(11/21)
                std::cout<<"objectmovement:"<<" ";
                for(int i=0;i<objectmovement.size();i++)
                std::cout << objectmovement.at(i) << " ";
                std::cout <<""<< std::endl;
                */
                if(objectmovementtmp.empty())
                {
                    std::cout << "all movements are zero" << std::endl;
                    maxmoveindex = 0;
                } 
                else
                {
                    /*
                    //commentout(11/21)                    
                    std::cout<<"objectmovementtmp:"<<" ";
                    for(int i=0;i<objectmovementtmp.size();i++)
                    std::cout << objectmovementtmp.at(i) << " ";
                    std::cout <<""<< std::endl;
                    */
                    auto maxobjectmovement = std::max_element(std::begin(objectmovementtmp), std::end(objectmovementtmp));
                    //std::cout << "*maxobjectmovement: "<< *maxobjectmovement << std::endl;//commentout(11/21)
                    float maxobjectmovementtmp = *maxobjectmovement;
                    std::vector<float>::iterator citerobmove =std::find(objectmovement.begin(), objectmovement.end(), maxobjectmovementtmp);
                    if (citerobmove != objectmovement.end()) 
                    {
                        maxmoveindex = std::distance(objectmovement.begin(), citerobmove);
                    }
                    if(objectmovement.at(maxmoveindex)>10)
                    {
                        std::cout << "object moving" << std::endl;
                    }
                    else
                    {
                        std::cout << "object stopping" << std::endl;
                        maxmoveindex = 0;
                    }

                    for(int i=0;i<classnames.size();i++)
                    {
                        lastboxcenterx.at(i) = boxcenterx.at(i);
                        lastboxcentery.at(i) = boxcentery.at(i);
                    }        
                }
                activityscoreobject.at(maxmoveindex) += 1;   
            }   
            else
            {
                std::cout << "add new object movement " << std::endl;
                lastboxcenterx.clear();
                lastboxcentery.clear();
                lastboxcenterx.resize(boxcenterx.size());
                lastboxcentery.resize(boxcentery.size());
                for(int i=0;i<classnames.size();i++)
                {
                    lastboxcenterx.at(i) = boxcenterx.at(i);
                    lastboxcentery.at(i) = boxcentery.at(i);
                }
            }
            //std::cout << "maxmoveindex: "<< maxmoveindex << std::endl;//commentout(11/21)
        }
        
    }
    
    double deltatime = 0;
    if(!personbox.empty())
    {
        deltatime = currenttimesec-lastcurrenttimesec;
    }
    lastcurrenttimesec = currenttimesec;

    //if((!personbox.empty())&&(!estimateposition.empty()))
    if((!activityscoreface.empty())&&(!activityscoreobject.empty()))
    {
        std::cout << "###########record activity time############"<< std::endl;
        //std::cout<<"deltatime:"<<deltatime<<std::endl;//commentout(11/21)
        if(darknet_cnt>1)
        {
            timerecordface.at(noseendminindex) += deltatime;
        }
        /*
        //commentout(11/21)
        std::cout<<"timerecordface:"<<" ";
        for(int i=0;i<timerecordface.size();i++)
        std::cout << timerecordface.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"activityscoreface:"<<" ";
        for(int i=0;i<activityscoreface.size();i++)
        std::cout << activityscoreface.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"activityscoreobject:"<<" ";
        for(int i=0;i<activityscoreobject.size();i++)
        std::cout << activityscoreobject.at(i) << " ";
        std::cout <<""<< std::endl;    
        */

        activityscorefacedata<<darknet_cnt<<","
         <<currenttimesec<<", "
         <<noseendminindex<<std::endl;
        scorefacedata<<darknet_cnt<<","<<currenttimesec<<",";
        scorefacelabel<<darknet_cnt<<","<<currenttimesec<<",";
        activityscoreobjectdata<<darknet_cnt<<","
        <<currenttimesec<<", "
        <<maxmoveindex<<std::endl;
        scoreobjectdata<<darknet_cnt<<","<<currenttimesec<<",";
        scoreobjectlabel<<darknet_cnt<<","<<currenttimesec<<",";
        timerecordfacedata<<darknet_cnt<<","<<currenttimesec<<",";
        for(int i=0;i<classnames.size();i++)
        {    
                if(i==(classnames.size()-1))
                {
                    scorefacedata<< activityscoreface.at(i);
                    scoreobjectdata<< activityscoreobject.at(i);
                    scorefacelabel<<classnames.at(i);
                    scoreobjectlabel<<classnames.at(i);
                    timerecordfacedata<<timerecordface.at(i);
                }
                else
                {
                    scorefacedata<< activityscoreface.at(i)<<",";
                    scoreobjectdata<< activityscoreobject.at(i)<<",";
                    scorefacelabel<<classnames.at(i)<<",";
                    scoreobjectlabel<<classnames.at(i)<<",";
                    timerecordfacedata<<timerecordface.at(i)<<",";
                }
        }
        scorefacedata<<std::endl;
        scoreobjectdata<<std::endl;  
        scorefacelabel<<std::endl;
        scoreobjectlabel<<std::endl; 
        timerecordfacedata<<std::endl;  

         timeusedata<<darknet_cnt<<","
        <<currenttimesec<<","
        << personvelocity.at(0)<<","
        << personvelocity.at(1)<<","
        << estimateposition.at(0)<<","
        << estimateposition.at(1)<<std::endl;  
    }
    
    std::cout <<"darknet callback end"<< std::endl;
}

//Darknet Image
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
    if(display_num==2)
    {
        //two displays
        //cvMoveWindow("RGB darknet image", 2910,10);
        cvMoveWindow("RGB darknet image", 1000,0);
    }
    else
    {
        cvMoveWindow("RGB darknet image", 1000,0);    
    }
    cv::waitKey(10);
}

//RGB
void CombiDarknetOpenface::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    int i, j;
    int x1, x2, y1, y2;
    int width = WIDTH;
    int height = HEIGHT;
    cv_bridge::CvImagePtr cv_ptr;
    static int lastdarknetcnt = 0;

    //std::cout<<"RGB image_captute"<<std::endl;
    rgb_cnt += 1;
    std::cout<<"rgb_callback:"<<rgb_cnt<<std::endl;
    if (darknet_cnt!=lastdarknetcnt)
    {
        //std::cout<<"image saved"<<std::endl;
        std_msgs::Int16 capturecnt;
        //cv::Mat test = cv_bridge::toCvShare(msg, "mono8")->image;
        //cv::Mat test = cv_bridge::toCvShare(msg, "bgr8")->image;
        //cv::resize(test, test, cv::Size(), 0.15, 0.15);        
        //cv::imwrite(std::to_string(darknet_cnt)+".png", test);    
        capturecnt.data = darknet_cnt;
        capture_cnt_pub.publish(capturecnt);
        lastdarknetcnt = darknet_cnt;
    }
    
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    cv::Mat &mat = cv_ptr->image;
    cv::putText(cv_ptr->image, std::to_string(darknet_cnt), cv::Point(550,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA); 
    
    if(!nose_tip.empty())
    {
        cv::circle(cv_ptr->image, cv::Point(nose_tip[0], nose_tip[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(chin[0], chin[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_eye[0], left_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_eye[0], right_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_mouth[0], left_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_mouth[0], right_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);

    }
 
    if(!personbox.empty())
    {
        cv::putText(cv_ptr->image, "person found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }
    else
    {
        cv::putText(cv_ptr->image, "person not found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }

    //facescore
    //if((!nose_end_point2D_drawmin.empty())&&(!personbox.empty())&&(!activityscoreface.empty()))
    if((!nose_end_point2D_drawmin.empty())&&(!personbox.empty())&&(!activityscoreface.empty()))
    {
        int zerocntface = std::count(activityscoreface.begin(), activityscoreface.end(), 0);
        int maxindexface = 0;
        
        //if(!(zerocntface==activityscoreface.size()))
        if(activityscoreface.at(0)==0)
        {
            auto maxactivityscoreface = std::max_element(std::begin(activityscoreface), std::end(activityscoreface));
            float maxactivityscorefacetmp = *maxactivityscoreface;
            std::vector<int>::iterator citeracscoreface =std::find(activityscoreface.begin(), activityscoreface.end(), maxactivityscorefacetmp);
            if (citeracscoreface != activityscoreface.end()) 
            {
                maxindexface = std::distance(activityscoreface.begin(), citeracscoreface);
            }
            if(!((boxcenterx[maxindexface]==0)&&(boxcentery[maxindexface]==0)))
            {
                //box
                cv::rectangle(cv_ptr->image, cv::Point(boxxmin2.at(maxindexface), boxymin2.at(maxindexface)), cv::Point(boxxmax2.at(maxindexface), boxymax2.at(maxindexface)), cv::Scalar(255, 0, 0), 3, 4);
                cv::circle(cv_ptr->image, cv::Point(boxcenterx[maxindexface], boxcentery[maxindexface]), 5, cv::Scalar(255,0,0), 3);
                cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(boxcenterx[maxindexface],boxcentery[maxindexface]), cv::Scalar(255,0,0), 2);
            }
        }
        else
        {
            maxindexface = 0;
        }
        //std::cout << "maxindexface:"<< maxindexface << std::endl;
        cv::putText(cv_ptr->image, classnames.at(maxindexface), cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
        
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw[0], nose_end_point2D_draw[1]), 5, cv::Scalar(255, 255,0), 3);
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw2[0], nose_end_point2D_draw2[1]), 5, cv::Scalar(0,255,255), 3);
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw3[0], nose_end_point2D_draw3[1]), 5, cv::Scalar(255,255,255), 3);
 
        if(((nose_end_point2D_draw3[0]>0)&&(nose_end_point2D_draw3[0]<640))&&((nose_end_point2D_draw3[1]>0)&&(nose_end_point2D_draw3[1]<480)))
        {
            cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw3[0],nose_end_point2D_draw3[1]), cv::Scalar(255,255,225), 2);
        }
        else if(((nose_end_point2D_draw2[0]>0)&&(nose_end_point2D_draw2[0]<640))&&((nose_end_point2D_draw2[1]>0)&&(nose_end_point2D_draw2[1]<480)))
        {    
            cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw2[0],nose_end_point2D_draw2[1]), cv::Scalar(0,255,225), 2);
        }
        else if(((nose_end_point2D_draw[0]>0)&&(nose_end_point2D_draw[0]<640))&&((nose_end_point2D_draw[1]>0)&&(nose_end_point2D_draw[1]<480)))
        {
            cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw[0],nose_end_point2D_draw[1]), cv::Scalar(255,255,0), 2);
        }
        
        //nose_end_point2D_drawmin
        cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_drawmin[0], nose_end_point2D_drawmin[1]), 5, cv::Scalar(255,0,255), 5);
      
        //nose_end_point2D_drawtmp
        //cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_drawtmp[0], nose_end_point2D_drawtmp[1]), 5, cv::Scalar(255,0,0), 3);  
    }

    /*
    //objectscore
    if(((!classnames.empty())&&(!personbox.empty())&&(!activityscoreobject.empty())))
    {
        int zerocntobject = std::count(activityscoreobject.begin(), activityscoreobject.end(), 0);
        
        int maxindexobject = 0;
        //if(!(zerocntobject==activityscoreobject.size()))
        if(activityscoreobject.at(0)==0)
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
    if(display_num==2)
    {
        //two displays
        //cvMoveWindow("RGB image",2910,350);
        cvMoveWindow("RGB image", 990,350);
    }
    else
    {
        //one display
        cvMoveWindow("RGB image", 990,350);
        
    }
    cv::waitKey(10);
    
    std::cout << "rgb image finished" << std::endl;
    std::cout<<""<<std::endl;
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

    int pointx1 = 0;
    int pointx2 = 0;
    int pointy1 = 0;
    int pointy2 = 0;
    double depthpoint = 0.0;

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

    depth_cnt += 1;
    std::cout<<"depth_callback:"<<depth_cnt<<std::endl;

    if(!personbox.empty())
    {
        
        x1ori = personbox.at(0);
        y1ori = personbox.at(1);
        x2ori = personbox.at(2);
        y2ori = personbox.at(3);

        xc = (x1ori+x2ori)/2;
        yc = (y1ori+y2ori)/2;

        /*
        x1 = xc-1;
        y1 = yc-1;
        x2 = xc+1
        y2 = yc+1;
        */

        x1 = x1ori;
        y1 = y1ori;
        x2 = x2ori;
        y2 = y2ori;
        
        if(!nose_tip.empty())
        {
            pointx1 = nose_tip[0]-1;
            pointy1 = nose_tip[1]-1;
            pointx2 = nose_tip[0]+1;
            pointy2 = nose_tip[1]+1;
            std::cout<<"point_nose:"<<nose_tip[0]<<", "<<nose_tip[1]<<std::endl; 
        }
        else
        {
            pointx1 = xc-1;
            pointy1 = yc-1;
            pointx2 = xc+1;
            pointy2 = yc+1;
            std::cout<<"point_center:"<<xc<<", "<<yc<<std::endl;
        }        

        std::vector<double>boxdepthivalue;

        int depthsumcnt = 0;
        for(int i = 0; i < cv_ptr->image.rows;i++)
        {
            float* Dimage = cv_ptr->image.ptr<float>(i);
            float* Iimage = depth.ptr<float>(i);
            char* Ivimage = img.ptr<char>(i);
            for(int j = 0 ; j < cv_ptr->image.cols; j++)
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
                            boxdepthivalue.push_back(Dimage[j]);
                            depthsumcnt +=1;
                            //std::cout<<"depthsumcnt:"<<depthsumcnt<<std::endl;
                        }
                    }
                }
                if(i > pointy1 && i < pointy2)
                {
                    if(j > pointx1 && j < pointx2)
                    {
                        if(Dimage[j] > 0.0)
                        {
                            depthpoint = Dimage[j];
                        }
                    }
                }
            }
        } 

        //distance = sum / ((x2-x1)*(y2-y1));
        distance = depthpoint;

        theta = AngleofView/2-((double)xc/640)*AngleofView;
        distancetmp = distance;
        CombiDarknetOpenface::ModifyPersonDistance(&distance);
        std::cout<<"distance:"<<distance<<std::endl;
        /*
        robotpose.clear();
        robotpose.push_back(-2.84785);
        robotpose.push_back(-2.64466);
        robotyaw = -13.1795;
        //robotyaw = 0;

        visualization_msgs::Marker robotposearrow;
        
        robotposearrow.header.frame_id = fixed_frame;
        robotposearrow.header.stamp = ros::Time::now();
        robotposearrow.ns = "basic_shapes";
        robotposearrow.type = visualization_msgs::Marker::ARROW;
        robotposearrow.action = visualization_msgs::Marker::ADD;
        robotposearrow.pose.position.x = robotpose.at(0);
        robotposearrow.pose.position.y = robotpose.at(1);
        robotposearrow.pose.position.z = 0;
        robotposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(robotyaw/180.0*M_PI);
        robotposearrow.scale.x = 0.3;
        robotposearrow.scale.y = 0.1;
        robotposearrow.scale.z = 0.1;
        // Set the color -- be sure to set alpha to something non-zero!
        robotposearrow.color.r = 0.0f;
        robotposearrow.color.g = 1.0f;
        robotposearrow.color.b = 0.0f;
        robotposearrow.color.a = 1.0f;
    
        robotposearrow.lifetime = ros::Duration();
        robotpose_arrow_pub.publish(robotposearrow);
        robotpose_cnt = 2;
        */

        if((fixed_frame =="map")&&(robotpose_cnt>0))
        {
            theta += robotyaw;
            double measurementx = distance * cos(theta/180.0*M_PI)+robotpose.at(0);
            double measurementy = distance * sin(theta/180.0*M_PI)+robotpose.at(1);
            //double measurementy = distance * sin(theta/180.0*M_PI)+robotpose.at(1);
            
            CombiDarknetOpenface::PublishPersonMeasurement(measurementx,measurementy);
            CombiDarknetOpenface::PublishPersonMarker(theta,measurementx,measurementy); 
            //std::cout<<"robotyaw:"<<robotyaw<<std::endl;//commentout(11/21)
            //std::cout<<"theta:"<<theta<<std::endl;//commentout(11/21)     
        }
        else if(fixed_frame =="base_link")
        {
            double measurementx = distance * cos(theta/180.0*M_PI);
            double measurementy = distance * sin(theta/180.0*M_PI);
            CombiDarknetOpenface::PublishPersonMeasurement(measurementx,measurementy);
            CombiDarknetOpenface::PublishPersonMarker(theta,measurementx,measurementy);  
        }
        
        cv::rectangle(img, cv::Point(x1ori, y1ori), cv::Point(x2ori, y2ori), cv::Scalar(0, 0, 255), 5, 4);
        if(!nose_tip.empty())
        {
            cv::circle(img, cv::Point(nose_tip[0],nose_tip[1]), 8, cv::Scalar(0, 0, 0), 3);
        }
        else
        {
            cv::circle(img, cv::Point(xc, yc), 8, cv::Scalar(0, 0, 0), 3);
        }  
        //cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 3, 4);
    }
    else
    {
        for(int i = 0; i < cv_ptr->image.rows;i++)
        {
            float* Dimage = cv_ptr->image.ptr<float>(i);
            float* Iimage = depth.ptr<float>(i);
            char* Ivimage = img.ptr<char>(i);
            for(int j = 0 ; j < cv_ptr->image.cols; j++)
            {
                if(Dimage[j] > 0.0)
                {
                    Iimage[j] = Dimage[j];
                    Ivimage[j] = (char)(255*(Dimage[j]/5.5));
                }
            }
        } 
    }

    cv::resize(img, img, cv::Size(), ResizeSize, ResizeSize);
    cv::imshow("Depth_Image", img);
    
    if(display_num==2)
    {
        //cvMoveWindow("Depth_Image", 3250,10);
        cvMoveWindow("Depth_Image", 1450,0);
    }
    else
    {
        //one display
        cvMoveWindow("Depth_Image", 1450,0);
        //cvMoveWindow("Depth_Image", 10,0);
    }
    
    cv::waitKey(10);
    
     depthdata<<darknet_cnt<<","
     <<currenttimesec<<", "
     << distancetmp<<", "
     << distance<<", "
     << xc<<", "
     << yc<<", "
     <<modify_distance_cnt<<","
     <<personmove_cnt<<","
     << personmove<<std::endl;  

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
        personmove = 2; 
    }
    else if((abs(personvelocity.at(0))>0.04)&&(abs(personvelocity.at(1))>0.04))
    {
        personmove_cnt += 1;
    }
    else if ((abs(personvelocity.at(0))>0.06)||(abs(personvelocity.at(1))>0.06))
    {
        personmove_cnt += 1;   
    }
    else
    {
        //person is stopping
        personmove = 0;
        personmove_cnt = 0;
    }

    if(personmove_cnt>4)
    {
        //person is moving
        personmove=1;
    }
    
    lastcurrenttimevelocity  = currenttimesec;

    std::cout<<"personmove_cnt:"<<personmove_cnt<<std::endl;
    std::cout<<"personmove:"<<personmove<<std::endl;
    
    //std::cout<<"estimateposition:"<<estimateposition[0]<<","<<estimateposition[1]<<std::endl;
    //std::cout<<"lastestimateposition:"<<lastestimateposition[0]<<","<<lastestimateposition[1]<<std::endl;
     //std::cout<<"personvelocity:"<<personvelocity[0]<<","<<personvelocity[1]<<std::endl;
    personvelocitydata<<darknet_cnt<<","
    <<currenttimesec<<", "
    << estimateposition.at(0)<<", "
    << estimateposition.at(1)<<", "
    << personvelocity.at(0)<<", "
    << personvelocity.at(1)<<", "
    << personmove_cnt<<", "
    <<personmove<<std::endl;

    visualization_msgs::Marker estimatepersonmarker;
        
    estimatepersonmarker.header.stamp = ros::Time::now();
    estimatepersonmarker.header.frame_id = fixed_frame;
    estimatepersonmarker.pose.position.x = estimateposition.at(0);
    estimatepersonmarker.pose.position.y = estimateposition.at(1);
    estimatepersonmarker.lifetime = ros::Duration(0);
    estimatepersonmarker.scale.x = 0.2;
    estimatepersonmarker.scale.y = 0.2;
    estimatepersonmarker.scale.z = 0.01;
    estimatepersonmarker.type = estimatepersonmarker.SPHERE;
    estimatepersonmarker.color.a = 0.75;
    estimatepersonmarker.color.r = 1.0;
    estimatepersonmarker.color.g = 0;
    estimatepersonmarker.color.b = 1.0;
    estimate_marker_pub.publish(estimatepersonmarker);

    visualization_msgs::Marker m2;
    m2.header.stamp = ros::Time::now();
    m2.header.frame_id = fixed_frame;
    m2.ns = "Person";
    m2.type = m2.TEXT_VIEW_FACING;
    m2.pose.position.x = estimateposition.at(0)+0.15;
    m2.pose.position.y = estimateposition.at(1);
    m2.text =  tostr(darknet_cnt);
    m2.scale.x = .1;
    m2.scale.y = .1;
    m2.scale.z = 0.2;
    m2.color.a = 1;
    m2.lifetime = ros::Duration();
    m2.color.r = 1.0;
    cnt_text_pub.publish(m2);
        
    std::cout<<""<<std::endl;
        
}

void CombiDarknetOpenface::msgCallback_RobotPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
        static ros::Time firsttime = ros::Time::now();
        ros::Time nowtime = ros::Time::now();

        double firsttimesec,nowtimesec,currenttimesec;
        firsttimesec = firsttime.toSec();
        nowtimesec = nowtime.toSec();
        currenttimesec = nowtimesec-firsttimesec;

        robotpose_cnt += 1;
        std::cout<<"robotpose_callback:"<<robotpose_cnt<<std::endl;
        
        if(robotpose_cnt>1)
        {
            lastrobotpose.push_back(robotpose[0]);
            lastrobotpose.push_back(robotpose[1]);
        }
        else
        {
            lastrobotpose.push_back(msg->pose.position.x);
            lastrobotpose.push_back(msg->pose.position.y);
        }
        robotpose.clear();
        robotpose.push_back(msg->pose.position.x);
        robotpose.push_back(msg->pose.position.y);
        tf::Quaternion q(msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        robotyaw = yaw/M_PI*180;//deg
        if(robotpose_cnt>1)
        {
            robotvelocity.push_back(robotpose.at(0) - lastrobotpose.at(0));
            robotvelocity.push_back(robotpose.at(1) - lastrobotpose.at(1));
        }
        else
        {
            robotvelocity.push_back(0.0);
            robotvelocity.push_back(0.0);
        }

        if((abs(robotvelocity.at(0))>0.01)&&(abs(robotvelocity.at(1))>0.01))
        {
            robotmove_cnt += 1;
        }
        else if ((abs(robotvelocity.at(0))>0.02)||(abs(robotvelocity.at(1))>0.02))
        {
            robotmove_cnt += 1;   
        }
        else
        {
            robotmove = 0;
            robotmove_cnt = 0;
        }
        
        if(robotmove_cnt>4)
        {
            robotmove=1;
        }
        
        lastcurrenttimevelocity  = currenttimesec;

        std::cout<<"robot_cnt:"<<robotmove_cnt<<std::endl;
        std::cout<<"robotmove:"<<robotmove<<std::endl; 
        std::cout<<"robotpose:"<<robotpose.at(0)<<","<<robotpose.at(1)<<std::endl;
        std::cout<<"robotyaw:"<<robotyaw<<std::endl;
        //std::cout<<"lastrobotpose:"<<lasterobotpose[0]<<","<<lastestimateposition[1]<<std::endl;
         //std::cout<<"robotvelocity:"<<robotvelocity[0]<<","<<robotvelocity[1]<<std::endl;
        
        visualization_msgs::Marker robotposearrow;
         
        robotposearrow.header.frame_id = fixed_frame;
        robotposearrow.header.stamp = ros::Time::now();
        robotposearrow.ns = "basic_shapes";
        robotposearrow.type = visualization_msgs::Marker::ARROW;
        robotposearrow.action = visualization_msgs::Marker::ADD;
         
         //std::cout<<"personarrow.pose.orientation:"<<theta<<std::endl;
        //robotarrow.pose.orientation=tf::createQuaternionMsgFromYaw(theta/180.0*M_PI);
        robotposearrow.pose.position.x = msg->pose.position.x;
        robotposearrow.pose.position.y = msg->pose.position.y;
        robotposearrow.pose.position.z = msg->pose.position.z;
        robotposearrow.pose.orientation.x = msg->pose.orientation.x;
        robotposearrow.pose.orientation.y = msg->pose.orientation.y;
        robotposearrow.pose.orientation.z = msg->pose.orientation.z;
        robotposearrow.pose.orientation.w = msg->pose.orientation.w;
        
         // Set the scale of the marker -- 1x1x1 here means 1m on a side
        robotposearrow.scale.x = 0.3;
        robotposearrow.scale.y = 0.1;
        robotposearrow.scale.z = 0.1;
         // Set the color -- be sure to set alpha to something non-zero!
        robotposearrow.color.r = 0.0f;
        robotposearrow.color.g = 1.0f;
        robotposearrow.color.b = 0.0f;
        robotposearrow.color.a = 1.0f;
     
        robotposearrow.lifetime = ros::Duration();
        robotpose_arrow_pub.publish(robotposearrow);

        robotvelocitydata<<darknet_cnt<<","
        <<currenttimesec<<", "
        << robotpose.at(0)<<", "
        << robotpose.at(1)<<", "
        << robotvelocity.at(0)<<", "
        << robotvelocity.at(1)<<", "
        << robotmove_cnt<<", "
        <<robotmove<<std::endl;
        
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
    
    if((!estimateposition.empty())&&(!headorientation.empty()))
    {
        headposearrow.header.frame_id = fixed_frame;
        headposearrow.header.stamp = ros::Time::now();
        headposearrow.ns = "basic_shapes";
        headposearrow.type = visualization_msgs::Marker::ARROW;
        headposearrow.action = visualization_msgs::Marker::ADD;
   
        headposearrow.pose.position.x = estimateposition[0];
        headposearrow.pose.position.y = estimateposition[1];
        //std::cout<<"headposearrow.pose:"<<headposearrow.pose.position.x<<","<<headposearrow.pose.position.y<<std::endl;
                
        if((0<=headorientation[2])&&(headorientation[2]<90))
        {
            headarrowtheta = 180-headorientation[2];
            //std::cout<<"headposearrow.orientation:"<<headorientation[2]<<", "<<arrowtheta<<std::endl;    
      
        }
        else if((-90<=headorientation[2])&&(headorientation[2]<0))
        {
            headarrowtheta = std::abs(headorientation[2])+180;
            //std::cout<<"headposearrow.orientation:"<<headorientation[2]<<", "<<arrowtheta<<std::endl;       
        }
        else
        {
            headarrowtheta = headorientation[2];
            //std::cout<<"out of yaw range :"<<std::endl;
        }
        
        //headposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(-180/180.0*M_PI);

        headposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(headarrowtheta/180.0*M_PI);
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

        headposearrow.lifetime = ros::Duration();
        headpose_arrow_pub.publish(headposearrow);
       
    }
    
}

void CombiDarknetOpenface::PublishPersonMarker(double theta, double measurementx, double measurementy)
{
    visualization_msgs::Marker personarrow;
    
    personarrow.header.frame_id = fixed_frame;
    personarrow.header.stamp = ros::Time::now();
    personarrow.ns = "basic_shapes";
    personarrow.type = visualization_msgs::Marker::ARROW;
    personarrow.action = visualization_msgs::Marker::ADD;
    
    //std::cout<<"personarrow.pose.orientation:"<<theta<<std::endl;
    personarrow.pose.position.x = measurementx;
    personarrow.pose.position.y = measurementy;
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
    personmarker.header.frame_id = fixed_frame;
    personmarker.pose.position.x = measurementx;
    personmarker.pose.position.y = measurementy;
    personmarker.lifetime = ros::Duration();
    personmarker.scale.x = 0.2;
    personmarker.scale.y = 0.2;
    personmarker.scale.z = 0.01;
    personmarker.type = personmarker.CUBE;
    personmarker.color.a = 0.75;
    personmarker.color.g =  1.0;
    person_marker_pub.publish(personmarker);

    visualization_msgs::Marker originmarker;

    originmarker.header.stamp = ros::Time::now();
    originmarker.header.frame_id = fixed_frame;
    originmarker.pose.position.x = 0.0;
    originmarker.pose.position.y = 0.0;
    originmarker.lifetime = ros::Duration();
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

    timerecordfacedata<<"cnt"<<","
    <<"time"<<","
    <<"activity time"<<std::endl;

    scorefacelabel<<"cnt"<<","
    <<"time"<<","
    <<"label"<<std::endl;

    scoreobjectlabel<<"cnt"<<","
    <<"time"<<","
    <<"label"<<std::endl;

    alltimerecord<<"darknet_cnt"<<","
    <<"time"<<","
    <<"robotmove_cnt"<<","
    <<"robotmove"<<","
    <<"personmove_cnt"<<","
    <<"personmove"<<","    
    <<"notmeasuremnt_cnt"<<std::endl;

    personvelocitydata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"personpose_x"<<","
    <<"personpose_y"<<","
    <<"personvelocity_x"<<","
    <<"personvelocity_y"<<","
    <<"personmove_cnt"<<","
    <<"personmove"<<std::endl;

    robotvelocitydata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"robotpose_x"<<","
    <<"robotpose_y"<<","
    <<"robotvelocity_x"<<","
    <<"robotvelocity_y"<<","
    <<"robotmove_cnt"<<","
    <<"robotmove"<<std::endl;

    robotcmdveldata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"msg.linear.x"<<","
    <<"msg.linear.y"<<","
    <<"msg.angular.z"<<","
    <<"robotmove"<<std::endl;

    depthdata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"distancetmp"<<","
    <<"distance"<<","
    <<"xc"<<","
    <<"yc"<<","
    <<"modify_distance_cnt"<<","
    <<"personmove_cnt"<<","
    <<"personmove"<<std::endl;

    noseobjecttheta<<"darknet_cnt"<<","
    <<"time"<<","
    <<"gazetheta"<<","
    <<"nosethetaminindex"<<std::endl;

    noseobjecttheta<<"darknet_cnt"<<","
    <<"currenttimesec"<<","
    <<"gazetheta"<<","
    <<"nosethetaminindex"<<std::endl;
    
    ros::NodeHandle nh;
    CombiDarknetOpenface pt(nh);
    
    ros::spin();
    return 0;
}