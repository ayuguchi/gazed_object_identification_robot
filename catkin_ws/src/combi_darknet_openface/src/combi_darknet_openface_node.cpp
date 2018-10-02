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

#define TmpDistance 250.0
#define FirstDistance 350.0
#define SecondDistance 500.0
#define ThirdDistance 650.0
#define DistanceStep 10
#define DistanceRange 650

/*
#define TmpDistance 250.0
#define FirstDistance 250.0
#define SecondDistance 500.0
#define ThirdDistance 750.0
#define DistanceStep 10
#define DistanceRange 750
*/

#define AngleofView 58

#define ResizeSize 0.5

#define OrientationInView 1
#define OrientationOutView 2
#define OrientationFront 3
#define RobotPoseReset 4
#define ObjectFar 5
#define FaceOrinentationError 6

#define RobotMoveCount 10000
#define PoseResetCount 50
#define RobotStopCount 300

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

int after_flag = 0;

int area_error = 0;
double errortmptheta;
double errortmpx;
double errortmpy;
double angularsig;
double linearxsig;
double linearysig;
double targetthetatmp;

int robot_move = 0;//0:stoping,1:moving:
int robot_move_cnt = 0;
int person_move = 1;//0:stoping,1:moving
int person_move_cnt = 0;

int robot_moving = 0;
int moving_cnt = 0;
int pose_reset = 0;
int pose_reset_cnt = 0;
int out_view_mode = 0;

double test_headangle = 0;

int display_num = 1;

int notmeasurement_cnt=0;
int object_far = 0; 
int move_mode = 0;
int lastmove_mode = 0;

double lastcurrenttimevelocity  = 0;

static std::string fixed_frame = "map";

std::ofstream headposedata("headposedata.csv", std::ios::trunc);
std::ofstream facefeatures("facefeatures.csv", std::ios::trunc);
std::ofstream timeusedata("timeusedata.csv", std::ios::trunc);
std::ofstream activityscorefacedata("activityscorefacedata.csv", std::ios::trunc);
std::ofstream activityscorefacedata2("activityscorefacedata2.csv", std::ios::trunc);
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
std::ofstream robot_movingdata("robot_movingdata.csv", std::ios::trunc);

class CombiDarknetOpenface
{
public:
    CombiDarknetOpenface(ros::NodeHandle nh);
    ~CombiDarknetOpenface();
    void msgCallback_FaceRecognition(const combi_darknet_openface::Faces::ConstPtr& msg );
    void ModifyHeadOrientation();
    void ModifyPersonDistance(double* distance);
    void ModifyObjectDistance(double* distance);
    void Calculate_TimeUse(double currenttimesec);
    void Linear_Line(double x1,double x2,double y1,double y2,double* a,double* b );
    double Round( double dSrc, int iLen );
    void Calculate_TimeUseOutofView(double currenttimesec);
    void msgCallback_ObjectRecognition(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void rgbObjectImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);//face_feature
    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void msgCallback_FilterMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void msgCallback_RobotPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void PublishPersonMeasurement(double measurementx, double measurementy);//input for KF
    void PublishPersonMarker(double theta, double measurementx, double measurementy);//input for KF
    void PublishObjectMarker(double measurementx, double measurementy);
    void PublishHeadposeArrow();
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
    ros::Publisher object_marker_pub;
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
    
    int maxmoveindex;
    int noseendminindex;
    int mindistanceindex;
    std::vector<int>activityscoreface;
    std::vector<int>activityscoreobject;
    std::vector<float>timerecordface;

    //depth
    double noseobjectmindist;
    double persondepthdist;
    
    std::vector<double>estimateposition;
    std::vector<double>lastestimateposition;
    std::vector<double>personvelocity;

    std::vector<double>robotpose;
    double robotyaw;
    double robotyawraw;
    geometry_msgs::PoseStamped robotoriginpose;
    std::vector<double>lastrobotpose;
    std::vector<double>robotvelocity;

    std::vector<int>personbox; 
    std::vector<int>detectedobjectbox; 
    
    std::vector<double>headorientation;    
    std::vector<int>nose_tip;
    std::vector<int>chin;
    std::vector<int>left_eye;
    std::vector<int>right_eye;
    std::vector<int>left_mouth;
    std::vector<int>right_mouth;
    double headarrowtheta;

    double headarrowangleraw;
    double headarrowangle;
    
    double headrobottheta;

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

    std_msgs::String speechtxt;
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
        object_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_object_marker", 1);

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
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,TmpDistance));
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
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,TmpDistance));
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
            
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,FirstDistance));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
            nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw.push_back(std::round(nose_end_point2D[0].y));
            
            nose_end_point2D.clear();
            nose_end_point3D.clear();        
            nose_end_point3D.push_back(cv::Point3f(0.0,0.0,SecondDistance));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
                
            nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].x));
            nose_end_point2D_draw2.push_back(std::round(nose_end_point2D[0].y));
        
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3f(0,0,ThirdDistance));
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
    }
    */

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
    maxmoveindex = 0;
    noseendminindex = 0;

    personbox.clear();
        
    int objectcnt = 0;
    int personindex = 0; 

    darknet_cnt += 1;
    std::cout<<"darknet_callback:"<<darknet_cnt<<std::endl;

    if(darknet_cnt==1)
    {
        speechtxt.data = "start";
        pepper_speech_pub.publish(speechtxt);
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
        else if(classnames.at(i)=="bench")
        {
            boxcenterx.at(i) = 0;
            boxcentery.at(i) = 0;      
        }
        else if(classnames.at(i)=="oven")
        {
            boxcenterx.at(i) = 0;
            boxcentery.at(i) = 0;      
        }
        else if(classnames.at(i)=="refrigerator")
        {
            boxcenterx.at(i) = 0;
            boxcentery.at(i) = 0;      
        }
    }


    for(int i=0;i<classnames.size();i++)
    {        
        if(classnames.at(i)=="bottle")
        {
            //double detectedobjectxc = (detectedobjectbox.at(0)+detectedobjectbox.at(2))/2;
            double xcerror =  boxcenterx.at(i)-320;
            //std::cout << "detectedobjectxc:"<< detectedobjectxc << std::endl;
            std::cout << "xcerror:"<< xcerror << std::endl;
        }
    } 

    //if((!personbox.empty())&&person_move)
    //if((!personbox.empty()))
    std::cout<<"robot_move:"<<robot_move<<std::endl;
    std::cout<<"person_move:"<<person_move<<std::endl;
    std::cout<<"notmeasurement_cnt:"<<notmeasurement_cnt<<std::endl;
    std::cout<<"robot_moving:"<<robot_moving<<std::endl;
    std::cout<<"pose_reset:"<<pose_reset<<std::endl;
    std::cout<<"pose_reset_cnt:"<<pose_reset_cnt<<std::endl;
    std::cout<<"headarrowtheta:"<<headarrowtheta<<std::endl;
    std::cout<<"robotyaw:"<<robotyaw<<std::endl;
    
    /*
    if((robot_moving==0)&&(notmeasurement_cnt==1))
    {
        exit(1);
    }
    */
    double headarrowthetatmp = headarrowtheta+180;
    if(headarrowthetatmp>360)
    {
        headarrowthetatmp -= 360;
    }
    headrobottheta = robotyaw-headarrowthetatmp;
    std::cout<<"headrobottheta:"<<headrobottheta<<std::endl;

    /*
    robotpose.clear();
    robotpose.push_back(-1);
    robotpose.push_back(-1);
    robotyaw = 167/M_PI*180;//deg
    person_move = 0;
    robot_move = 0;
    */
    
    //if((!personbox.empty())&&(!robot_move)&&(!person_move))
    if(((!personbox.empty())&&(!robot_move)&&(!person_move))&&(notmeasurement_cnt<RobotMoveCount)&&(!robot_moving)&&(!pose_reset))
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
    else if(pose_reset)
    {
        std::cout << "###########pose reset############"<< std::endl;
        if(pose_reset_cnt==PoseResetCount)
        {
            std::cout << "###########pose move############"<< std::endl;
            move_mode = RobotPoseReset;
            after_flag = 1;
            CombiDarknetOpenface::ChangeViewPoint(currenttimesec);
        }
        else
        {
            pose_reset_cnt += 1;
            //after_flag = 0;            
            after_flag = 1;            
            
            std::cout << "###########measure timeuse out of view############"<< std::endl;
            CombiDarknetOpenface::Calculate_TimeUseOutofView(currenttimesec);    
        }
    }
    else if(person_move)
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

    static double lastcurrenttimesec;
    double deltatime = 0;
    if(darknet_cnt ==1)
    {
        lastcurrenttimesec = currenttimesec;
    }
    
    deltatime = currenttimesec-lastcurrenttimesec;
    
    lastcurrenttimesec = currenttimesec;
    
    if((!activityscoreface.empty())&&(!activityscoreobject.empty()))
    {
        std::cout << "###########record activity time############"<< std::endl;
        
        if(darknet_cnt>1)
        {
            timerecordface.at(noseendminindex) += deltatime;
        }
        /*
        //commentout(11/21)
        std::cout<<"deltatime:"<<deltatime<<std::endl;//commentout(11/21)
        std::cout<<"timerecordface:"<<" ";
        for(int i=0;i<timerecordface.size();i++)
        std::cout << timerecordface.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"activityscoreface  :"<<" ";
        for(int i=0;i<activityscoreface.size();i++)
        std::cout << activityscoreface.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"activityscoreobject:"<<" ";
        for(int i=0;i<activityscoreobject.size();i++)
        std::cout << activityscoreobject.at(i) << " ";
        std::cout <<""<< std::endl;    
        */

        std::cout << "noseendminindex:"<< noseendminindex << std::endl;
        activityscorefacedata<<darknet_cnt<<","
         <<currenttimesec<<", "
         <<noseendminindex<<std::endl;
        scorefacedata<<darknet_cnt<<","<<currenttimesec<<",";
        scorefacelabel<<darknet_cnt<<","<<currenttimesec<<",";
        activityscorefacedata2<<darknet_cnt<<","
        <<currenttimesec<<","
        <<notmeasurement_cnt<<","
        <<move_mode<<","
        <<robot_moving<<std::endl;
        
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

        if(kf_cnt>1)
        {
            timeusedata<<darknet_cnt<<","
            <<currenttimesec<<","
            << personvelocity.at(0)<<","
            << personvelocity.at(1)<<","
            << estimateposition.at(0)<<","
            << estimateposition.at(1)<<std::endl;      
        }
    }

    alltimerecord<<darknet_cnt<<","
    <<currenttimesec<<","
    <<robot_move_cnt<<","
    <<robot_move<<","
    <<person_move_cnt<<","
    <<person_move<<","
    <<notmeasurement_cnt<<","
    <<pose_reset_cnt<<","
    <<pose_reset<<","
    <<move_mode<<","
    <<robot_moving<<","
    <<moving_cnt<<std::endl;    
    
    std::cout <<"darknet_callback end"<< std::endl;
    std::cout <<""<< std::endl;
    
    frame_num += 1;
}

void CombiDarknetOpenface::Linear_Line(double x1,double y1,double x2,double y2,double* a,double* b )
{
    *a = (y2-y1)/(x2-x1);
    double a2 = *a;
    *b = y1-a2*x1;
}

double CombiDarknetOpenface::Round( double dSrc, int iLen )
{
	double	dRet;

	dRet = dSrc * pow(10.0, iLen);
	dRet = (double)(int)(dRet + 0.5);

	return dRet * pow(10.0, -iLen);
}
void CombiDarknetOpenface::ChangeViewPoint(double currenttimesec)
{
    static geometry_msgs::Twist twist;
    double disterror = 0.0;
    double distxerror = 0.0;
    double distyerror = 0.0;
    double angleerror = 0.0;
    static std::vector<double>targetrobotpose;
    static std::vector<double>beforerobotpose;
    static int exitflag = 0;
    
    after_flag = 1;
    if(targetrobotpose.empty()&&(!robotpose.empty()))
    {
        std::cout<<"init targetrobotpose"<<std::endl;
        for(int i=0;i<robotpose.size();i++)
        {        
            targetrobotpose.push_back(robotpose.at(i));
        }
        targetrobotpose.push_back(robotyaw);
    }

    if(beforerobotpose.empty())
    {
        std::cout<<"init beforerobotpose"<<std::endl;
        for(int i=0;i<robotpose.size();i++)
        {        
            beforerobotpose.push_back(robotpose.at(i));
        }
        beforerobotpose.push_back(robotyaw);
    }
    if((!robot_moving)&&(!pose_reset))
    {
        beforerobotpose.at(0) = robotpose.at(0);
        beforerobotpose.at(1) = robotpose.at(1);
        beforerobotpose.at(2) = robotyaw;
        //speechtxt.data = "saved current position";
        //pepper_speech_pub.publish(speechtxt);
    }
    if(!robot_moving)
    {
        //speechtxt.data = "robot moving start";
        //pepper_speech_pub.publish(speechtxt);
        moving_cnt = 0;
    }
    else
    {
        moving_cnt += 1;
    }
    
    double robotyawtmp = robotyaw;
    
    /*
    if(darknet_cnt==100)
    {
        robotyawtmp = robotyaw;
        robotyaw-=30;
    }
    */
    
    if((!robotpose.empty())&&(!nose_end_point2D_drawtmp.empty()&&(!pose_reset)))
    {
        if(move_mode==OrientationInView)
        {
            std::cout<<"move_mode==OrientationInView"<<std::endl;
            if(robot_moving==0)
            {
                double deltatheta = 0;
                targetrobotpose.at(2) = headarrowtheta+180;
                if(targetrobotpose.at(2)>360)
                {
                    targetrobotpose.at(2)-=360;
                }
                
                /*
                std::cout<<"deltatheta:"<<deltatheta<<std::endl;
                std::cout<<"robot pose:"<<robotpose.at(0)<<","<<robotpose.at(1)<<std::endl;
                std::cout<<"person pose:"<<estimateposition.at(0)<<","<<estimateposition.at(1)<<std::endl;
                std::cout<<"target robotpose:"<<targetrobotpose.at(0)<<","<<targetrobotpose.at(1)<<std::endl;
                */
                double r = std::sqrt(std::pow(robotpose.at(0)-estimateposition.at(0), 2) + std::pow(robotpose.at(1)-estimateposition.at(1), 2));
                
                targetrobotpose.at(0) = r*cos(headarrowtheta/180.0*M_PI)+estimateposition.at(0);
                targetrobotpose.at(1) = r*sin(headarrowtheta/180.0*M_PI)+estimateposition.at(1);
                
                double r2 = std::sqrt(std::pow(targetrobotpose.at(0)-estimateposition.at(0), 2) + std::pow(targetrobotpose.at(1)-estimateposition.at(1), 2));
                
                robot_moving = 1;
                std::cout<<"r:"<<r<<std::endl;
                std::cout<<"r2:"<<r2<<std::endl;       
                std::cout<<"robot pose:"<<robotpose.at(0)<<","<<robotpose.at(1)<<std::endl;
                std::cout<<"person pose:"<<estimateposition.at(0)<<","<<estimateposition.at(1)<<std::endl;
                std::cout<<"headarrowtheta:"<<headarrowtheta<<std::endl;
            }    
        }
        else if(move_mode==OrientationOutView)
        {
            std::cout<<"move_mode==OrientationOutView"<<std::endl;
            pose_reset=1;

            if(robot_moving==0)
            {
                targetrobotpose.at(0) = robotpose.at(0);
                targetrobotpose.at(1) = robotpose.at(1);

                if((0<=headrobottheta)&&(headrobottheta<10))
                {
                    std::cout<<"out of view mode:1"<<std::endl;   
                    targetrobotpose.at(2) = robotyaw;
                    out_view_mode=1;
                }
                else if((10<=headrobottheta)&&(headrobottheta<=30))
                {
                    std::cout<<"out of view mode:2"<<std::endl;   
                    targetrobotpose.at(2) = headarrowtheta;
                    out_view_mode=2;
                }
                else if((30<=headrobottheta)&&(headrobottheta<=90))
                {
                    std::cout<<"out of view mode:3"<<std::endl;   
                    //targetrobotpose.at(2) = robotyaw+headrobottheta;  
                    //targetrobotpose.at(2) = robotyaw+(360-headarrowtheta);
                    targetrobotpose.at(2) = robotyaw+(90-headrobottheta)+45;
                    //targetrobotpose.at(2) = robotyaw+90;
                    
                    out_view_mode=3;
                }
                else if((0>=headrobottheta)&&(headrobottheta>-10))
                {
                    std::cout<<"out of view mode:4"<<std::endl;   
                    targetrobotpose.at(2) = robotyaw;
                    out_view_mode=4;
                }
                else if((-10>=headrobottheta)&&(headrobottheta>-30))
                {
                    std::cout<<"out of view mode:5"<<std::endl;   
                    targetrobotpose.at(2) = headarrowtheta;
                    out_view_mode=5;
                }
                else if((-30>=headrobottheta)&&(headrobottheta>-90))
                {
                    std::cout<<"out of view mode:6"<<std::endl;   
                    //targetrobotpose.at(2) = robotyaw+headrobottheta;
                    //targetrobotpose.at(2) = robotyaw-((180-robotyaw)+headarrowtheta);
                    //targetrobotpose.at(2) = robotyaw-(90+headrobottheta)-45;
                    targetrobotpose.at(2) = robotyaw-90;
                    out_view_mode=6;
                }
                else
                {
                    std::cout<<"out of view mode error"<<std::endl;
                    std::cout<<"headarrowangle:"<<headarrowangle<<std::endl;
                    std::cout<<"headarrowtheta:"<<headarrowtheta<<std::endl;
                    std::cout<<"headrobottheta:"<<headrobottheta<<std::endl;
                }
                targetthetatmp = targetrobotpose.at(2);
                /*
                if((0<=headrobottheta)&&(headrobottheta<20))
                {
                    std::cout<<"out of view mode:1"<<std::endl;   
                    targetrobotpose.at(2) = robotyaw;
                    out_view_mode=1;
                }
                else if((20<=headrobottheta)&&(headrobottheta<=45))
                {
                    std::cout<<"out of view mode:2"<<std::endl;   
                    targetrobotpose.at(2) = headarrowtheta;
                    out_view_mode=2;
                }
                else if((45<=headrobottheta)&&(headrobottheta<=90))
                {
                    std::cout<<"out of view mode:3"<<std::endl;   
                    //targetrobotpose.at(2) = robotyaw+headrobottheta;  
                    //targetrobotpose.at(2) = robotyaw+(360-headarrowtheta);
                    targetrobotpose.at(2) = robotyaw+(90-headrobottheta)+45;
                    out_view_mode=3;
                }
                if((0>=headrobottheta)&&(headrobottheta>-20))
                {
                    std::cout<<"out of view mode:4"<<std::endl;   
                    targetrobotpose.at(2) = robotyaw;
                    out_view_mode=1;
                }
                else if((-20>=headrobottheta)&&(headrobottheta>-45))
                {
                    std::cout<<"out of view mode:5"<<std::endl;   
                    targetrobotpose.at(2) = headarrowtheta;
                    out_view_mode=2;
                }
                else if((-45>=headrobottheta)&&(headrobottheta>-90))
                {
                    std::cout<<"out of view mode:6"<<std::endl;   
                    //targetrobotpose.at(2) = robotyaw+headrobottheta;
                    //targetrobotpose.at(2) = robotyaw-((180-robotyaw)+headarrowtheta);
                    targetrobotpose.at(2) = robotyaw-(90+headrobottheta)-45;
                    out_view_mode=3;
                }
                */
                robot_moving = 1;
            }
            std::cout<<"out_view_mode;"<<out_view_mode<<std::endl;
        }
        else if(move_mode==ObjectFar)
        {
            std::cout<<"move_mode==ObjectFar"<<std::endl;
            if(robot_moving==0)
            {
    
                targetrobotpose.at(0) = robotpose.at(0);
                targetrobotpose.at(1) = robotpose.at(1);   
                targetrobotpose.at(2) = robotyaw;
                robot_moving = 1;   
            }  
        } 
        else if(move_mode==FaceOrinentationError)
        {
            std::cout<<"FaceOrinentationError"<<std::endl;
            if(robot_moving==0)
            {
    
                targetrobotpose.at(0) = robotpose.at(0);
                targetrobotpose.at(1) = robotpose.at(1);   
                targetrobotpose.at(2) = robotyaw;
                robot_moving = 1;   
            }  
        }
    }
    else if(move_mode==RobotPoseReset)
    {
        std::cout<<"move_mode===RobotPoseReset"<<std::endl;
        if((robot_moving==0)&&(pose_reset_cnt==PoseResetCount))
        {
            speechtxt.data = "start to reset the pose";
            pepper_speech_pub.publish(speechtxt);
           
            targetrobotpose.at(0) = beforerobotpose.at(0);
            targetrobotpose.at(1) = beforerobotpose.at(1);
            targetrobotpose.at(2) = beforerobotpose.at(2);
            robot_moving = 1;
            pose_reset = 2;
        }  
    } 
    else
    {
        std::cout<<"mode none"<<std::endl;
        std::cout<<"move_mode:"<<move_mode<<std::endl;
        std::cout<<"pose_reset:"<<pose_reset<<std::endl;
        std::cout<<"pose_reset_cnt:"<<pose_reset_cnt<<std::endl;
        //exit(1);
    }       

    if(robot_moving)
    {
        disterror = std::sqrt(std::pow(targetrobotpose.at(0)-robotpose.at(0), 2) + std::pow(targetrobotpose.at(1)-robotpose.at(1), 2));
        
        distxerror = targetrobotpose.at(0)-robotpose.at(0);
        distyerror = targetrobotpose.at(1)-robotpose.at(1);
        angleerror = targetrobotpose.at(2)-robotyaw;

        errortmptheta = angleerror;
        errortmpx = distxerror;
        errortmpy = distyerror;

        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.angular.z= 0;
        static int signalangle = 0;
        static int signallinearx = 0;
        static int signallineary = 0;
        static int signallinear = 0;
        double a = 0;
        double b = 0;
        double robottheta = 180-robotyaw;
        double robotphi = robotyaw-180;
        double linearsigval = 0.05;

        if((angleerror<=0)&&(abs(angleerror)>5)&&(!signallinearx)&&(!signallineary))
        {
            std::cout<<"angleerror<=0"<<std::endl;
            twist.angular.z= -0.1;
            
            signalangle = 1;
        }
        else if((angleerror>=0)&&(abs(angleerror)>5)&&(!signallinearx)&&(!signallineary))
        {
            std::cout<<"angleerror>=0"<<std::endl;          
            twist.angular.z= 0.1;
            signalangle = 1;
        }
        else
        {
            //std::cout<<"angle error else"<<std::endl;
            twist.angular.z= 0;
            signalangle = 0; 
        }
        
        //if((distyerror<=0)&&(abs(distyerror)>0.05)&&(!signalangle)&&(!signallinearx))
        if((distyerror<=0)&&(abs(distyerror)>0.1)&&(!signalangle)&&(!signallinearx))
        {
            std::cout<<"distyerror<=0"<<std::endl;
            signallineary = 1;
            if((90<=robotyaw)&&(robotyaw<=180))
            {
                std::cout<<"signal mode :1"<<std::endl;
                if((0<=robottheta)&&(robottheta<=45))
                {
                    twist.linear.y = linearsigval;
                    twist.linear.x = (-1*linearsigval)*(robottheta/45);
                }
                else if((45<robottheta)&&(robottheta<=90))
                {
                    Linear_Line(45,linearsigval,90.0,0.0,&a,&b);
                    twist.linear.y = a*robottheta+b;
                    twist.linear.x = -1*linearsigval;
                }
                else
                {
                    std::cout<<"robottheta none"<<std::endl;
                }     
            }
            else if((180<robotyaw)&&(robotyaw<270))
            {
                std::cout<<"signal mode :3"<<std::endl;
                if((0<=robotphi)&&(robotphi<=45))
                {
                    twist.linear.y = linearsigval;
                    twist.linear.x = linearsigval*(robotphi/45);
                }
                else if((45<robotphi)&&(robotphi<=90))
                {
                    Linear_Line(45,linearsigval,90.0,0.0,&a,&b);
                    twist.linear.y = a*robotphi+b;
                    twist.linear.x = linearsigval;
                }
                else
                {
                    std::cout<<"robotphi none"<<std::endl;
                } 
            }
            else
            {
                std::cout<<"signal mode none"<<std::endl;
            }       
        }
        else if((distyerror>=0)&&(abs(distyerror)>0.1)&&(!signalangle)&&(!signallinearx))
        {
           std::cout<<"distyerror>=0"<<std::endl;
           signallineary = 1;
           if((90<=robotyaw)&&(robotyaw<=180))
            {
                std::cout<<"signal mode :2"<<std::endl;
                if((0<=robottheta)&&(robottheta<=45))
                {
                    twist.linear.y = -1*linearsigval;
                    twist.linear.x = linearsigval*(robottheta/45);
                }
                else if((45<robottheta)&&(robottheta<=90))
                {
                    Linear_Line(45,(-1)*linearsigval,90,0,&a,&b);
                    twist.linear.y = a*robottheta+b;
                    twist.linear.x = linearsigval;
                }
                else
                {
                    std::cout<<"robottheta none"<<std::endl;
                }     
            }
            else if((180<robotyaw)&&(robotyaw<270))
            {
                std::cout<<"signal mode :4"<<std::endl;
                if((0<=robotphi)&&(robotphi<=45))
                {
                    twist.linear.y = (-1)*linearsigval;
                    twist.linear.x = (-1*linearsigval)*(robotphi/45);
                }
                else if((45<robotphi)&&(robotphi<=90))
                {
                    Linear_Line(45,(-1)*linearsigval,90.0,0.0,&a,&b);
                    twist.linear.y = a*robotphi+b;
                    twist.linear.x = (-1)*linearsigval;
                }
                else
                {
                    std::cout<<"robotphi none"<<std::endl;
                } 
            }
            else
            {
                std::cout<<"signal mode none"<<std::endl;
            }
        }
        else
        {
            signallineary = 0;
        }

        if((distxerror<=0)&&(abs(distxerror)>0.1)&&(!signalangle))
        {
            std::cout<<"distxerror<=0"<<std::endl;
            if((90<=robotyaw)&&(robotyaw<=180))
            {
                std::cout<<"signal mode :5"<<std::endl;
                if((0<=robottheta)&&(robottheta<=45))
                {
                    twist.linear.y = linearsigval;
                    twist.linear.x = linearsigval*(robottheta/45);
                }
                else if((45<robottheta)&&(robottheta<=90))
                {
                    Linear_Line(45,linearsigval,90,0,&a,&b);
                    twist.linear.y = linearsigval;
                    twist.linear.x = a*robottheta+b;
                }
                else
                {
                    std::cout<<"robottheta none"<<std::endl;
                }     
            }
            else if((180<robotyaw)&&(robotyaw<270))
            {
                std::cout<<"signal mode :7"<<std::endl;
                if((0<=robotphi)&&(robotphi<=45))
                {
                    twist.linear.y = (-1*linearsigval)*(robotphi/45);
                    twist.linear.x = linearsigval;
                }
                else if((45<robotphi)&&(robotphi<=90))
                {
                    Linear_Line(45,linearsigval,90,0,&a,&b);
                    twist.linear.y = (-1)*linearsigval;
                    twist.linear.x = a*robotphi+b;
                }
                else
                {
                    std::cout<<"robotphi none"<<std::endl;
                } 
            }
            else
            {
                std::cout<<"signal mode none"<<std::endl;
            }      
        }
        else if((distxerror>=0)&&(abs(distxerror)>0.1)&&(!signalangle))
        {
            std::cout<<"distxerror>=0"<<std::endl;
            if((90<=robotyaw)&&(robotyaw<=180))
            {
                std::cout<<"signal mode :6"<<std::endl;
                if((0<=robottheta)&&(robottheta<=45))
                {
                    twist.linear.y = (-1*linearsigval)*(robottheta/45);
                    twist.linear.x = -1*linearsigval;
                }
                else if((45<robottheta)&&(robottheta<=90))
                {
                    Linear_Line(45,(-1)*linearsigval,90,0,&a,&b);
                    twist.linear.y = -1*linearsigval;
                    twist.linear.x = a*robottheta+b;
                }
                else
                {
                    std::cout<<"robottheta none"<<std::endl;
                }     
            }
            else if((180<robotyaw)&&(robotyaw<270))
            {
                std::cout<<"signal mode :8"<<std::endl;
                if((0<=robotphi)&&(robotphi<=45))
                {
                    twist.linear.y = linearsigval*(robotphi/45);
                    twist.linear.x = (-1)*linearsigval;
                }
                else if((45<robotphi)&&(robotphi<=90))
                {
                    Linear_Line(45,(-1)*linearsigval,90,0,&a,&b);
                    twist.linear.y = linearsigval;
                    twist.linear.x = a*robotphi+b;
                }
                else
                {
                    std::cout<<"robotphi none"<<std::endl;
                } 
            }
            else
            {
                std::cout<<"signal mode none"<<std::endl;
            }
        }
        else
        {
            signallinearx = 0;
        }

        angularsig = Round(twist.angular.z,3);
        linearxsig = Round(twist.linear.x,3);
        linearysig = Round(twist.linear.y,3);

        std::cout<<"robotmovig:"<<robot_moving<<","<<"moving_cnt:"<<moving_cnt<<std::endl;
        std::cout<<"current pose:"<<robotpose.at(0)<<","<<robotpose.at(1)<<","<<robotyaw<<std::endl; 
        std::cout<<"target:"<<targetrobotpose.at(0)<<","<<targetrobotpose.at(1)<<","<<targetrobotpose.at(2)<<std::endl;
        std::cout<<"error:"<<distxerror<<","<<distyerror<<","<<angleerror<<std::endl;
        std::cout<<"twistx:"<<twist.linear.x<<","<<twist.linear.y<<","<<twist.angular.z<<std::endl;
        std::cout<<"signalinearx:"<<signallinearx<<std::endl;
        std::cout<<"signalineary:"<<signallineary<<std::endl;
        std::cout<<"signalangle:"<<signalangle<<std::endl;
        
        if(abs(twist.linear.y)>linearsigval)
        {
            exitflag = 1;
        }
        if(abs(twist.linear.x)>linearsigval)
        {
            exitflag = 1;
        }
        
        if(moving_cnt>RobotStopCount)
        {
            speechtxt.data = "robot stop and exit";
            pepper_speech_pub.publish(speechtxt);
            exitflag = 1;
        }

        /*
        if(moving_cnt==40)
        {
            speechtxt.data = "error reset";
            pepper_speech_pub.publish(speechtxt);
            distxerror = 0;
            distyerror = 0;
            angleerror = 0;
            signallineary = 0;
            signallinearx = 0;
            signalangle = 0;
        }
        */
        if((out_view_mode==1)||(out_view_mode==2)||(out_view_mode==4)||(out_view_mode==5))
        {
            signallineary = 0;
            signallinearx = 0;
        }
        if((!signallineary)&&(!signallinearx)&&(!signalangle))
        {
            std::cout<<"finish movement"<<std::endl;
            speechtxt.data = "finish movement";
            pepper_speech_pub.publish(speechtxt);
            notmeasurement_cnt = 0;
            moving_cnt = 0;
            robot_moving = 0;
            robotyaw = robotyawtmp;
            if(move_mode==OrientationInView)
            {
                //exitflag = 1;
                exitflag = 0;
            }
            if(pose_reset==2)
            {
                pose_reset = 0;
                pose_reset_cnt = 0;
                speechtxt.data = "finished pose reset and exit";
                pepper_speech_pub.publish(speechtxt);
                exitflag = 1;
            }
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            after_flag = 1;
        }
 
        visualization_msgs::Marker targetrobotposearrow;
        targetrobotposearrow.header.frame_id = fixed_frame;
        targetrobotposearrow.header.stamp = ros::Time::now();
        targetrobotposearrow.ns = "basic_shapes";
        targetrobotposearrow.type = visualization_msgs::Marker::ARROW;
        targetrobotposearrow.action = visualization_msgs::Marker::ADD;
        targetrobotposearrow.pose.position.x = targetrobotpose.at(0);
        targetrobotposearrow.pose.position.y = targetrobotpose.at(1);
        targetrobotposearrow.pose.position.z = 0;
        targetrobotposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(targetrobotpose.at(2)/180.0*M_PI);
        targetrobotposearrow.lifetime = ros::Duration();
        targetrobotposearrow.scale.x = 0.3;
        targetrobotposearrow.scale.y = 0.1;
        targetrobotposearrow.scale.z = 0.1;
        targetrobotposearrow.color.r = 1.0f;
        targetrobotposearrow.color.g = 0.0f;
        targetrobotposearrow.color.b = 0.0f;
        targetrobotposearrow.color.a = 1.0f;
        target_robotpose_pub.publish(targetrobotposearrow);

        if(exitflag)
        {
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            velocity_pub.publish(twist);
            exit(1);    
        }
        else
        {
            velocity_pub.publish(twist);
            //exit(1); 
        }   

        /*
        notmeasurement_cnt = 0;
        moving_cnt = 0;
        robot_moving = 0;
        pose_reset = 0;
        robotyaw = robotyawtmp;
        signalangle = 0;
        signallinearx = 0;
        signallineary = 0;
        after_flag = 0;
        */
    }
}

void CombiDarknetOpenface::Calculate_TimeUse(double currenttimesec)
{
    noseendminindex = 0;
    nose_end_point2D_drawmin.clear();
    detectedobjectbox.clear();
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
        for(int j=0;j<DistanceRange/DistanceStep;j++)
        {
            i += DistanceStep;
            distancestep.push_back(i);
        }
        double thetanoseendtmpx = nose_end_point2D_drawtmp[0]-nose_tip[0];
        double thetanoseendtmpy = nose_end_point2D_drawtmp[1]-nose_tip[1];
        double thetanoseendtmp = atan2(thetanoseendtmpy,thetanoseendtmpx)* 180 / PI;

        double thetanoseend3x = nose_end_point2D_draw3[0]-nose_tip[0];
        double thetanoseend3y = nose_end_point2D_draw3[1]-nose_tip[1];
        double thetanoseend3 = atan2(thetanoseend3y,thetanoseend3x)* 180 / PI;
        double thetanoseend2x = nose_end_point2D_draw2[0]-nose_tip[0];
        double thetanoseend2y = nose_end_point2D_draw2[1]-nose_tip[1];
        double thetanoseend2 = atan2(thetanoseend2y,thetanoseend2x)* 180 / PI;
        double thetanoseendx = nose_end_point2D_draw[0]-nose_tip[0];
        double thetanoseendy = nose_end_point2D_draw[1]-nose_tip[1];
        double thetanoseend = atan2(thetanoseendy,thetanoseendx)* 180 / PI;

        /*
        std::cout<<"nose:"<<nose_tip[0]<<", "<<nose_tip[1]<<std::endl;    
        cout << "nose_end_point2D_drawtmp  :"<<nose_end_point2D_drawtmp[0]<<","<<nose_end_point2D_drawtmp[1]<< endl;
        std::cout << "thetanoseendtmp:"<< thetanoseendtmp << std::endl;
        cout << "nose_end_point2D_draw  :"<<nose_end_point2D_draw[0]<<","<<nose_end_point2D_draw[1]<< endl;
        std::cout << "thetanoseend:"<< thetanoseend << std::endl;
        cout << "nose_end_point2D_draw2  :"<<nose_end_point2D_draw2[0]<<","<<nose_end_point2D_draw2[1]<< endl;
        std::cout << "thetanoseend2:"<< thetanoseend2 << std::endl;
        cout << "nose_end_point2D_draw3  :"<<nose_end_point2D_draw3[0]<<","<<nose_end_point2D_draw3[1]<< endl;
        std::cout << "thetanoseend3:"<< thetanoseend3 << std::endl;
        */

        std::vector<float> noseenddistance;
        std::vector<float> noseenddistancetmp;
        std::vector<int> eachminindex;
        std::vector<float> eachminnoseenddistancetmp;
        std::vector<float> eachnoseend;
        int noseendminindextmp = 0;
        int minnoseend = 10000;
        float minnoseenddistancetmp = 100000;
        int mindistancestep = 0;

        nose_end_point2D.clear();
        nose_end_point3D.clear();
        nose_end_point3D.push_back(cv::Point3d(0,0,TmpDistance));
        projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);
        
        /*
        //const face orientation
        std::cout<<"########const face orientation#############" << std::endl;
        std::vector<float> objectdistance;
        std::vector<float> objectdistancetmp;
        for(int i=0;i<classnames.size();i++)
        {
            objectdistance.push_back(std::sqrt(std::pow(boxcenterx.at(i)-nose_end_point2D_draw3.at(0) , 2) + std::pow(boxcentery.at(i)-nose_end_point2D_draw3.at(1), 2)));
         
            if((boxcenterx.at(i) == 0)&&(boxcentery.at(i) == 0))
            {
                 objectdistance.at(i) = 0;
            }

            if(objectdistance.at(i)>0)
            {
                  objectdistancetmp.push_back(objectdistance.at(i));
             }
        }

        //commentout(11/21)
        std::cout<<"objectdistance  :"<<" ";
        for(int i=0;i<objectdistance.size();i++)
        std::cout << objectdistance.at(i) << " ";
        std::cout <<""<< std::endl;
        std::cout<<"objectdistancetmp  :"<<" ";
        for(int i=0;i<objectdistancetmp.size();i++)
        std::cout << objectdistancetmp.at(i) << " ";
        std::cout <<""<< std::endl;
      
        if(objectdistancetmp.empty())
        {
              std::cout << "no detected objects " << std::endl;
             mindistanceindex = 0;
        }
        else
         {
            auto minobjectdistance = std::min_element(std::begin(objectdistancetmp), std::end(objectdistancetmp));
            std::cout << "*minobjectdistance:"<< *minobjectdistance << std::endl;
            float minobjectdistancetmp = *minobjectdistance;
            std::vector<float>::iterator citerobdist =std::find(objectdistance.begin(), objectdistance.end(), minobjectdistancetmp);
            if (citerobdist != objectdistance.end()) 
            {
                mindistanceindex = std::distance(objectdistance.begin(), citerobdist);
            }
            if(minobjectdistancetmp>65)
            {
                std::cout << "gaze is not Assigned" << std::endl;
                mindistanceindex = 0;
            }
            else if(!(((nose_end_point2D_draw3[0]>0)&&(nose_end_point2D_draw3[0]<640))&&((nose_end_point2D_draw3[1]>0)&&(nose_end_point2D_draw3[1]<480))))
            {
                std::cout << "gaze is not Measured" << std::endl;
                mindistanceindex = 0;
            }    
            std::cout << "mindistanceindex:"<< mindistanceindex << std::endl;
        }
        activityscoreface.at(mindistanceindex) += 1;
        */

        //variable face orintation
        for(int i=0;i<distancestep.size();i++)
        {
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3d(0,0,distancestep.at(i)));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);

            double thetanoseendmintmpx = nose_end_point2D[0].x-nose_tip[0];
            double thetanoseendmintmpy = nose_end_point2D[0].y-nose_tip[1];
            double thetanoseendmintmp = atan2(thetanoseendmintmpy,thetanoseendmintmpx)* 180 / PI;
            
            if(abs(thetanoseendtmp-thetanoseendmintmp)<10)
            {
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
                    //move_mode = OrientationInView;
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
        }
        
        if(!nose_end_point2D_drawmin.empty())
        {
            int noseendminindextmp2 = noseendminindex;
            if(minnoseend>65)
            {
                std::cout << "gaze is not Assigned" << std::endl;
                noseendminindex = 0;
                //move_mode = OrientationInView;
            }
            
            if(noseobjectmindist>persondepthdist)
            {
                std::cout << "object is far" << std::endl;
                noseendminindex = 0;
                //exit(1);
                object_far = 1; 
            }
            else
            {
                object_far = 0;
            }
            
            
            if(!noseendminindex)
            {
                double nosetoendmin = std::sqrt(std::pow(nose_end_point2D_drawmin[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_drawmin[1]-nose_tip[1], 2));
            
                double nosetoendconst3 = std::sqrt(std::pow(nose_end_point2D_draw3[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_draw3[1]-nose_tip[1], 2));
                double nosetoendconst2 = std::sqrt(std::pow(nose_end_point2D_draw2[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_draw2[1]-nose_tip[1], 2));
                double nosetoendconsttmp = std::sqrt(std::pow(nose_end_point2D_drawtmp[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_drawtmp[1]-nose_tip[1], 2));
                
                /*
                if(nosetoendconsttmp<nosetoendmin) 
                {
                    nose_end_point2D_drawmin[0] = nose_end_point2D_drawtmp[0];
                    nose_end_point2D_drawmin[1] = nose_end_point2D_drawtmp[1];
                    std::cout << "nose_end_point2D_drawmin modified tmp"<<std::endl;
                }
                */
                   
                int gazeobjectx = boxcenterx.at(noseendminindextmp2)-nose_tip[0];
                int gazeobjecty = boxcentery.at(noseendminindextmp2)-nose_tip[1];
                double thetagazeobject = atan2(gazeobjecty,gazeobjectx)* 180 / PI;
                
                double thetanoseendminx = nose_end_point2D_drawmin[0]-nose_tip[0];
                double thetanoseendminy = nose_end_point2D_drawmin[1]-nose_tip[1];
                double thetanoseendmin = atan2(thetanoseendminy,thetanoseendminx)* 180 / PI;
                
                if(abs(thetagazeobject-thetanoseendmin)<30)
                {
                    move_mode = OrientationInView;   
                    if(object_far)
                    {
                        move_mode = ObjectFar;   
                    }      
                }
                else
                {
                    move_mode = OrientationOutView;
                }
                //move_mode = OrientationOutView;
                //move_mode = OrientationOutView;         
                
                double nosetoend = std::sqrt(std::pow(nose_end_point2D_drawtmp[0]-nose_tip[0], 2) + std::pow(nose_end_point2D_drawtmp[1]-nose_tip[1], 2));
                
                /*
                if((nosetoend<((right_eye[0]-left_eye[0])/2)*2.5))
                {
                    move_mode = OrientationFront;         
                }
                std::cout << "eye:" << (right_eye[0]-left_eye[0])/2 <<std::endl;          
                std::cout << "nosetoend:" << nosetoend <<std::endl;              
                */
                std::cout << "######indicator########"<< std::endl;
                std::cout << "nose:" << nose_tip[0] << "," << nose_tip[1]<< std::endl;
                std::cout << "noseendminindextmp2:"<< noseendminindextmp2 << std::endl;
                std::cout << "boxcenter.at(noseendminindextmp2)  :"<<boxcenterx.at(noseendminindextmp2)<<","<<boxcentery.at(noseendminindextmp2)<< endl;
                std::cout << "thetagazeobject:"<< thetagazeobject << std::endl;
                std::cout << "nose_end_point2D_drawmin  :"<<nose_end_point2D_drawmin[0]<<","<<nose_end_point2D_drawmin[1]<< endl;
                std::cout << "thetanoseendmin:"<< thetanoseendmin << std::endl;
            }
        }
        else
        {
            std::cout << "nose_end_point2D_drawmin.empty()" << std::endl;
            noseendminindex = 0;
            move_mode=FaceOrinentationError;       
        }
        
        //commentout(11/21)
        std::cout << "noseobjectmindist:"<< noseobjectmindist << std::endl;
        std::cout << "persondepthdist:"<< noseobjectmindist << std::endl;
        std::cout << "minnoseend:"<< minnoseend << std::endl;
        std::cout << "noseendminindex:"<< noseendminindex << std::endl;
        cout << "boxcenter  :"<<boxcenterx[noseendminindex]<<","<<boxcentery[noseendminindex]<< endl;
        
        std::cout << "mindistancestep:"<< mindistancestep << std::endl;
        
        activityscoreface.at(noseendminindex) += 1;
        if(noseendminindex)
        {
            detectedobjectbox.push_back(boxxmin2.at(noseendminindex));
            detectedobjectbox.push_back(boxymin2.at(noseendminindex));
            detectedobjectbox.push_back(boxxmax2.at(noseendminindex));
            detectedobjectbox.push_back(boxymax2.at(noseendminindex));
        }

        if(darknet_cnt==1)
        {
            lastmove_mode = move_mode;    
        }
        if(noseendminindex)
        {
            notmeasurement_cnt = 0;
            move_mode = 0;    
        }
        else if(lastmove_mode == move_mode)
        {
            notmeasurement_cnt += 1;
        }
        else
        {
            notmeasurement_cnt = 0;
        }
        lastmove_mode = move_mode;
        std::cout << "move_mode:"<< move_mode << std::endl;
        std::cout << "notmeasurement_cnt:"<< notmeasurement_cnt << std::endl;
        
    }
    //object movement
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
                    if(classnames.at(i)=="bench")
                    {
                        objectmovement.at(i) = 0;   
                    }
                    if(classnames.at(i)=="oven")
                    {
                        objectmovement.at(i) = 0;   
                    }
                    if(classnames.at(i)=="refrigerator")
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
    std::cout <<"calculate time-use end"<< std::endl;
}

void CombiDarknetOpenface::Calculate_TimeUseOutofView(double currenttimesec)
{
    detectedobjectbox.empty();
    std::cout<<"Calculate_TimeUseOutofView"<<std::endl;
    //const face orientation
    std::cout<<"########const face orientation#############" << std::endl;
    std::vector<float> objectdistance;
    std::vector<float> objectdistancetmp;
    for(int i=0;i<classnames.size();i++)
    {
        objectdistance.push_back(std::sqrt(std::pow(boxcenterx.at(i)-320, 2) + std::pow(boxcentery.at(i)-240, 2)));
         
        if((boxcenterx.at(i) == 0)&&(boxcentery.at(i) == 0))
        {
            objectdistance.at(i) = 0;
        }
       if(objectdistance.at(i)>0)
        {
            objectdistancetmp.push_back(objectdistance.at(i));
        }
    }

    //commentout(11/21)
    std::cout<<"objectdistance  :"<<" ";
    for(int i=0;i<objectdistance.size();i++)
    std::cout << objectdistance.at(i) << " ";
    std::cout <<""<< std::endl;
    std::cout<<"objectdistancetmp  :"<<" ";
    for(int i=0;i<objectdistancetmp.size();i++)
    std::cout << objectdistancetmp.at(i) << " ";
    std::cout <<""<< std::endl;
  
    if(objectdistancetmp.empty())
    {
        std::cout << "no detected objects " << std::endl;
        noseendminindex = 0;
    }
    else
    {
        auto minobjectdistance = std::min_element(std::begin(objectdistancetmp), std::end(objectdistancetmp));
        std::cout << "*minobjectdistance:"<< *minobjectdistance << std::endl;
        float minobjectdistancetmp = *minobjectdistance;
        std::vector<float>::iterator citerobdist =std::find(objectdistance.begin(), objectdistance.end(), minobjectdistancetmp);
        if(citerobdist != objectdistance.end()) 
        {
            noseendminindex = std::distance(objectdistance.begin(), citerobdist);
        }

        double xcerror =  boxcenterx.at(noseendminindex)-320;
        //std::cout << "detectedobjectxc:"<< detectedobjectxc << std::endl;
        if(abs(xcerror)>200)
        {
            std::cout << "gaze is not Assigned" << std::endl;
            noseendminindex = 0;
        }  
        std::cout << "xcerror:"<< xcerror << std::endl;
        
        /*
        if(minobjectdistancetmp>200)
        {
            std::cout << "gaze is not Assigned" << std::endl;
            noseendminindex = 0;
        }  
        std::cout << "noseendminindex:"<< noseendminindex << std::endl;
        */
    }
    activityscoreface.at(noseendminindex) += 1;
    if(noseendminindex)
    {
        detectedobjectbox.push_back(boxxmin2.at(noseendminindex));
        detectedobjectbox.push_back(boxymin2.at(noseendminindex));
        detectedobjectbox.push_back(boxxmax2.at(noseendminindex));
        detectedobjectbox.push_back(boxymax2.at(noseendminindex));

        /*
        double detectedobjectxc = (detectedobjectbox.at(0)+detectedobjectbox.at(2))/2;
        double xcerror =  detectedobjectxc-320;
        std::cout << "detectedobjectxc:"<< detectedobjectxc << std::endl;
        std::cout << "xcerror:"<< xcerror << std::endl;
        */
    }
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
        cvMoveWindow("RGB darknet image", 2910,10);
        //cvMoveWindow("RGB darknet image", 1000,0);
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
    cv::Mat dataimage = cv::Mat::zeros(480, 640, CV_8UC3);
    
    /*
    if(!nose_tip.empty())
    {
        cv::circle(cv_ptr->image, cv::Point(nose_tip[0], nose_tip[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(chin[0], chin[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_eye[0], left_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_eye[0], right_eye[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(left_mouth[0], left_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::circle(cv_ptr->image, cv::Point(right_mouth[0], right_mouth[1]), 10, cv::Scalar(0, 0, 255), 3);
        cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_drawtmp[0],nose_end_point2D_drawtmp[1]), cv::Scalar(0,255,0), 5);
    }
    */
    
    if(!after_flag)
    {
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
                    //cv::circle(cv_ptr->image, cv::Point(boxcenterx[maxindexface], boxcentery[maxindexface]), 5, cv::Scalar(255,0,0), 3);
                    //cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(boxcenterx[maxindexface],boxcentery[maxindexface]), cv::Scalar(255,0,0), 2);
                }
            }
            else
            {
                maxindexface = 0;
            }
            //std::cout << "maxindexface:"<< maxindexface << std::endl;

            if(!nose_end_point2D_drawmin.empty())
            {
                cv::putText(cv_ptr->image, classnames.at(maxindexface), cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
            }
            else
            {
                cv::putText(cv_ptr->image, "cannot measure", cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
            }

            //cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw[0], nose_end_point2D_draw[1]), 5, cv::Scalar(255, 255,0), 3);
            //cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw2[0], nose_end_point2D_draw2[1]), 5, cv::Scalar(0,255,255), 3);
            cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_draw3[0], nose_end_point2D_draw3[1]), 5, cv::Scalar(255,255,255), 3);
 
            double thetanoseendtmpx = nose_end_point2D_drawtmp[0]-nose_tip[0];
            double thetanoseendtmpy = nose_end_point2D_drawtmp[1]-nose_tip[1];
            double thetanoseendtmp = atan2(thetanoseendtmpy,thetanoseendtmpx)* 180 / PI;
            double thetanoseend3x = nose_end_point2D_draw3[0]-nose_tip[0];
            double thetanoseend3y = nose_end_point2D_draw3[1]-nose_tip[1];
            double thetanoseend3 = atan2(thetanoseend3y,thetanoseend3x)* 180 / PI;
            double thetanoseend2x = nose_end_point2D_draw2[0]-nose_tip[0];
            double thetanoseend2y = nose_end_point2D_draw2[1]-nose_tip[1];
            double thetanoseend2 = atan2(thetanoseend2y,thetanoseend2x)* 180 / PI;
            double thetanoseendx = nose_end_point2D_draw[0]-nose_tip[0];
            double thetanoseendy = nose_end_point2D_draw[1]-nose_tip[1];
            double thetanoseend = atan2(thetanoseendy,thetanoseendx)* 180 / PI;
    
            if((((nose_end_point2D_draw3[0]>0)&&(nose_end_point2D_draw3[0]<640))&&((nose_end_point2D_draw3[1]>0)&&(nose_end_point2D_draw3[1]<480)))&&(abs(thetanoseendtmp-thetanoseend3)<10))
            {
                //cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw3[0],nose_end_point2D_draw3[1]), cv::Scalar(255,255,225), 2);
                cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw3[0],nose_end_point2D_draw3[1]), cv::Scalar(0,255,0), 2);
            }
            else if((((nose_end_point2D_draw2[0]>0)&&(nose_end_point2D_draw2[0]<640))&&((nose_end_point2D_draw2[1]>0)&&(nose_end_point2D_draw2[1]<480)))&&(abs(thetanoseendtmp-thetanoseend2)<10))
            {    
                //cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw2[0],nose_end_point2D_draw2[1]), cv::Scalar(0,255,225), 2);
                cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw2[0],nose_end_point2D_draw2[1]), cv::Scalar(0,255,0), 2);
            }
            else if((((nose_end_point2D_draw[0]>0)&&(nose_end_point2D_draw[0]<640))&&((nose_end_point2D_draw[1]>0)&&(nose_end_point2D_draw[1]<480)))&&(abs(thetanoseendtmp-thetanoseend)<10))
            {
                //cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw[0],nose_end_point2D_draw[1]), cv::Scalar(255,255,0), 2);
                cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_draw[0],nose_end_point2D_draw[1]), cv::Scalar(0,255,0), 2);
                
            }
        
            //nose_end_point2D_drawmin
            cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_drawmin[0], nose_end_point2D_drawmin[1]), 7, cv::Scalar(0,0,255), 5);

            //cv::line(cv_ptr->image,cv::Point(nose_tip[0],nose_tip[1]),cv::Point(nose_end_point2D_drawtmp[0],nose_end_point2D_drawtmp[1]), cv::Scalar(0,255,0), 5);
            //cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_drawtmp[0], nose_end_point2D_drawtmp[1]), 5, cv::Scalar(0,255,0), 3);  
        }

       if(notmeasurement_cnt==RobotMoveCount)
        {
            //cv::putText(cv_ptr->image, "robot moving", cv::Point(20,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            //cv::putText(cv_ptr->image, std::to_string(moving_cnt), cv::Point(550,55),   CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,255), 2,CV_AA); 
        }
        else if(pose_reset)
        {
            if((pose_reset_cnt==PoseResetCount))
            {
                cv::putText(cv_ptr->image, "robot returning", cv::Point(20,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            }
            else
            {
                int maxindexface = 0;    
                if(pose_reset_cnt!=PoseResetCount)
                {
                    int zerocntface = std::count(activityscoreface.begin(), activityscoreface.end(), 0);
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
                            cv::line(cv_ptr->image,cv::Point(320,240),cv::Point(boxcenterx[maxindexface],boxcentery[maxindexface]), cv::Scalar(255,0,0), 2);
                        }
                    }
                    else
                    {
                        maxindexface = 0;
                    }
                }
                cv::putText(cv_ptr->image, classnames.at(maxindexface), cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
                cv::putText(cv_ptr->image, "out measuring", cv::Point(20,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA); 
            }    
        }
        else
        {
            /*
            cv::putText(cv_ptr->image, tostr(notmeasurement_cnt), cv::Point(20,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
        
            if(move_mode == OrientationInView)
            {
                cv::putText(cv_ptr->image, "In View", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            }
            else if(move_mode==OrientationOutView)
            {
                if((0<=headrobottheta)&&(headrobottheta<10))
                {
                    cv::putText(cv_ptr->image, "Out of View Robot", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);               
                }
                else if((10<=headrobottheta)&&(headrobottheta<=25))
                {
                    cv::putText(cv_ptr->image, "Out of View Behind", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);                        
                }
                else if((25<=headrobottheta)&&(headrobottheta<=90))
                {
                    cv::putText(cv_ptr->image, "Out of View Side", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);                           
                }
                else if((0>=headrobottheta)&&(headrobottheta>-10))
                {
                    cv::putText(cv_ptr->image, "Out of View Robot", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);              
                }
                else if((-10>=headrobottheta)&&(headrobottheta>-25))
                {
                    cv::putText(cv_ptr->image, "Out of View Behind", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);                                          
                }
                else if((-25>=headrobottheta)&&(headrobottheta>-90))
                {
                    cv::putText(cv_ptr->image, "Out of View Side", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);                           
                }
                else
                {
                    cv::putText(cv_ptr->image, "Out of View None", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);                                          
                }
            }
            else if(move_mode==OrientationFront)
            {
                cv::putText(cv_ptr->image, "OrientationFront", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            }
            else if(move_mode==RobotPoseReset)
            {
                cv::putText(cv_ptr->image, "RobotPoseReset", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            }
            else if(move_mode==ObjectFar)
            {
                cv::putText(cv_ptr->image, "ObjectFar", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            }
            else if(move_mode==FaceOrinentationError)
            {
                cv::putText(cv_ptr->image, "FaceOrinentationError", cv::Point(90,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            } 
        */
        }
    }

    cv::putText(dataimage, std::to_string(darknet_cnt), cv::Point(550,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA); 
    if(!personbox.empty())
    {
        cv::putText(dataimage, "person found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }
    else
    {
        cv::putText(cv_ptr->image, "person not found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }
    
    /*
    i=0;
    for(int i=0;i<classnames.size();i++)
    {        
        if(classnames.at(i)=="bottle")
        {
            cv::rectangle(cv_ptr->image, cv::Point(boxxmin2.at(i), boxymin2.at(i)), cv::Point(boxxmax2.at(i), boxymax2.at(i)), cv::Scalar(255, 0, 0), 3, 4);
        }
    }

    */

    cv::putText(dataimage, tostr(angularsig), cv::Point(20,120), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(linearxsig), cv::Point(20,150), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(linearysig), cv::Point(20,180), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);

    cv::putText(dataimage, tostr(errortmptheta), cv::Point(140,120), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(errortmpx), cv::Point(140,150), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(errortmpy), cv::Point(140,180), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(headarrowangle), cv::Point(20,210), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(headarrowtheta), cv::Point(20,240), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(headrobottheta), cv::Point(20,270), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);

    cv::putText(dataimage, tostr(robotyaw), cv::Point(20,330), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(targetthetatmp), cv::Point(20,360), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(errortmptheta), cv::Point(20,390), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);


    cv::putText(dataimage, std::to_string(moving_cnt), cv::Point(550,55), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,255), 2,CV_AA);
    cv::putText(dataimage, std::to_string(pose_reset_cnt), cv::Point(550,85), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,0,255), 2,CV_AA);
    cv::putText(dataimage, std::to_string(out_view_mode), cv::Point(550,115), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);
    
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

    //cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), ResizeSize, ResizeSize);
    cv::resize(dataimage, dataimage, cv::Size(), ResizeSize, ResizeSize);
    
    cv::namedWindow("data", CV_WINDOW_AUTOSIZE);
    cv::imshow("data", dataimage);
    if(display_num==2)
    {
        //two displays
        cvMoveWindow("data",2910,350);
        //cvMoveWindow("RGB image", 990,350);
    }
    else
    {
        //one display
        cvMoveWindow("data", 1350,350);
    }
    
    cv::imshow("RGB image", cv_ptr->image);
    if(display_num==2)
    {
        //two displays
        cvMoveWindow("RGB image",2910,350);
        //cvMoveWindow("RGB image", 990,350);
    }
    else
    {
        //one display
        cvMoveWindow("RGB image",1000,350);
        
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
    double persondepthpoint = 0.0;
    double persondist = 0.0;
    double personangle = 0.0;
    double persondisttmp = 0.0;

    int objectx1ori = 0;
    int objectx2ori = 0;
    int objecty1ori = 0;
    int objecty2ori = 0;
    int objectx1 = 0;
    int objectx2 = 0;
    int objecty1 = 0;
    int objecty2 = 0;
    int objectxc = 0;
    int objectyc = 0;
    int objectpointx1 = 0;
    int objectpointx2 = 0;
    int objectpointy1 = 0;
    int objectpointy2 = 0;
    double objectdepthpoint = 0.0;
    double objectdist = 0.0;
    double objectangle = 0.0;
    double objectdisttmp = 0.0;


    int i, j, k;
    int width = WIDTH;
    int height = HEIGHT;
    double sum = 0.0;
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

    if((!personbox.empty())||(!detectedobjectbox.empty()))
    {
        if(!personbox.empty())
        {
            x1ori = personbox.at(0);
            y1ori = personbox.at(1);
            x2ori = personbox.at(2);
            y2ori = personbox.at(3);

            xc = (x1ori+x2ori)/2;
            yc = (y1ori+y2ori)/2;

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
        }
        std::vector<double>boxdepthivalue;

        if(!detectedobjectbox.empty())
        {
            objectx1ori = detectedobjectbox.at(0);
            objecty1ori = detectedobjectbox.at(1);
            objectx2ori = detectedobjectbox.at(2);
            objecty2ori = detectedobjectbox.at(3);

            objectxc = (objectx1ori+objectx2ori)/2;
            objectyc = (objecty1ori+objecty2ori)/2;
            objectpointx1 = objectxc-1;
            objectpointy1 = objectyc-1;
            objectpointx2 = objectxc+1;
            objectpointy2 = objectyc+1; 
        }

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
                            persondepthpoint = Dimage[j];
                        }
                    }
                }
                if(i > objectpointy1 && i < objectpointy2)
                {
                    if(j > objectpointx1 && j < objectpointx2)
                    {
                        if(Dimage[j] > 0.0)
                        {
                            objectdepthpoint = Dimage[j];
                        }
                    }
                }
            }
        } 
        noseobjectmindist = 0;
        persondepthdist = 0;
        if(!personbox.empty())
        {
             //distance = sum / ((x2-x1)*(y2-y1));
             persondist = persondepthpoint;
             personangle = AngleofView/2-((double)xc/640)*AngleofView;
             persondisttmp = persondist;
             CombiDarknetOpenface::ModifyPersonDistance(&persondist);
             persondepthdist = persondist;
             std::cout<<"persondist,angle:"<<persondist<<","<<personangle<<std::endl;
             if((fixed_frame =="map")&&(robotpose_cnt>0))
             {
                 Eigen::MatrixXd Ap = Eigen::MatrixXd(3,3);
                 Eigen::MatrixXd Bp = Eigen::MatrixXd(3,3);
                 Eigen::MatrixXd Tp = Eigen::MatrixXd(3,3);
                 
                 Ap << cos(robotyaw/180.0*M_PI), sin(robotyaw/180.0*M_PI), robotpose.at(0), 
                       sin(robotyaw/180.0*M_PI), cos(robotyaw/180.0*M_PI), robotpose.at(1), 
                       0,  0,   1;
                 
                 Bp << cos(personangle/180.0*M_PI), sin(personangle/180.0*M_PI), persondist*cos(personangle/180.0*M_PI), 
                       sin(personangle/180.0*M_PI), cos(personangle/180.0*M_PI), persondist*sin(personangle/180.0*M_PI), 
                       0,  0,   1;
                 
                 Tp = Ap*Bp;
     
                 double personmeasurementx = Tp(0,2);
                 double personmeasurementy = Tp(1,2);
     
                 /*//commentout(11/21)
                 std::cout<<"persondist:"<<persondist<<std::endl;
                 std::cout<<"personangle:"<<personangle<<std::endl;             
                 std::cout << "Ap: \n" << Ap << std::endl;
                 std::cout << "Cp: \n" << Cp << std::endl;
                 std::cout << "Tp: \n" << Tp << std::endl;
                 */
                 std::cout<<"personmeasurement:"<<personmeasurementx<<","<<personmeasurementy<<std::endl;
     
                 CombiDarknetOpenface::PublishPersonMeasurement(personmeasurementx,personmeasurementy);
                 CombiDarknetOpenface::PublishPersonMarker(personangle,personmeasurementx,personmeasurementy); 
             }
             else if(fixed_frame =="base_link")
             {
                 double personmeasurementx = persondist * cos(personangle/180.0*M_PI);
                 double personmeasurementy = persondist * sin(personangle/180.0*M_PI);
                 CombiDarknetOpenface::PublishPersonMeasurement(personmeasurementx,personmeasurementy);
                 CombiDarknetOpenface::PublishPersonMarker(personangle,personmeasurementx,personmeasurementy);  
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
        if(!detectedobjectbox.empty())
        {
            //distance = sum / ((x2-x1)*(y2-y1));
            objectdist = objectdepthpoint;
            noseobjectmindist = objectdist;
            objectangle = AngleofView/2-((double)objectxc/640)*AngleofView;
            objectdisttmp = objectdist;
            CombiDarknetOpenface::ModifyObjectDistance(&objectdist);
            noseobjectmindist = objectdist;
            std::cout<<"objectdist,angle:"<<objectdist<<","<<objectangle<<std::endl;
            if((fixed_frame =="map")&&(robotpose_cnt>0))
            {
                Eigen::MatrixXd Ao = Eigen::MatrixXd(3,3);
                Eigen::MatrixXd Bo = Eigen::MatrixXd(3,3);
                Eigen::MatrixXd To = Eigen::MatrixXd(3,3);
                
                Ao << cos(robotyaw/180.0*M_PI), sin(robotyaw/180.0*M_PI), robotpose.at(0), 
                      sin(robotyaw/180.0*M_PI), cos(robotyaw/180.0*M_PI), robotpose.at(1), 
                      0,  0,   1;
                
                Bo << cos(objectangle/180.0*M_PI), sin(objectangle/180.0*M_PI), objectdist*cos(objectangle/180.0*M_PI), 
                      sin(objectangle/180.0*M_PI), cos(objectangle/180.0*M_PI), objectdist*sin(objectangle/180.0*M_PI), 
                      0,  0,   1;
                
                To = Ao*Bo;
    
                double objectmeasurementx = To(0,2);
                double objectmeasurementy = To(1,2);
    
                /*//commentout(11/21)
                std::cout<<"objectdist:"<<objectdist<<std::endl;
                std::cout<<"objectangle:"<<objectangle<<std::endl;             
                std::cout << "Ao: \n" << Ao << std::endl;
                std::cout << "Co: \n" << Co << std::endl;
                std::cout << "To: \n" << To << std::endl;
                */
                std::cout<<"objectmeasurement:"<<objectmeasurementx<<","<<objectmeasurementy<<std::endl;
                cv::rectangle(img, cv::Point(detectedobjectbox.at(0), detectedobjectbox.at(1)), cv::Point(detectedobjectbox.at(2), detectedobjectbox.at(3)), cv::Scalar(0, 0, 255), 5, 4);
                
                CombiDarknetOpenface::PublishObjectMarker(objectmeasurementx,objectmeasurementy); 
            }
        }
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

    std::cout<<"persondepthdist:"<<persondepthdist<<std::endl;
    std::cout<<"noseobjectmindist:"<<noseobjectmindist<<std::endl;
    

    cv::resize(img, img, cv::Size(), ResizeSize, ResizeSize);
    cv::imshow("Depth_Image", img);

    
    if(display_num==2)
    {
        cvMoveWindow("Depth_Image", 3250,10);
        //cvMoveWindow("Depth_Image", 1450,0);
    }
    else
    {
        //one display
        cvMoveWindow("Depth_Image", 1350,0);
        //cvMoveWindow("Depth_Image", 10,0);
    }
    
    cv::waitKey(10);
    
     depthdata<<darknet_cnt<<","
     <<currenttimesec<<", "
     << persondisttmp<<", "
     << persondist<<", "
     << xc<<", "
     << yc<<", "
     <<modify_distance_cnt<<","
     <<person_move_cnt<<","
     << person_move<<std::endl;  

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

void CombiDarknetOpenface::ModifyObjectDistance(double *distance)
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
        person_move = 2; 
    }
    else if((abs(personvelocity.at(0))>0.04)&&(abs(personvelocity.at(1))>0.04))
    {
        person_move_cnt += 1;
    }
    else if ((abs(personvelocity.at(0))>0.06)||(abs(personvelocity.at(1))>0.06))
    {
        person_move_cnt += 1;   
    }
    else
    {
        //person is stopping
        person_move = 0;
        person_move_cnt = 0;
    }

    if(person_move_cnt>4)
    {
        //person is moving
        person_move=1;
    }
    
    lastcurrenttimevelocity  = currenttimesec;

    std::cout<<"person_move_cnt:"<<person_move_cnt<<std::endl;
    std::cout<<"person_move:"<<person_move<<std::endl;
    std::cout<<"estimateposition:"<<estimateposition[0]<<","<<estimateposition[1]<<std::endl;

    //std::cout<<"lastestimateposition:"<<lastestimateposition[0]<<","<<lastestimateposition[1]<<std::endl;
     //std::cout<<"personvelocity:"<<personvelocity[0]<<","<<personvelocity[1]<<std::endl;
    personvelocitydata<<darknet_cnt<<","
    <<currenttimesec<<", "
    << estimateposition.at(0)<<", "
    << estimateposition.at(1)<<", "
    << personvelocity.at(0)<<", "
    << personvelocity.at(1)<<", "
    << person_move_cnt<<", "
    <<person_move<<std::endl;

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
    m2.pose.position.x = estimateposition.at(0)-0.25;
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
        robotyawraw = yaw/M_PI*180;//deg
        
        if(robotyawraw<0)
        {
            robotyaw = 180+(180-(abs(robotyawraw)));
        }
        else
        {
            robotyaw = robotyawraw;
        }
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
            robot_move_cnt += 1;
        }
        else if ((abs(robotvelocity.at(0))>0.02)||(abs(robotvelocity.at(1))>0.02))
        {
            robot_move_cnt += 1;   
        }
        else
        {
            robot_move = 0;
            robot_move_cnt = 0;
        }
        
        if(robot_move_cnt>4)
        {
            robot_move=1;
        }
        
        lastcurrenttimevelocity  = currenttimesec;

        std::cout<<"robot_cnt:"<<robot_move_cnt<<std::endl;
        std::cout<<"robot_move:"<<robot_move<<std::endl; 
        std::cout<<"robotpose:"<<robotpose.at(0)<<","<<robotpose.at(1)<<std::endl;
        std::cout<<"robotyawraw:"<<robotyawraw<<std::endl;
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
        << robot_move_cnt<<", "
        <<robot_move<<std::endl;
        
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
        
        //headorientation[2] *= 1.4;
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

        headarrowangle = headorientation[2];

        headarrowtheta += robotyaw;
        if(headarrowtheta>=360)
            headarrowtheta-=360;
        
        /*
        headarrowangleraw = atan2(nose_end_point2D_drawtmp.at(0)-nose_tip[0],nose_end_point2D_drawtmp.at(1)-nose_tip[1])* 180 / PI;
        headarrowangle = headarrowangleraw;
        if(headarrowangleraw>90)
        {   
            headarrowangle = 90;
        }
        else if(headarrowangleraw<-90)
        {
            headarrowangle = -90;
        }

        headarrowtheta = headarrowangle+180+robotyaw;

        if(headarrowtheta>=360)
        headarrowtheta-=360;
        */
        headposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(headarrowtheta/180.0*M_PI);
        
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

void CombiDarknetOpenface::PublishObjectMarker(double measurementx, double measurementy)
{
    visualization_msgs::Marker objectmarker;

    objectmarker.header.stamp = ros::Time::now();
    objectmarker.header.frame_id = fixed_frame;
    objectmarker.pose.position.x = measurementx;
    objectmarker.pose.position.y = measurementy;
    objectmarker.lifetime = ros::Duration(1);
    objectmarker.scale.x = 0.2;
    objectmarker.scale.y = 0.2;
    objectmarker.scale.z = 0.01;
    objectmarker.type = objectmarker.CUBE;
    objectmarker.color.a = 0.75;
    objectmarker.color.b =  1.0;
    object_marker_pub.publish(objectmarker);
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

    activityscorefacedata2<<"cnt"<<","
    <<"time"<<","
    <<"notmeasurement_cnt"<<","
    <<"move_mode"<<","
    <<"robot_moving"<<std::endl;

    
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
    <<"currenttimesec"<<","
    <<"robot_move_cnt"<<","
    <<"robot_move"<<","
    <<"person_move_cnt"<<","
    <<"person_move"<<","
    <<"notmeasurement_cnt"<<","
    <<"pose_reset_cnt"<<","
    <<"pose_reset"<<","
    <<"move_mode"<<","
    <<"robot_moving"<<","
    <<"moving_cnt"<<std::endl; 

    personvelocitydata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"personpose_x"<<","
    <<"personpose_y"<<","
    <<"personvelocity_x"<<","
    <<"personvelocity_y"<<","
    <<"person_move_cnt"<<","
    <<"person_move"<<std::endl;

    robotvelocitydata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"robotpose_x"<<","
    <<"robotpose_y"<<","
    <<"robotvelocity_x"<<","
    <<"robotvelocity_y"<<","
    <<"robot_move_cnt"<<","
    <<"robot_move"<<std::endl;

    robotcmdveldata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"msg.linear.x"<<","
    <<"msg.linear.y"<<","
    <<"msg.angular.z"<<","
    <<"robot_move"<<std::endl;

    depthdata<<"darknet_cnt"<<","
    <<"time"<<","
    <<"distancetmp"<<","
    <<"distance"<<","
    <<"xc"<<","
    <<"yc"<<","
    <<"modify_distance_cnt"<<","
    <<"person_move_cnt"<<","
    <<"person_move"<<std::endl;

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