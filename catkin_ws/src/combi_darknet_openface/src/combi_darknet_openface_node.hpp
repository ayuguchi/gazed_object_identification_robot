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