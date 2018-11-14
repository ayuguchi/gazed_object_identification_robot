#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <strings.h>
#include <iostream>
#include <memory>

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
#include <openface_ros/Face.h>
#include <openface_ros/Faces.h>

#include <boost/foreach.hpp>
#include <algorithm>

#include "CachedValue.h"

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

int frame_num = 0;
int modify_yaw_cnt = 0;
int modify_distance_cnt = 0;

int rgb_cnt = 0;
int depth_cnt = 0;
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
std::ofstream alltimerecord("alltimerecord.csv", std::ios::trunc);
std::ofstream noseobjecttheta("noseobjecttheta.csv", std::ios::trunc);

class CombiDarknetOpenface
{
public:
    typedef std::vector<double> EulerAngles;
    enum class PersonMovingState{
        Stopping = 0,
        Moving = 1,
        Unrecognized = 2
    };
    CombiDarknetOpenface(ros::NodeHandle nh);
    ~CombiDarknetOpenface();
    void onRecognizedFace(const openface_ros::Faces::ConstPtr& msg );
    void calculateTimeUse(double currenttimesec);
    void linearLine(double x1,double x2,double y1,double y2,double* a,double* b );
    void calculateTimeUseOutofView(double currenttimesec);
    void onRecognizedObject(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void onRgbImageUpdated(const sensor_msgs::ImageConstPtr& msg);//face_feature
    void onDepthImageUpdated(const sensor_msgs::ImageConstPtr& msg);
    void onPersonPositionEstimated(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void onRobotPoseUpdated(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publishPersonMeasurement(double measurement_x, double measurement_y, const std::unique_ptr<cv::Point2d>& estimated_position) const;
    void publishPersonMarker(double theta, double measurement_x, double measurement_y) const;
    void publishObjectMarker(double measurement_x, double measurement_y) const;
    void publishHeadPoseArrow(const cv::Point2d& position, double head_arrow_angle_deg) const;
    void publishEstimatedPersonPositionMarker(const cv::Point2d& position, int num_tracking_frame) const;
    void changeViewPoint(double currenttimesec);

private:
    const std::string FIXED_FRAME = "map";
    const cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 541.20870062659242, 0, 318.78756964392710, 0 ,  540.20435182225424, 236.43301053278904, 0, 0, 1);
    const cv::Mat dist_coeffs = (cv::Mat_<double>(4,1) << 0.06569569924719, -0.25862424608946, 0.00010394071172, -0.00024019257963);
    ros::Subscriber ros_object_sub;
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

    ros::Publisher capture_cnt_pub;

    ros::NodeHandle nh1;

    std::vector<std::string>classnames;
    std::vector<std::string>classname;
    //
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

    std::unique_ptr<cv::Point2i> box_min_position_ptr;    
    std::unique_ptr<cv::Point2i> box_max_position_ptr;
    std::unique_ptr<cv::Point2i> box2_min_position_ptr;    
    std::unique_ptr<cv::Point2i> box2_max_position_ptr;
    std::unique_ptr<cv::Point2i> box_center_position_ptr;
    std::unique_ptr<cv::Point2i> last_box_center_position_ptr;

    int maxmoveindex;
    int noseendminindex;
    int mindistanceindex;
    std::vector<int>activityscoreface;
    std::vector<int>activityscoreobject;
    std::vector<float>timerecordface;

    //depth
    double noseobjectmindist;
    double persondepthdist;
    
    //
    std::unique_ptr<cv::Point2d> estimate_position_ptr;
    std::unique_ptr<cv::Vec2d> person_velocity_ptr;

    //
    std::vector<double>robotpose;
    double robotyaw;
    double robotyawraw;
    geometry_msgs::PoseStamped robotoriginpose;
    std::vector<double>lastrobotpose;
    std::vector<double>robotvelocity;

	std::unique_ptr<cv::Point2f> robot_position_ptr;    
    std::unique_ptr<cv::Point2f> last_robot_position_ptr;
    std::unique_ptr<cv::Point2f> robot_velocity_ptr;

    std::unique_ptr<cv::Rect> person_box;
    std::vector<int>detectedobjectbox; 

    std::unique_ptr<cv::Point2i> nose_tip_position_ptr;
    std::unique_ptr<cv::Point2i> chin_position_ptr;
    std::unique_ptr<cv::Point2i> left_eye_position_ptr;
    std::unique_ptr<cv::Point2i> right_eye_position_ptr;
    std::unique_ptr<cv::Point2i> left_mouth_position_ptr;
    std::unique_ptr<cv::Point2i> right_mouth_position_ptr;
    double head_arrow_theta;

    double headarrowangleraw;
    double head_arrow_angle;
    
    double headrobottheta;

    std::unique_ptr<cv::Point2i> nose_end_point2D_drawtmp;
    std::unique_ptr<cv::Point2i> nose_end_point2D_draw;
    std::unique_ptr<cv::Point2i> nose_end_point2D_draw2;
    std::unique_ptr<cv::Point2i> nose_end_point2D_draw3;
    std::vector<int>nose_end_point2D_drawmin;    

    int darknet_cnt = 0;
    PersonMovingState person_moving_state = PersonMovingState::Moving;//0:stoping,1:moving

    // Extrinsic Parameters
    cv::Mat rotation_vector;
    cv::Mat translation_vector;

    std::vector<double>lastrotation_value;   
    std::vector<double>lasttranslation_value;   

    CachedValue<double> person_distance_cache = CachedValue<double>(0.0);
    CachedValue<double> object_distance_cache = CachedValue<double>(0.0);

    void updateExtrinsicParameters(const cv::Point2i& nose_tip_position, const cv::Point2i& chin_position, const cv::Point2i& left_eye_position, const cv::Point2i& right_eye_position, const cv::Point2i& left_mouth_position, const cv::Point2i& right_mouth_position);
    void updateExtrinsicParameters(const std::vector<cv::Point3f>& model_points, const std::vector<cv::Point2f>& image_points);
    void modifyHeadOrientation(EulerAngles& head_orientation);
    cv::Point2i getProjectedPoint(const cv::Point3f& point_3d) const;
    double calcHeadArrowAngle(const EulerAngles& head_orientation) const;
};