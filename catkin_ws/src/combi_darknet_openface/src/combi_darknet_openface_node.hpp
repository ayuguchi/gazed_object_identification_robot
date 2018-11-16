#include <ros/ros.h>
#include <memory>
#include <algorithm>
#include <vector>
#include <tuple>

#ifndef OPENCV_TRAITS_ENABLE_DEPRECATED
    #define OPENCV_TRAITS_ENABLE_DEPRECATED
#endif
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <openface_ros/Face.h>
#include <openface_ros/Faces.h>

#include "CachedValue.h"


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
    void calculateTimeUse();
    void calculateTimeUseOutofView();
    void onRecognizedFace(const openface_ros::Faces::ConstPtr& msg );
    void onRecognizedObject(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg );
    void onRgbImageUpdated(const sensor_msgs::ImageConstPtr& msg) const;//face_feature
    void onDepthImageUpdated(const sensor_msgs::ImageConstPtr& msg);
    void onPersonPositionEstimated(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void onCameraInfoUpdated(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void publishPersonMeasurement(const cv::Point2d& position, const std::unique_ptr<cv::Point2d>& estimated_position) const;
    void publishPersonMarker(double theta, const cv::Point2d& position) const;
    void publishObjectMarker(const cv::Point2d& position) const;
    void publishHeadPoseArrow(const cv::Point2d& position, double head_arrow_angle_deg) const;
    void publishEstimatedPersonPositionMarker(const cv::Point2d& position, int num_tracking_frame) const;
    //void changeViewPoint(double currenttimesec);

private:
    const std::string FIXED_FRAME = "map";
    const double TMP_DISTANCE = 250.0;
    const double search_distances[3] = {350.0, 500.0, 650.0};
    const std::vector<std::string> ignore_object_list = {"person", "dining table", "bench", "oven", "refrigerator"};
    const int search_distance_step = 10;
    const int search_distance_range = 650;
    const double gaze_assigned_thresh = 65.0;
    const double angle_of_view = 58.0;
    const std::size_t pose_reset_count = 50;

    ros::NodeHandle nh1;

    ros::Subscriber ros_object_sub;    
    ros::Subscriber ros_face_sub;
    ros::Subscriber rgb_img_sub;
    ros::Subscriber depth_img_sub;
    ros::Subscriber camera_info_sub;
    ros::Subscriber ros_filtered_sub;

    ros::Publisher measurement_pub;
    ros::Publisher headpose_arrow_pub;
    ros::Publisher origin_marker_pub;
    ros::Publisher person_arrow_pub;
    ros::Publisher person_marker_pub;
    ros::Publisher object_marker_pub;
    ros::Publisher estimate_marker_pub;
    ros::Publisher cnt_text_pub;
    ros::Publisher destination_marker_pub;

    std::vector<std::string> class_names;
    std::vector<cv::Rect> object_boxes;
    std::vector<cv::Point2i> object_centers;
    std::vector<cv::Point2i> last_object_centers;

    std::size_t nearest_object_index = 0;
    std::vector<int>activityscoreface;
    std::vector<int>activityscoreobject;
    
    std::unique_ptr<cv::Point2d> estimate_position_ptr;
    std::unique_ptr<cv::Vec2d> person_velocity_ptr;

    std::vector<double>robotpose;
    double robotyaw;

    std::unique_ptr<cv::Rect> person_box;

    std::unique_ptr<cv::Point2i> nose_tip_position_ptr;
    std::unique_ptr<cv::Point2i> chin_position_ptr;
    std::unique_ptr<cv::Point2i> left_eye_position_ptr;
    std::unique_ptr<cv::Point2i> right_eye_position_ptr;
    std::unique_ptr<cv::Point2i> left_mouth_position_ptr;
    std::unique_ptr<cv::Point2i> right_mouth_position_ptr;

    std::unique_ptr<cv::Point2i> nose_end_point2D_drawtmp;
    std::unique_ptr<cv::Point2i> nose_end_point2D_draw;
    std::unique_ptr<cv::Point2i> nose_end_point2D_draw2;
    std::unique_ptr<cv::Point2i> nose_end_point2D_draw3;
    std::unique_ptr<cv::Point2i> nearest_gaze_position_ptr;

    PersonMovingState person_moving_state = PersonMovingState::Moving;//0:stoping,1:moving

    std::size_t darknet_cnt = 0;
    std::size_t robotpose_cnt = 0;
    std::size_t person_move_cnt = 0;
    bool after_flag = false;
    int robot_moving = 0;
    int pose_reset = 0;
    std::size_t pose_reset_cnt = 0;

    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 541.20870062659242, 0, 318.78756964392710, 0 ,  540.20435182225424, 236.43301053278904, 0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(4,1) << 0.06569569924719, -0.25862424608946, 0.00010394071172, -0.00024019257963);
    cv::Size image_size = cv::Size(640, 480);
    // Extrinsic Parameters
    cv::Mat rotation_vector;
    cv::Mat translation_vector;

    CachedValue<double> person_distance_cache = CachedValue<double>(0.0);
    CachedValue<double> object_distance_cache = CachedValue<double>(0.0);

    void updateExtrinsicParameters(const cv::Point2i& nose_tip_position, const cv::Point2i& chin_position, const cv::Point2i& left_eye_position, const cv::Point2i& right_eye_position, const cv::Point2i& left_mouth_position, const cv::Point2i& right_mouth_position);
    void updateExtrinsicParameters(const std::vector<cv::Point3f>& model_points, const std::vector<cv::Point2f>& image_points);
    void applyCacheIf(EulerAngles& head_orientation, cv::Mat& rotation_vector, cv::Mat& translation_vector, bool condition) const;
    cv::Point2i getProjectedPoint(const cv::Point3f& point_3d) const;
    double calcHeadArrowAngle(const EulerAngles& head_orientation) const;
    bool isInImageArea(const cv::Point2i& point) const;
    bool isIgnoredObjectClass(const std::string& class_name) const;
    /// \return index, distance, gaze_position
    std::tuple<std::size_t, double, cv::Point2i> getNearestObject(const cv::Point2i& nose_tip_point, const cv::Point2i& nose_end_point, const std::vector<cv::Point2i>& object_centers, float angle_distance_thresh=10.0f) const;
    cv::Point2i invalidPoint()const;
    bool isInvalidPoint(const cv::Point2i& p)const;
    bool hasFocusedObject() const;
    cv::Rect getFocusedObjectBox() const;
    double getDepthValue(const cv::Mat& depth_image, const cv::Rect& area)const;
    cv::Point2d getPositionInGlobal(const cv::Point2d& robot_trans, double robot_yaw, double distance, double angle)const;
};