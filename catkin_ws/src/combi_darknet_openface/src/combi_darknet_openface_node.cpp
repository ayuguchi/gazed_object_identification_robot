#include "combi_darknet_openface_node.hpp"

#include <limits>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>
#include <combi_darknet_openface/GazeDetectionResult.h>
#include <combi_darknet_openface/DetectionResult.h>

#include "math_util.h"


CombiDarknetOpenface::CombiDarknetOpenface()
    : nh1("~"), class_names({"none"})
{
    ros_object_sub = nh1.subscribe("darknet_ros/bounding_boxes",1, &CombiDarknetOpenface::onRecognizedObject, this);
    ros_filtered_sub= nh1.subscribe("estimate_pos",1, &CombiDarknetOpenface::onPersonPositionEstimated, this);
    ros_face_sub = nh1.subscribe("faces",1, &CombiDarknetOpenface::onRecognizedFace, this);
    rgb_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &CombiDarknetOpenface::onRgbImageUpdated, this);
    depth_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &CombiDarknetOpenface::onDepthImageUpdated, this);
    camera_info_sub = nh1.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1, &CombiDarknetOpenface::onCameraInfoUpdated, this);
    measurement_pub = nh1.advertise<geometry_msgs::PoseStamped>("filter_measurement", 1);
    headpose_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_headpose_arrow", 1);
    origin_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_origin_marker", 1);
    person_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_person_arrow", 1);
    person_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_person_marker", 1);
    object_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_object_marker", 1);
    estimate_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_estimateperson_marker", 1);
    gaze_detection_pub = nh1.advertise<combi_darknet_openface::GazeDetectionResult>("gaze_detection_result", 1);
    cv::namedWindow("RGB image", CV_WINDOW_AUTOSIZE);
}


CombiDarknetOpenface::~CombiDarknetOpenface()
{
}


void CombiDarknetOpenface::onCameraInfoUpdated(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    this->camera_matrix = cv::Mat(3, 3, CV_64FC1);
    for(std::size_t i = 0; i < msg->K.size(); ++i)
    {
        this->camera_matrix.at<double>(i / 3, i % 3) = msg->K[i];
    }
    this->dist_coeffs = cv::Mat(msg->D.size(), 1, CV_64FC1);
    for(std::size_t i = 0; i < msg->D.size(); ++i)
    {
        this->dist_coeffs.at<double>(i, 0) = msg->D[i];
    }
    this->image_size.width = msg->width;
    this->image_size.height = msg->height;
}


void CombiDarknetOpenface::onRecognizedFace(const openface_ros::Faces::ConstPtr& msg)
{
    this->nose_tip_position_ptr.reset();
    this->chin_position_ptr.reset();
    this->left_eye_position_ptr.reset();
    this->right_eye_position_ptr.reset();
    this->left_mouth_position_ptr.reset();
    this->right_mouth_position_ptr.reset();
    this->nose_end_point2D_drawtmp.reset();

    for(const auto& detected_face : msg->faces)
    {
        if(detected_face.landmarks_2d.empty())
        {
            continue;
        }

        this->nose_tip_position_ptr.reset(new cv::Point2i(std::round(detected_face.landmarks_2d[33].x), std::round(detected_face.landmarks_2d[33].y)));
        this->chin_position_ptr.reset(new cv::Point2i(std::round(detected_face.landmarks_2d[8].x), std::round(detected_face.landmarks_2d[8].y)));
        this->left_eye_position_ptr.reset(new cv::Point2i(std::round(detected_face.landmarks_2d[36].x), std::round(detected_face.landmarks_2d[36].y)));
        this->right_eye_position_ptr.reset(new cv::Point2i(std::round(detected_face.landmarks_2d[45].x), std::round(detected_face.landmarks_2d[45].y)));
        this->left_mouth_position_ptr.reset(new cv::Point2i(std::round(detected_face.landmarks_2d[48].x), std::round(detected_face.landmarks_2d[48].y)));
        this->right_mouth_position_ptr.reset(new cv::Point2i(std::round(detected_face.landmarks_2d[54].x), std::round(detected_face.landmarks_2d[54].y)));

        this->updateExtrinsicParameters(
            *this->nose_tip_position_ptr,
            *this->chin_position_ptr,
            *this->left_eye_position_ptr,
            *this->right_eye_position_ptr,
            *this->left_mouth_position_ptr,
            *this->right_mouth_position_ptr
        );

        nose_end_point2D_drawtmp.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, this->TMP_DISTANCE))));

        cv::Mat rotation_matrix  = cv::Mat::zeros(3, 3, CV_64FC1); // Rotation in axis-angle form
        cv::Rodrigues(this->rotation_vector, rotation_matrix);
        cv::Mat proj_matrix = (cv::Mat_<double>(3,4) <<
            rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2), 0,
            rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2), 0,
            rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2), 0
        );
        cv::Mat head_translation_vector = this->translation_vector.clone();
        cv::Vec3d euler_angles;
        cv::decomposeProjectionMatrix(
            proj_matrix,
            this->camera_matrix,
            rotation_matrix,
            head_translation_vector,
            cv::noArray(),
            cv::noArray(),
            cv::noArray(),
            euler_angles
        );
        EulerAngles head_orientation = {
            euler_angles[2],  // roll
            euler_angles[0],  // pitch
            euler_angles[1]  // yaw
        };
        this->applyCacheIf(
            head_orientation,
            this->rotation_vector,
            this->translation_vector,
            !this->isInImageArea(*this->nose_end_point2D_drawtmp)
        );

        if(this->estimate_position_ptr && (!head_orientation.empty()))
        {
            double head_arrow_theta = calcHeadArrowAngle(head_orientation);
            this->publishHeadPoseArrow(*this->estimate_position_ptr, head_arrow_theta);
        }
    }
}


bool CombiDarknetOpenface::isInImageArea(const cv::Point2i& point) const
{
    return (point.x > 0) && (point.x < this->image_size.width) && (point.y > 0) && (point.y < this->image_size.height);
}


void CombiDarknetOpenface::updateExtrinsicParameters(const cv::Point2i& nose_tip_position, const cv::Point2i& chin_position, const cv::Point2i& left_eye_position, const cv::Point2i& right_eye_position, const cv::Point2i& left_mouth_position, const cv::Point2i& right_mouth_position)
{
    std::vector<cv::Point2f> image_points = {
        nose_tip_position,
        chin_position,
        left_eye_position,
        right_eye_position,
        left_mouth_position,
        right_mouth_position
    };

    const float face_model_scale = 0.225;
    std::vector<cv::Point3f> model_points = {
        cv::Point3f(0.0f, 0.0f, 0.0f),  // Nose tip
        cv::Point3f(0.0f, -330.0*face_model_scale, -65.0*face_model_scale),  // Chin
        cv::Point3f(-225.0*face_model_scale, 170.0*face_model_scale, -135.0*face_model_scale),  // Left eye left corner
        cv::Point3f(225.0*face_model_scale, 170.0*face_model_scale, -135.0*face_model_scale),  // Right eye right corner
        cv::Point3f(-150.0*face_model_scale, -150.0*face_model_scale, -125.0*face_model_scale),  // Left Mouth corner
        cv::Point3f(150.0*face_model_scale, -150.0*face_model_scale, -125.0*face_model_scale)  // Right mouth corner
    };

    this->updateExtrinsicParameters(model_points, image_points);
}


void CombiDarknetOpenface::updateExtrinsicParameters(const std::vector<cv::Point3f>& model_points, const std::vector<cv::Point2f>& image_points)
{
    this->rotation_vector  = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation in axis-angle form
    this->translation_vector = cv::Mat::zeros(3, 1, CV_64FC1);

    cv::solvePnP(model_points, image_points, this->camera_matrix, this->dist_coeffs, this->rotation_vector, this->translation_vector, CV_ITERATIVE);
}


void CombiDarknetOpenface::applyCacheIf(EulerAngles& head_orientation, cv::Mat& rotation_vector, cv::Mat& translation_vector, bool condition) const
{
    static bool initialized = false;
    static EulerAngles previous_head_orientation;
    static cv::Mat previous_rotation_vector;
    static cv::Mat previous_translation_vector;

    if(!initialized)
    {
        previous_head_orientation =  head_orientation;
        previous_rotation_vector = rotation_vector.clone();
        previous_translation_vector = translation_vector.clone();
        initialized = true;
    }

    if(condition)
    {
        head_orientation = previous_head_orientation;
        rotation_vector = previous_rotation_vector.clone();
        translation_vector = previous_translation_vector.clone();
    }
    else
    {
        previous_head_orientation = head_orientation;
        previous_rotation_vector = rotation_vector.clone();
        previous_translation_vector = translation_vector.clone();
    }
}


cv::Point2i CombiDarknetOpenface::getProjectedPoint(const cv::Point3f& point_3d) const
{
    std::vector<cv::Point3f> point_3d_list = {point_3d};
    std::vector<cv::Point2f> point_2d_list;
    cv::projectPoints(point_3d_list, this->rotation_vector, this->translation_vector, this->camera_matrix, this->dist_coeffs, point_2d_list);
    return cv::Point2i(std::round(point_2d_list[0].x), std::round(point_2d_list[0].y));
}


double CombiDarknetOpenface::calcHeadArrowAngle(const EulerAngles& head_orientation) const
{
    double ret_val = 0.0;
    if(0 <= head_orientation[2] && head_orientation[2] < 90)
    {
        ret_val = 180 - head_orientation[2];
    }
    else if(-90 <= head_orientation[2] && head_orientation[2]<0)
    {
        ret_val = std::abs(head_orientation[2]) + 180;
    }
    else
    {
        ret_val = head_orientation[2];
    }

    ret_val += this->robotyaw;
    if(ret_val>=360)
    {
        ret_val-=360;
    }
    return ret_val;
}


void CombiDarknetOpenface::onRecognizedObject(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg )
{
    std::vector<std::string> current_frame_class_names = {"none"};
    std::vector<cv::Rect> current_frame_object_boxes = {cv::Rect(0, 0, 0, 0)};

    this->nearest_object_index = 0;
    this->person_box.reset();

    for(const auto& detected_object :  msg->boundingBoxes)
    {
        current_frame_class_names.push_back(detected_object.Class);
        current_frame_object_boxes.push_back(cv::Rect(cv::Point(detected_object.xmin, detected_object.ymin), cv::Point(detected_object.xmax, detected_object.ymax)));
        if(detected_object.Class=="person")
        {
            this->person_box.reset(new cv::Rect(cv::Point(detected_object.xmin, detected_object.ymin), cv::Point(detected_object.xmax, detected_object.ymax)));
        }
        if(std::find(this->class_names.begin(),this->class_names.end(),detected_object.Class) == this->class_names.end())
        {
            this->class_names.push_back(detected_object.Class);
        }
    }
    this->object_boxes.resize(this->class_names.size());
    std::fill(this->object_boxes.begin(), this->object_boxes.end(), cv::Rect(0, 0, 0, 0));
    for(std::size_t i = 0; i < current_frame_class_names.size(); ++i)
    {
        auto box_index = std::distance(
            this->class_names.begin(),
            std::find(this->class_names.begin(), this->class_names.end(), current_frame_class_names[i])
        );
        this->object_boxes[box_index] = current_frame_object_boxes[i];
    }
    this->object_centers.resize(this->object_boxes.size());
    for(std::size_t i = 0; i < this->class_names.size(); ++i)
    {
        if(this->isIgnoredObjectClass(this->class_names[i]))
        {
            this->object_centers[i] = this->invalidPoint();
        }
        else
        {
            this->object_centers[i] = (this->object_boxes[i].tl() + this->object_boxes[i].br()) / 2;
        }
    }
    if(this->person_box && this->person_moving_state == PersonMovingState::Stopping)
    {
        this->calculateTimeUse();
    }
    else{
        this->calculateTimeUseOutofView();
    }
    this->publishGazeDetectionResult(msg->header);
}


bool CombiDarknetOpenface::isIgnoredObjectClass(const std::string& class_name) const
{
    return std::find(this->ignore_object_list.begin(), this->ignore_object_list.end(), class_name) != this->ignore_object_list.end();
}


void CombiDarknetOpenface::calculateTimeUse()
{
    this->nearest_object_index = 0;
    this->nearest_gaze_position_ptr.reset();
    if(this->nose_tip_position_ptr && this->nose_end_point2D_drawtmp)
    {
        double nearest_object_distance;
        this->nearest_gaze_position_ptr.reset(new cv::Point2i(this->invalidPoint()));
        std::tie(this->nearest_object_index, nearest_object_distance, *this->nearest_gaze_position_ptr) = this->findGazedObject(*this->nose_tip_position_ptr, *this->nose_end_point2D_drawtmp, this->object_centers);
        if(this->nearest_gaze_position_ptr && !this->isInvalidPoint(*this->nearest_gaze_position_ptr))
        {
            if(nearest_object_distance > this->gaze_assigned_thresh)
            {
                this->nearest_object_index = 0;
            }
            if(this->object_distance_cache.value() > this->person_distance_cache.value())
            {
                this->nearest_object_index = 0;
            }
        }
    }
}


std::tuple<std::size_t, double, cv::Point2i> CombiDarknetOpenface::findGazedObject(const cv::Point2i& nose_tip_point, const cv::Point2i& nose_end_point, const std::vector<cv::Point2i>& object_centers, float angle_distance_thresh) const
{
    double nose_angle = math_util::radToDeg(math_util::atan2(nose_end_point - nose_tip_point));
    std::size_t min_object_index = 0;
    float min_object_distance = std::numeric_limits<float>::infinity();
    cv::Point2i min_gaze_position = this->invalidPoint();
    for(double distance = this->search_distance_step; distance <= this->search_distance_range; distance += this->search_distance_step)
    {
        cv::Point2i target_point = this->getProjectedPoint(cv::Point3d(0,0,distance));
        double target_point_angle = math_util::radToDeg(math_util::atan2(target_point - nose_tip_point));
        if(abs(target_point_angle - nose_angle) > angle_distance_thresh)
        {
            continue;
        }
        std::vector<float> nose_end_distances;
        for(const auto& center_position : object_centers)
        {
            if(this->isInvalidPoint(center_position))
            {
                nose_end_distances.push_back(std::numeric_limits<float>::infinity());
                continue;
            }
            nose_end_distances.push_back(cv::norm(center_position - target_point));
        }
        if(std::count(nose_end_distances.begin(), nose_end_distances.end(), std::numeric_limits<float>::infinity()) == nose_end_distances.size())
        {
            continue;
        }
        auto min_element_iter = std::min_element(std::begin(nose_end_distances), std::end(nose_end_distances));
        if(*min_element_iter < min_object_distance)
        {
            min_object_index = std::distance(nose_end_distances.begin(), min_element_iter);
            min_object_distance = *min_element_iter;
            min_gaze_position = target_point;

        }
    }
    return std::forward_as_tuple(min_object_index, min_object_distance, min_gaze_position);
}


void CombiDarknetOpenface::calculateTimeUseOutofView()
{
    this->nearest_object_index = 0;
    this->nearest_gaze_position_ptr.reset();
}


void CombiDarknetOpenface::onRgbImageUpdated(const sensor_msgs::ImageConstPtr& msg) const
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << ex.what());
        return;
    }
    if(this->nose_tip_position_ptr && this->isInImageArea(*this->nose_tip_position_ptr))
    {
        cv::circle(cv_ptr->image, *nose_tip_position_ptr, 5, cv::Scalar(0,0,255), 5);
    }
    if(this->nose_end_point2D_drawtmp && this->isInImageArea(*this->nose_end_point2D_drawtmp))
    {
        cv::circle(cv_ptr->image, *nose_end_point2D_drawtmp, 5, cv::Scalar(0,0,255), 5);
    }
    if(this->nose_tip_position_ptr && this->nose_end_point2D_drawtmp && this->isInImageArea(*this->nose_tip_position_ptr) && this->isInImageArea(*this->nose_end_point2D_drawtmp))
    {
        cv::line(cv_ptr->image, *this->nose_tip_position_ptr, *this->nose_end_point2D_drawtmp, cv::Scalar(0, 255, 0), 2);
    }
    if(this->nearest_gaze_position_ptr)
    {
        cv::circle(cv_ptr->image, *nearest_gaze_position_ptr, 7, cv::Scalar(0,0,255), 5);
    }
    cv::imshow("RGB image", cv_ptr->image);
    cv::waitKey(10);
}


void CombiDarknetOpenface::onDepthImageUpdated(const sensor_msgs::ImageConstPtr& msg)
{
    if(!this->person_box && !this->hasGazedObject())
    {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& ex)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << ex.what());
        return;
    }
    if(this->person_box)
    {
        cv::Rect person_area = nose_tip_position_ptr ?
            cv::Rect(nose_tip_position_ptr->x-1, nose_tip_position_ptr->y-1, 3, 3) :
            cv::Rect((this->person_box->tl() + this->person_box->br()) / 2, cv::Size(3, 3));
        double current_person_distance = this->getDepthValue(cv_ptr->image, person_area);
        this->person_distance_cache.update(current_person_distance);;
        double person_angle = this->angle_of_view / 2 - (static_cast<double>((this->person_box->tl().x + this->person_box->br().x) / 2) / this->image_size.width) * this->angle_of_view;
        cv::Point2d person_position = this->getPositionInGlobal(
            cv::Point2d(0, 0),
            robotyaw,
            this->person_distance_cache.value(),
            person_angle
        );
        this->publishPersonMeasurement(person_position, this->estimate_position_ptr);
        this->publishPersonMarker(person_angle, person_position);
    }

    if(this->hasGazedObject())
    {
        cv::Rect object_area = cv::Rect((this->getGazedObjectBox().tl() + this->getGazedObjectBox().br()) / 2, cv::Size(3, 3));
        double current_object_distance = this->getDepthValue(cv_ptr->image, object_area);
        this->object_distance_cache.update(current_object_distance);
        double object_angle = this->angle_of_view / 2 - (static_cast<double>((this->getGazedObjectBox().tl().x + this->getGazedObjectBox().br().x) / 2) / this->image_size.width) * this->angle_of_view;
        cv::Point2d object_position = this->getPositionInGlobal(
            cv::Point2d(0, 0),
            robotyaw,
            this->object_distance_cache.value(),
            object_angle
        );
        this->publishObjectMarker(object_position);
    }
}


cv::Point2d CombiDarknetOpenface::getPositionInGlobal(const cv::Point2d& robot_trans, double robot_yaw, double distance, double angle)const
{
    Eigen::Matrix3d A;
    A << cos(math_util::degToRad(robot_yaw)), sin(math_util::degToRad(robot_yaw)), robot_trans.x,
        sin(math_util::degToRad(robot_yaw)), cos(math_util::degToRad(robot_yaw)), robot_trans.y,
        0,  0,   1;
    Eigen::Matrix3d B;
    B << cos(math_util::degToRad(angle)), sin(math_util::degToRad(angle)), distance * cos(math_util::degToRad(angle)),
        sin(math_util::degToRad(angle)), cos(math_util::degToRad(angle)), distance * sin(math_util::degToRad(angle)),
        0,  0,   1;
    Eigen::Matrix3d T = A * B;
    return cv::Point2d(T(0,2), T(1,2));
}


double CombiDarknetOpenface::getDepthValue(const cv::Mat& depth_image, const cv::Rect& area)const
{
    cv::Mat conv_image;
    depth_image.convertTo(conv_image, CV_64F);
    double ret_val = 0.0;
    for(std::size_t i = area.tl().y; i < area.br().y; ++i)
    {
        for(std::size_t j = area.tl().x; j < area.br().x; ++j)
        {
            if(conv_image.at<double>(i, j) > 0.0)
            {
                ret_val = static_cast<double>(conv_image.at<double>(i, j));
            }
        }
    }
    return ret_val;
}


void CombiDarknetOpenface::onPersonPositionEstimated(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    static std::unique_ptr<cv::Point2d> last_estimate_position_ptr;
    if(!last_estimate_position_ptr)
    {
        last_estimate_position_ptr.reset(new cv::Point2d(msg->pose.position.x, msg->pose.position.y));
    }
    this->estimate_position_ptr.reset(new cv::Point2d(msg->pose.position.x, msg->pose.position.y));
    this->person_velocity_ptr.reset(new cv::Vec2d((*this->estimate_position_ptr) - (*last_estimate_position_ptr)));
    last_estimate_position_ptr.reset(new cv::Point2d(*this->estimate_position_ptr));
    if(!this->person_box)
    {
        this->person_moving_state = PersonMovingState::Unrecognized;
    }
    else if((abs((*this->person_velocity_ptr)[0]) > 0.04 && abs((*this->person_velocity_ptr)[1]) > 0.04) ||
            (abs((*this->person_velocity_ptr)[0]) > 0.06 || abs((*this->person_velocity_ptr)[1]) > 0.06))
    {
        person_move_cnt += 1;
    }
    else
    {
        this->person_moving_state = PersonMovingState::Stopping;
        person_move_cnt = 0;
    }
    if(person_move_cnt>4)
    {
        this->person_moving_state = PersonMovingState::Moving;
    }

    this->publishEstimatedPersonPositionMarker(*this->estimate_position_ptr);
}


void CombiDarknetOpenface::publishEstimatedPersonPositionMarker(const cv::Point2d& position) const
{
    visualization_msgs::Marker estimate_person_marker;
    estimate_person_marker.header.stamp = ros::Time::now();
    estimate_person_marker.header.frame_id = this->FIXED_FRAME;
    estimate_person_marker.pose.position.x = position.x;
    estimate_person_marker.pose.position.y = position.y;
    estimate_person_marker.lifetime = ros::Duration(0);
    estimate_person_marker.scale.x = 0.2;
    estimate_person_marker.scale.y = 0.2;
    estimate_person_marker.scale.z = 0.01;
    estimate_person_marker.type = visualization_msgs::Marker::SPHERE;
    estimate_person_marker.color.a = 0.75;
    estimate_person_marker.color.r = 1.0;
    estimate_person_marker.color.g = 0;
    estimate_person_marker.color.b = 1.0;
    this->estimate_marker_pub.publish(estimate_person_marker);
}


void CombiDarknetOpenface::publishPersonMeasurement(const cv::Point2d& position, const std::unique_ptr<cv::Point2d>& estimated_position) const
{
    static int kf_failed_cnt = 0;
    geometry_msgs::PoseStamped input_pose;
    input_pose.pose.position.x = position.x;
    input_pose.pose.position.y = position.y;
    if(!estimated_position)
    {
        input_pose.pose.position.z = 1;
        measurement_pub.publish(input_pose);
    }
    else
    {
        double dist_error = cv::norm(position - *estimated_position);
        if(dist_error<1.2)
        {
            kf_failed_cnt = 0;
            input_pose.pose.position.z = 0;
            measurement_pub.publish(input_pose);
        }
        else
        {
            if(kf_failed_cnt == 5)
            {
                input_pose.pose.position.z = 1;
                measurement_pub.publish(input_pose);
                kf_failed_cnt = 0;
            }
            else
            {
                kf_failed_cnt += 1;
            }
        }
    }
}


void CombiDarknetOpenface::publishHeadPoseArrow(const cv::Point2d& position, double head_arrow_angle_deg) const
{
    visualization_msgs::Marker head_pose_arrow;
    head_pose_arrow.header.frame_id = FIXED_FRAME;
    head_pose_arrow.header.stamp = ros::Time::now();
    head_pose_arrow.ns = "basic_shapes";
    head_pose_arrow.type = visualization_msgs::Marker::ARROW;
    head_pose_arrow.action = visualization_msgs::Marker::ADD;
    head_pose_arrow.pose.position.x = position.x;
    head_pose_arrow.pose.position.y = position.y;
    head_pose_arrow.pose.orientation = tf::createQuaternionMsgFromYaw(math_util::degToRad(head_arrow_angle_deg));
    head_pose_arrow.scale.x = 0.3;
    head_pose_arrow.scale.y = 0.1;
    head_pose_arrow.scale.z = 0.1;
    head_pose_arrow.color.r = 0.0f;
    head_pose_arrow.color.g = 0.0f;
    head_pose_arrow.color.b = 1.0f;
    head_pose_arrow.color.a = 1.0f;
    head_pose_arrow.lifetime = ros::Duration();
    headpose_arrow_pub.publish(head_pose_arrow);
}


void CombiDarknetOpenface::publishPersonMarker(double theta, const cv::Point2d& position) const
{
    visualization_msgs::Marker person_arrow;
    person_arrow.header.frame_id = FIXED_FRAME;
    person_arrow.header.stamp = ros::Time::now();
    person_arrow.ns = "basic_shapes";
    person_arrow.type = visualization_msgs::Marker::ARROW;
    person_arrow.action = visualization_msgs::Marker::ADD;
    person_arrow.pose.position.x = position.x;
    person_arrow.pose.position.y = position.y;
    person_arrow.pose.orientation = tf::createQuaternionMsgFromYaw(math_util::degToRad(theta));
    person_arrow.scale.x = 0.3;
    person_arrow.scale.y = 0.1;
    person_arrow.scale.z = 0.1;
    person_arrow.color.r = 1.0f;
    person_arrow.color.g = 0.0f;
    person_arrow.color.b = 0.0f;
    person_arrow.color.a = 1.0f;
    person_arrow.lifetime = ros::Duration();
    person_arrow_pub.publish(person_arrow);

    visualization_msgs::Marker person_marker;
    person_marker.header.stamp = ros::Time::now();
    person_marker.header.frame_id = FIXED_FRAME;
    person_marker.pose.position.x = position.x;
    person_marker.pose.position.y = position.y;
    person_marker.lifetime = ros::Duration();
    person_marker.scale.x = 0.2;
    person_marker.scale.y = 0.2;
    person_marker.scale.z = 0.01;
    person_marker.type = visualization_msgs::Marker::CUBE;
    person_marker.color.a = 0.75;
    person_marker.color.g =  1.0;
    person_marker_pub.publish(person_marker);

    visualization_msgs::Marker origin_marker;
    origin_marker.header.stamp = ros::Time::now();
    origin_marker.header.frame_id = FIXED_FRAME;
    origin_marker.pose.position.x = 0.0;
    origin_marker.pose.position.y = 0.0;
    origin_marker.lifetime = ros::Duration();
    origin_marker.scale.x = 0.1;
    origin_marker.scale.y = 0.1;
    origin_marker.scale.z = 0.1;
    origin_marker.color.a = 1.0f;
    origin_marker.color.r = 1.0;
    origin_marker.color.g = 1.0;
    origin_marker.color.b = 1.0;
    origin_marker.type = visualization_msgs::Marker::SPHERE;
    origin_marker_pub.publish(origin_marker);
}


void CombiDarknetOpenface::publishObjectMarker(const cv::Point2d& position) const
{
    visualization_msgs::Marker object_marker;
    object_marker.header.stamp = ros::Time::now();
    object_marker.header.frame_id = FIXED_FRAME;
    object_marker.pose.position.x = position.x;
    object_marker.pose.position.y = position.y;
    object_marker.lifetime = ros::Duration(1);
    object_marker.scale.x = 0.2;
    object_marker.scale.y = 0.2;
    object_marker.scale.z = 0.01;
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.color.a = 0.75;
    object_marker.color.b =  1.0;
    object_marker_pub.publish(object_marker);
}


void CombiDarknetOpenface::publishGazeDetectionResult(const std_msgs::Header& header) const
{
    if(!this->nose_tip_position_ptr || !this->nose_end_point2D_drawtmp)
    {
        return;
    }
    combi_darknet_openface::GazeDetectionResult result;
    result.header = header;
    result.face.x = this->nose_tip_position_ptr->x;
    result.face.y = this->nose_tip_position_ptr->y;
    result.face.theta = math_util::atan2(*this->nose_end_point2D_drawtmp - *this->nose_tip_position_ptr);
    result.gazedObjectIndex = this->nearest_object_index == 0 ? combi_darknet_openface::GazeDetectionResult::OUT_OF_VIEW : this->nearest_object_index - 1;
    for(std::size_t i = 1; i < this->class_names.size(); ++i)
    {
        combi_darknet_openface::DetectionResult d;
        d.Class = this->class_names[i];
        d.xmin = this->object_boxes[i].tl().x;
        d.ymin = this->object_boxes[i].tl().y;
        d.xmax = this->object_boxes[i].br().x;
        d.ymax = this->object_boxes[i].br().y;
        result.detectionResults.push_back(d);
    }
    this->gaze_detection_pub.publish(result);
}


cv::Point2i CombiDarknetOpenface::invalidPoint()const
{
    return cv::Point2i(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
}


bool CombiDarknetOpenface::isInvalidPoint(const cv::Point2i& p)const
{
    return p.x == std::numeric_limits<int>::max() && p.y == std::numeric_limits<int>::max();
}


bool CombiDarknetOpenface::hasGazedObject() const
{
    return this->nearest_object_index != 0;
}


cv::Rect CombiDarknetOpenface::getGazedObjectBox() const
{
    return this->object_boxes.at(this->nearest_object_index);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "combi_darknet_openface");
    volatile CombiDarknetOpenface pt;
    ros::spin();
    return 0;
}