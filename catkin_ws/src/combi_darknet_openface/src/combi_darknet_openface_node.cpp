#include "combi_darknet_openface_node.hpp"

#include <iostream>
#include <limits>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>

#include "math_util.h"

CombiDarknetOpenface::CombiDarknetOpenface(ros::NodeHandle nh)
    : nh1(nh), class_names({"none"})
{
    ros_object_sub = nh1.subscribe("darknet_ros/bounding_boxes",1, &CombiDarknetOpenface::onRecognizedObject, this);

    ros_face_sub = nh1.subscribe("faces",1, &CombiDarknetOpenface::onRecognizedFace, this);
    rgb_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &CombiDarknetOpenface::onRgbImageUpdated, this);
    depth_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &CombiDarknetOpenface::onDepthImageUpdated, this);
    camera_info_sub = nh1.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1, &CombiDarknetOpenface::onCameraInfoUpdated, this);

    ros_filtered_sub= nh1.subscribe("estimate_pos",1, &CombiDarknetOpenface::onPersonPositionEstimated, this);
    measurement_pub = nh1.advertise<geometry_msgs::PoseStamped>("filter_measurement", 1);

    headpose_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_headpose_arrow", 1);

    origin_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_origin_marker", 1);
    person_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_person_arrow", 1);
    person_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_person_marker", 1);
    object_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_object_marker", 1);

    estimate_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_estimateperson_marker", 1);
    cnt_text_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_cnt_txt", 1);

    destination_marker_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_destination_marker", 1);

    capture_cnt_pub = nh1.advertise<std_msgs::Int16>("/image_capture_cnt", 1);
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

        nose_end_point2D_drawtmp.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, TmpDistance))));

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

        nose_end_point2D_draw.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, FirstDistance))));
        nose_end_point2D_draw2.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, SecondDistance))));
        nose_end_point2D_draw3.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, ThirdDistance))));

        if(this->estimate_position_ptr && (!head_orientation.empty()))
        {
            this->head_arrow_theta = calcHeadArrowAngle(head_orientation);
            this->publishHeadPoseArrow(*this->estimate_position_ptr, this->head_arrow_theta);
            this->head_arrow_angle = head_orientation[2];
        }
        // headposedata << time_from_run_sec << ", "
        //     << detected_face.head_pose.position.x << ", "
        //     << detected_face.head_pose.position.y << ", "
        //     << detected_face.head_pose.position.z << ","
        //     << head_orientation[0] << ", "
        //     << head_orientation[1] << ", "
        //     << head_orientation[2] << std::endl;
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
    static ros::Time firsttime = ros::Time::now();
    static double previous_time_sec = std::numeric_limits<double>::max();
    ros::Time nowtime = ros::Time::now();
    if(previous_time_sec == std::numeric_limits<double>::max())
    {
        previous_time_sec = nowtime.toSec() - firsttime.toSec();
    }
    double current_time_sec = nowtime.toSec() - firsttime.toSec();

    std::vector<std::string> current_frame_class_names = {"none"};
    std::vector<cv::Rect> current_frame_object_boxes = {cv::Rect(0, 0, 0, 0)};

    activityscoreface.clear();
    activityscoreobject.clear();
    this->most_active_object_index = 0;
    this->nearest_object_index = 0;
    this->person_box.reset();
    darknet_cnt += 1;

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

    activityscoreface.resize(this->class_names.size());
    activityscoreobject.resize(this->class_names.size());
    fill(activityscoreface.begin(), activityscoreface.end(),0);
    fill(activityscoreobject.begin(), activityscoreobject.end(),0);

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
            this->object_centers[i] = cv::Point2i(0, 0);
        }
        else
        {
            this->object_centers[i] = (this->object_boxes[i].tl() + this->object_boxes[i].br()) / 2;
        }
    }

    if(this->person_box && this->person_moving_state == PersonMovingState::Stopping &&(notmeasurement_cnt<RobotMoveCount)&&(!robot_moving)&&(!pose_reset))
    {
        this->calculateTimeUse();
        if(!robotpose.empty())
        {
            robotoriginpose.pose.position.x = robotpose.at(0);
            robotoriginpose.pose.position.y = robotpose.at(1);
            robotoriginpose.pose.position.z = 0.0;
            robotoriginpose.pose.orientation=tf::createQuaternionMsgFromYaw(math_util::degToRad(robotyaw));
        }
    }
    else if(notmeasurement_cnt==RobotMoveCount)
    {
//        CombiDarknetOpenface::changeViewPoint(currenttimesec);
    }
    else if(pose_reset)
    {
        if(pose_reset_cnt==PoseResetCount)
        {
            std::cout << "###########pose move############"<< std::endl;
            //move_mode = RobotPoseReset;
            after_flag = 1;
//            CombiDarknetOpenface::changeViewPoint(currenttimesec);
        }
        else
        {
            //pose_reset_cnt += 1;
            after_flag = 1;
            this->calculateTimeUseOutofView();
        }
    }

    if(this->object_viewing_times.size() < this->class_names.size())
    {
        this->object_viewing_times.resize(this->class_names.size(), 0);
    }

    if((!activityscoreface.empty()) && (!activityscoreobject.empty()))
    {
        if(darknet_cnt>1)
        {
            this->object_viewing_times.at(this->nearest_object_index) += current_time_sec - previous_time_sec;
        }

        // std::cout << "this->nearest_object_index:"<< this->nearest_object_index << std::endl;
        // activityscorefacedata<<darknet_cnt<<","
        //     <<currenttimesec<<", "
        //     <<this->nearest_object_index<<std::endl;
        // scorefacedata<<darknet_cnt<<","<<currenttimesec<<",";
        // scorefacelabel<<darknet_cnt<<","<<currenttimesec<<",";
        // activityscorefacedata2<<darknet_cnt<<","
        //     <<currenttimesec<<","
        //     <<notmeasurement_cnt<<","
        //     <<move_mode<<","
        //     <<robot_moving<<std::endl;

        // activityscoreobjectdata<<darknet_cnt<<","
        //     <<currenttimesec<<", "
        //     <<this->most_active_object_index<<std::endl;
        // scoreobjectdata<<darknet_cnt<<","<<currenttimesec<<",";
        // scoreobjectlabel<<darknet_cnt<<","<<currenttimesec<<",";
        // this->object_viewing_timesdata<<darknet_cnt<<","<<currenttimesec<<",";
        // for(int i=0;i<this->class_names.size();i++)
        // {
        //     if(i==(this->class_names.size()-1))
        //     {
        //         scorefacedata<< activityscoreface.at(i);
        //         scoreobjectdata<< activityscoreobject.at(i);
        //         scorefacelabel<<this->class_names.at(i);
        //         scoreobjectlabel<<this->class_names.at(i);
        //         this->object_viewing_timesdata<<this->object_viewing_times.at(i);
        //     }
        //     else
        //     {
        //         scorefacedata<< activityscoreface.at(i)<<",";
        //         scoreobjectdata<< activityscoreobject.at(i)<<",";
        //         scorefacelabel<<this->class_names.at(i)<<",";
        //         scoreobjectlabel<<this->class_names.at(i)<<",";
        //         this->object_viewing_timesdata<<this->object_viewing_times.at(i)<<",";
        //     }
        // }
        // scorefacedata<<std::endl;
        // scoreobjectdata<<std::endl;
        // scorefacelabel<<std::endl;
        // scoreobjectlabel<<std::endl;
        // this->object_viewing_timesdata<<std::endl;

        // if(kf_cnt>1)
        // {
        //     timeusedata<<darknet_cnt<<","
        //         <<currenttimesec<<","
        //         << personvelocity.at(0)<<","
        //         << personvelocity.at(1)<<","
        //         << estimateposition.at(0)<<","
        //         << estimateposition.at(1)<<std::endl;
        // }
    }

    // alltimerecord<<darknet_cnt<<","
    //     <<currenttimesec<<","
    //     <<robot_move_cnt<<","
    //     <<robot_move<<","
    //     <<person_move_cnt<<","
    //     <<static_cast<int>(person_moving_state)<<","
    //     <<notmeasurement_cnt<<","
    //     <<pose_reset_cnt<<","
    //     <<pose_reset<<","
    //     <<move_mode<<","
    //     <<robot_moving<<","
    //     <<moving_cnt<<std::endl;
    previous_time_sec = current_time_sec;
    frame_num += 1;
}

bool CombiDarknetOpenface::isIgnoredObjectClass(const std::string& class_name) const
{
    return std::find(this->ignore_object_list.begin(), this->ignore_object_list.end(), class_name) != this->ignore_object_list.end();
}

void CombiDarknetOpenface::linearLine(double x1,double y1,double x2,double y2,double* a,double* b )
{
    *a = (y2-y1)/(x2-x1);
    double a2 = *a;
    *b = y1-a2*x1;
}

/*
void CombiDarknetOpenface::changeViewPoint(double currenttimesec)
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
    }
    if(!robot_moving)
    {
        moving_cnt = 0;
    }
    else
    {
        moving_cnt += 1;
    }

    double robotyawtmp = robotyaw;

    if((!robotpose.empty())&&(nose_end_point2D_drawtmp&&(!pose_reset)))
    {
        if(move_mode==OrientationInView)
        {
            std::cout<<"move_mode==OrientationInView"<<std::endl;
            if(robot_moving==0)
            {
                double deltatheta = 0;
                targetrobotpose.at(2) = head_arrow_theta+180;
                if(targetrobotpose.at(2)>360)
                {
                    targetrobotpose.at(2)-=360;
                }

                double r = std::sqrt(std::pow(robotpose.at(0)-this->estimate_position_ptr->x, 2) + std::pow(robotpose.at(1)-this->estimate_position_ptr->y, 2));

                targetrobotpose.at(0) = r*cos(math_util::degToRad(head_arrow_theta))+this->estimate_position_ptr->x;
                targetrobotpose.at(1) = r*sin(math_util::degToRad(head_arrow_theta))+this->estimate_position_ptr->y;

                double r2 = std::sqrt(std::pow(targetrobotpose.at(0)-this->estimate_position_ptr->x, 2) + std::pow(targetrobotpose.at(1)-this->estimate_position_ptr->y, 2));

                robot_moving = 1;
                std::cout<<"r:"<<r<<std::endl;
                std::cout<<"r2:"<<r2<<std::endl;
                std::cout<<"robot pose:"<<robotpose.at(0)<<","<<robotpose.at(1)<<std::endl;
                std::cout<<"person pose:"<<this->estimate_position_ptr->x<<","<<this->estimate_position_ptr->y<<std::endl;
                std::cout<<"head_arrow_theta:"<<head_arrow_theta<<std::endl;
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
                    targetrobotpose.at(2) = head_arrow_theta;
                    out_view_mode=2;
                }
                else if((30<=headrobottheta)&&(headrobottheta<=90))
                {
                    std::cout<<"out of view mode:3"<<std::endl;
                    targetrobotpose.at(2) = robotyaw+(90-headrobottheta)+45;

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
                    targetrobotpose.at(2) = head_arrow_theta;
                    out_view_mode=5;
                }
                else if((-30>=headrobottheta)&&(headrobottheta>-90))
                {
                    std::cout<<"out of view mode:6"<<std::endl;
                    targetrobotpose.at(2) = robotyaw-90;
                    out_view_mode=6;
                }
                else
                {
                    std::cout<<"out of view mode error"<<std::endl;
                    std::cout<<"headarrowangle:"<<head_arrow_angle<<std::endl;
                    std::cout<<"head_arrow_theta:"<<head_arrow_theta<<std::endl;
                    std::cout<<"headrobottheta:"<<headrobottheta<<std::endl;
                }
                targetthetatmp = targetrobotpose.at(2);

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
            ROS_INFO("start to reset the pose");

            targetrobotpose.at(0) = beforerobotpose.at(0);
            targetrobotpose.at(1) = beforerobotpose.at(1);
            targetrobotpose.at(2) = beforerobotpose.at(2);
            robot_moving = 1;
            pose_reset = 2;
        }
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
            twist.angular.z= 0;
            signalangle = 0;
        }

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
                    linearLine(45,linearsigval,90.0,0.0,&a,&b);
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
                    linearLine(45,linearsigval,90.0,0.0,&a,&b);
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
                    linearLine(45,(-1)*linearsigval,90,0,&a,&b);
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
                    linearLine(45,(-1)*linearsigval,90.0,0.0,&a,&b);
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
                    linearLine(45,linearsigval,90,0,&a,&b);
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
                    linearLine(45,linearsigval,90,0,&a,&b);
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
                    linearLine(45,(-1)*linearsigval,90,0,&a,&b);
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
                    linearLine(45,(-1)*linearsigval,90,0,&a,&b);
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

        angularsig = math_util::round(twist.angular.z,3);
        linearxsig = math_util::round(twist.linear.x,3);
        linearysig = math_util::round(twist.linear.y,3);

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
            ROS_INFO("robot stop and exit");
            exitflag = 1;
        }

        if((out_view_mode==1)||(out_view_mode==2)||(out_view_mode==4)||(out_view_mode==5))
        {
            signallineary = 0;
            signallinearx = 0;
        }
        if((!signallineary)&&(!signallinearx)&&(!signalangle))
        {
            ROS_INFO("finish movement");
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
                ROS_INFO("finished pose reset and exit");
                exitflag = 1;
            }
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            after_flag = 1;
        }
    }
}
*/

void CombiDarknetOpenface::calculateTimeUse()
{
    this->nearest_object_index = 0;
    this->nearest_gaze_position_ptr.reset();
    if(nose_end_point2D_draw && this->person_box)
    {
        double nearest_object_distance;
        this->nearest_gaze_position_ptr.reset(new cv::Point2i(this->invalidPoint()));
        std::tie(this->nearest_object_index, nearest_object_distance, *this->nearest_gaze_position_ptr) = this->getNearestObject(*this->nose_tip_position_ptr, *this->nose_end_point2D_drawtmp, this->object_centers);
        if(this->nearest_gaze_position_ptr && !this->isInvalidPoint(*this->nearest_gaze_position_ptr))
        {
            if(nearest_object_distance > this->gaze_assigned_thresh)
            {
                this->nearest_object_index = 0;
            }
            if(this->object_distance_cache.value() > this->person_distance_cache.value())
            {
                this->nearest_object_index = 0;
                object_far = 1;
            }
            else
            {
                object_far = 0;
            }

            // if(this->nearest_object_index != 0)
            // {
            //     int gazeobjectx = this->object_centers.at(this->nearest_object_index).x-nose_tip_position_ptr->x;
            //     int gazeobjecty = this->object_centers.at(this->nearest_object_index).y-nose_tip_position_ptr->y;
            //     double thetagazeobject = math_util::radToDeg(atan2(gazeobjecty,gazeobjectx));

            //     double thetanoseendminx = nearest_gaze_position_ptr->x-nose_tip_position_ptr->x;
            //     double thetanoseendminy = nearest_gaze_position_ptr->y-nose_tip_position_ptr->y;
            //     double thetanoseendmin = math_util::radToDeg(atan2(thetanoseendminy,thetanoseendminx));

            //     if(abs(thetagazeobject-thetanoseendmin)<30)
            //     {
            //         move_mode = OrientationInView;
            //         if(object_far)
            //         {
            //             move_mode = ObjectFar;
            //         }
            //     }
            //     else
            //     {
            //         move_mode = OrientationOutView;
            //     }

            //     double nosetoend = std::sqrt(std::pow(nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x, 2) + std::pow(nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y, 2));

            //     std::cout << "######indicator########"<< std::endl;
            //     std::cout << "nose:" << nose_tip_position_ptr->x << "," << nose_tip_position_ptr->y<< std::endl;
            //     std::cout << "min_nose_end_index_tmp2:"<< min_nose_end_index_tmp2 << std::endl;
            //     std::cout << "boxcenter.at(min_nose_end_index_tmp2)  :"<<this->object_centers.at(this->nearest_object_index)<< std::endl;
            //     std::cout << "thetagazeobject:"<< thetagazeobject << std::endl;
            //     std::cout << "nearest_gaze_position_ptr  :"<<nearest_gaze_position_ptr->x<<","<<nearest_gaze_position_ptr->y<< std::endl;
            //     std::cout << "thetanoseendmin:"<< thetanoseendmin << std::endl;
            // }
        }
        // else
        // {
        //     this->nearest_object_index = 0;
        //     move_mode=FaceOrinentationError;
        // }

        //commentout(11/21)
        // std::cout << "noseobjectmindist:"<< noseobjectmindist << std::endl;
        // std::cout << "this->person_distance:"<< noseobjectmindist << std::endl;
        // std::cout << "minnoseend:"<< minnoseend << std::endl;
        // std::cout << "this->nearest_object_index:"<< this->nearest_object_index << std::endl;
        // std::cout << "boxcenter  :"<<this->object_centers[this->nearest_object_index]<< std::endl;

        activityscoreface.at(this->nearest_object_index) += 1;

        // if(darknet_cnt==1)
        // {
        //     lastmove_mode = move_mode;
        // }
        // if(this->nearest_object_index)
        // {
        //     notmeasurement_cnt = 0;
        //     move_mode = 0;
        // }
        // else if(lastmove_mode == move_mode)
        // {
        //     notmeasurement_cnt += 1;
        // }
        // else
        // {
        //     notmeasurement_cnt = 0;
        // }
        // lastmove_mode = move_mode;
        // std::cout << "move_mode:"<< move_mode << std::endl;
        // std::cout << "notmeasurement_cnt:"<< notmeasurement_cnt << std::endl;

    }
    //object movement
    // if(this->class_names.empty() ||  !this->person_box)
    // {
    //     return;
    // }

    // if(this->last_object_centers.empty())
    // {
    //     this->last_object_centers = this->object_centers;
    // }
    // else
    // {
    //     if(this->last_object_centers.size() == this->object_centers.size())
    //     {
    //         std::vector<float> object_movement_list;
    //         // std::vector<float> objectmovementtmp;
    //         for(std::size_t i = 0; i < this->class_names.size(); i++)
    //         {
    //             object_movement_list.push_back(this->isIgnoredObjectClass(this->class_names.at(i)) ? 0.0 : cv::norm(this->object_centers.at(i) - this->last_object_centers.at(i)));
    //         }
    //         auto max_move_object_iter = std::max_element(object_movement_list.begin(), object_movement_list.end());
    //         this->most_active_object_index = std::distance(objectmovement.begin(), citerobmove);
    //         if(*max_move_object_iter <= 10)
    //         {
    //             this->most_active_object_index = 0;
    //         }
    //         activityscoreobject.at(this->most_active_object_index) += 1;
    //     }
    //     this->last_object_centers = this->object_centers;
    // }
}

std::tuple<std::size_t, double, cv::Point2i> CombiDarknetOpenface::getNearestObject(const cv::Point2i& nose_tip_point, const cv::Point2i& nose_end_point, const std::vector<cv::Point2i>& object_centers, float angle_distance_thresh) const
{
    double nose_angle = math_util::radToDeg(math_util::atan2(nose_end_point - nose_tip_point));
    std::size_t min_object_index = 0;
    float min_object_distance = std::numeric_limits<float>::infinity();
    cv::Point2i min_gaze_position = this->invalidPoint();
    for(int distance = this->search_distance_step; distance <= this->search_distance_range; distance += this->search_distance_step)
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
            if(center_position.x == 0 && center_position.y == 0)
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
    std::vector<float> objectdistance;
    std::vector<float> objectdistancetmp;
    for(int i=0;i<this->class_names.size();i++)
    {
        objectdistance.push_back(std::sqrt(std::pow(this->object_centers.at(i).x-320, 2) + std::pow(this->object_centers.at(i).y-240, 2)));

        if((this->object_centers.at(i).x == 0)&&(this->object_centers.at(i).y == 0))
        {
            objectdistance.at(i) = 0;
        }
        if(objectdistance.at(i)>0)
        {
            objectdistancetmp.push_back(objectdistance.at(i));
        }
    }

    if(objectdistancetmp.empty())
    {
        this->nearest_object_index = 0;
    }
    else
    {
        auto minobjectdistance = std::min_element(std::begin(objectdistancetmp), std::end(objectdistancetmp));
        float minobjectdistancetmp = *minobjectdistance;
        std::vector<float>::iterator citerobdist =std::find(objectdistance.begin(), objectdistance.end(), minobjectdistancetmp);
        if(citerobdist != objectdistance.end())
        {
            this->nearest_object_index = std::distance(objectdistance.begin(), citerobdist);
        }

        double xcerror =  this->object_centers.at(this->nearest_object_index).x-320;
        if(abs(xcerror)>200)
        {
            this->nearest_object_index = 0;
        }
    }
    activityscoreface.at(this->nearest_object_index) += 1;
}

//RGB
void CombiDarknetOpenface::onRgbImageUpdated(const sensor_msgs::ImageConstPtr& msg) const
{
    cv_bridge::CvImagePtr cv_ptr;
    static int lastdarknetcnt = 0;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    cv::Mat dataimage = cv::Mat::zeros(480, 640, CV_8UC3);

    if(!after_flag)
    {
        //facescore
        if((nearest_gaze_position_ptr)&&(this->person_box)&&(!activityscoreface.empty()))
        {
            int zerocntface = std::count(activityscoreface.begin(), activityscoreface.end(), 0);
            int maxindexface = 0;

            if(activityscoreface.at(0)==0)
            {
                auto maxactivityscoreface = std::max_element(std::begin(activityscoreface), std::end(activityscoreface));
                float maxactivityscorefacetmp = *maxactivityscoreface;
                auto citeracscoreface =std::find(activityscoreface.begin(), activityscoreface.end(), maxactivityscorefacetmp);
                if (citeracscoreface != activityscoreface.end())
                {
                    maxindexface = std::distance(activityscoreface.begin(), citeracscoreface);
                }
                if(!((this->object_centers[maxindexface].x==0)&&(this->object_centers[maxindexface].y==0)))
                {
                    //box
                    cv::rectangle(cv_ptr->image, this->object_boxes[maxindexface], cv::Scalar(255, 0, 0), 3, 4);
                }
            }
            else
            {
                maxindexface = 0;
            }

            if(nearest_gaze_position_ptr)
            {
                cv::putText(cv_ptr->image, this->class_names.at(maxindexface), cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
            }
            else
            {
                cv::putText(cv_ptr->image, "cannot measure", cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
            }

            cv::circle(cv_ptr->image, *nose_end_point2D_draw3, 5, cv::Scalar(255,255,255), 3);

            double nose_end_angle_tmpx = nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x;
            double nose_end_angle_tmpy = nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y;
            double nose_end_angle_tmp = math_util::radToDeg(atan2(nose_end_angle_tmpy,nose_end_angle_tmpx));
            double nose_end_angle_3x = nose_end_point2D_draw3->x-nose_tip_position_ptr->x;
            double nose_end_angle_3y = nose_end_point2D_draw3->y-nose_tip_position_ptr->y;
            double nose_end_angle_3 = math_util::radToDeg(atan2(nose_end_angle_3y,nose_end_angle_3x));
            double nose_end_angle_2x = nose_end_point2D_draw2->x-nose_tip_position_ptr->x;
            double nose_end_angle_2y = nose_end_point2D_draw2->y-nose_tip_position_ptr->y;
            double nose_end_angle_2 = math_util::radToDeg(atan2(nose_end_angle_2y,nose_end_angle_2x));
            double thetanoseendx = nose_end_point2D_draw->x-nose_tip_position_ptr->x;
            double thetanoseendy = nose_end_point2D_draw->y-nose_tip_position_ptr->y;
            double thetanoseend = math_util::radToDeg(atan2(thetanoseendy,thetanoseendx));

            if((((nose_end_point2D_draw3->x>0)&&(nose_end_point2D_draw3->x<640))&&((nose_end_point2D_draw3->y>0)&&(nose_end_point2D_draw3->y<480)))&&(abs(nose_end_angle_tmp-nose_end_angle_3)<10))
            {
                cv::line(cv_ptr->image,*nose_tip_position_ptr, *nose_end_point2D_draw3, cv::Scalar(0,255,0), 2);
            }
            else if((((nose_end_point2D_draw2->x>0)&&(nose_end_point2D_draw2->x<640))&&((nose_end_point2D_draw2->y>0)&&(nose_end_point2D_draw2->y<480)))&&(abs(nose_end_angle_tmp-nose_end_angle_2)<10))
            {
                cv::line(cv_ptr->image,*nose_tip_position_ptr, *nose_end_point2D_draw2, cv::Scalar(0,255,0), 2);
            }
            else if((((nose_end_point2D_draw->x>0)&&(nose_end_point2D_draw->x<640))&&((nose_end_point2D_draw->y>0)&&(nose_end_point2D_draw->y<480)))&&(abs(nose_end_angle_tmp-thetanoseend)<10))
            {
                cv::line(cv_ptr->image,*nose_tip_position_ptr, *nose_end_point2D_draw, cv::Scalar(0,255,0), 2);

            }
            cv::circle(cv_ptr->image, *nearest_gaze_position_ptr, 7, cv::Scalar(0,0,255), 5);
        }

        if(pose_reset)
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
                        auto citeracscoreface =std::find(activityscoreface.begin(), activityscoreface.end(), maxactivityscorefacetmp);
                        if (citeracscoreface != activityscoreface.end())
                        {
                            maxindexface = std::distance(activityscoreface.begin(), citeracscoreface);
                        }
                        if(!((this->object_centers[maxindexface].x==0)&&(this->object_centers[maxindexface].y==0)))
                        {
                            //box
                            cv::rectangle(cv_ptr->image, this->object_boxes[maxindexface], cv::Scalar(255, 0, 0), 3, 4);
                            cv::circle(cv_ptr->image, this->object_centers[maxindexface], 5, cv::Scalar(255,0,0), 3);
                            cv::line(cv_ptr->image,cv::Point(320,240), this->object_centers[maxindexface], cv::Scalar(255,0,0), 2);
                        }
                    }
                    else
                    {
                        maxindexface = 0;
                    }
                }
                cv::putText(cv_ptr->image, this->class_names.at(maxindexface), cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
                cv::putText(cv_ptr->image, "out measuring", cv::Point(20,90), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
            }
        }
    }

    cv::putText(dataimage, std::to_string(darknet_cnt), cv::Point(550,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    if(this->person_box)
    {
        cv::putText(dataimage, "person found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }
    else
    {
        cv::putText(cv_ptr->image, "person not found", cv::Point(20,25), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
    }

    cv::putText(dataimage, std::to_string(angularsig), cv::Point(20,120), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(linearxsig), cv::Point(20,150), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(linearysig), cv::Point(20,180), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);

    cv::putText(dataimage, std::to_string(errortmptheta), cv::Point(140,120), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(errortmpx), cv::Point(140,150), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(errortmpy), cv::Point(140,180), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(head_arrow_angle), cv::Point(20,210), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(head_arrow_theta), cv::Point(20,240), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(headrobottheta), cv::Point(20,270), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);

    cv::putText(dataimage, std::to_string(robotyaw), cv::Point(20,330), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(targetthetatmp), cv::Point(20,360), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, std::to_string(errortmptheta), cv::Point(20,390), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);


    cv::putText(dataimage, std::to_string(moving_cnt), cv::Point(550,55), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,255), 2,CV_AA);
    cv::putText(dataimage, std::to_string(pose_reset_cnt), cv::Point(550,85), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,0,255), 2,CV_AA);
    cv::putText(dataimage, std::to_string(out_view_mode), cv::Point(550,115), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);

    cv::resize(dataimage, dataimage, cv::Size(), ResizeSize, ResizeSize);

    cv::namedWindow("data", CV_WINDOW_AUTOSIZE);
    cv::imshow("data", dataimage);

    cv::namedWindow("RGB image", CV_WINDOW_AUTOSIZE);
    cv::imshow("RGB image", cv_ptr->image);
}

void CombiDarknetOpenface::onDepthImageUpdated(const sensor_msgs::ImageConstPtr& msg)
{
    if(!this->person_box && !this->hasFocusedObject())
    {
        return;
    }
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }

    if(this->person_box)
    {
        cv::Rect person_area = nose_tip_position_ptr ?
            cv::Rect(nose_tip_position_ptr->x-1, nose_tip_position_ptr->y-1, 3, 3) :
            cv::Rect((this->person_box->tl() + this->person_box->br()) / 2, cv::Size(3, 3));
        double current_person_distance = this->getDepthValue(cv_ptr->image, person_area);
        this->person_distance_cache.update(current_person_distance);;
        double person_angle = AngleofView / 2 - (static_cast<double>((this->person_box->tl().x + this->person_box->br().x) / 2) / this->image_size.width) * AngleofView;
        if(robotpose_cnt > 0)
        {
            cv::Point2d person_position = this->getPositionInGlobal(
                cv::Point2d(robotpose.at(0), robotpose.at(1)),
                robotyaw,
                this->person_distance_cache.value(),
                person_angle
            );
            this->publishPersonMeasurement(person_position, this->estimate_position_ptr);
            this->publishPersonMarker(person_angle, person_position);
        }
    }

    if(this->hasFocusedObject())
    {
        cv::Rect object_area = cv::Rect((this->getFocusedObjectBox().tl() + this->getFocusedObjectBox().br()) / 2, cv::Size(3, 3));
        double current_object_distance = this->getDepthValue(cv_ptr->image, object_area);
        this->object_distance_cache.update(current_object_distance);
        double object_angle = AngleofView / 2 - (static_cast<double>((this->getFocusedObjectBox().tl().x + this->getFocusedObjectBox().br().x) / 2) / this->image_size.width) * AngleofView;
        if(robotpose_cnt > 0)
        {
            cv::Point2d object_position = this->getPositionInGlobal(
                cv::Point2d(robotpose.at(0), robotpose.at(1)),
                robotyaw,
                this->object_distance_cache.value(),
                object_angle
            );
            this->publishObjectMarker(object_position);
        }
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
    static std::unique_ptr<cv::Point2d> last_estimate_position_ptr(new cv::Point2d(msg->pose.position.x, msg->pose.position.y));
    last_estimate_position_ptr.reset(new cv::Point2d(*this->estimate_position_ptr));
    this->estimate_position_ptr.reset(new cv::Point2d(msg->pose.position.x, msg->pose.position.y));
    this->person_velocity_ptr.reset(new cv::Vec2d((*this->estimate_position_ptr) - (*last_estimate_position_ptr)));
    if(this->person_box)
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

    this->publishEstimatedPersonPositionMarker(*this->estimate_position_ptr, this->darknet_cnt);
}

void CombiDarknetOpenface::publishEstimatedPersonPositionMarker(const cv::Point2d& position, int num_tracking_frame) const
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

    visualization_msgs::Marker m2;
    m2.header.stamp = ros::Time::now();
    m2.header.frame_id = this->FIXED_FRAME;
    m2.ns = "Person";
    m2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m2.pose.position.x = position.x - 0.25;
    m2.pose.position.y = position.y;
    m2.text =  std::to_string(num_tracking_frame);
    m2.scale.x = .1;
    m2.scale.y = .1;
    m2.scale.z = 0.2;
    m2.color.a = 1;
    m2.lifetime = ros::Duration();
    m2.color.r = 1.0;
    this->cnt_text_pub.publish(m2);
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

cv::Point2i CombiDarknetOpenface::invalidPoint()const
{
    return cv::Point2i(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
}


bool CombiDarknetOpenface::isInvalidPoint(const cv::Point2i& p)const
{
    return p.x == std::numeric_limits<int>::max() && p.y == std::numeric_limits<int>::max();
}

bool CombiDarknetOpenface::hasFocusedObject() const
{
    return this->nearest_object_index != 0;
}


cv::Rect CombiDarknetOpenface::getFocusedObjectBox() const
{
    return this->object_boxes.at(this->nearest_object_index);
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