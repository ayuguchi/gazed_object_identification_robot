#include "combi_darknet_openface_node.hpp"
#include "math_util.h"

CombiDarknetOpenface::CombiDarknetOpenface(ros::NodeHandle nh):
nh1(nh)
{
    ros_object_sub = nh1.subscribe("darknet_ros/bounding_boxes",1, &CombiDarknetOpenface::onRecognizedObject, this);

    ros_face_sub = nh1.subscribe("faces",1, &CombiDarknetOpenface::onRecognizedFace, this);
    rgb_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 1, &CombiDarknetOpenface::onRgbImageUpdated, this);
    depth_img_sub = nh1.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &CombiDarknetOpenface::onDepthImageUpdated, this);

    ros_filtered_sub= nh1.subscribe("estimate_pos",1, &CombiDarknetOpenface::onPersonPositionEstimated, this);
    measurement_pub = nh1.advertise<geometry_msgs::PoseStamped>("filter_measurement", 1);
    ros_robotpose_sub = nh1.subscribe("/current_robot_pose",1, &CombiDarknetOpenface::onRobotPoseUpdated, this);

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
}

CombiDarknetOpenface::~CombiDarknetOpenface()
{
}

void CombiDarknetOpenface::onRecognizedFace(const combi_darknet_openface::Faces::ConstPtr& msg)
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
        cv::Mat rot_matrix_x, rot_matrix_y, rot_matrix_z;
        cv::Vec3d euler_angles;
        cv::decomposeProjectionMatrix(
            proj_matrix,
            this->camera_matrix,
            rotation_matrix,
            this->translation_vector,
            rot_matrix_x,
            rot_matrix_y,
            rot_matrix_z,
            euler_angles
        );
        EulerAngles head_orientation = {
            euler_angles[2],  // roll
            euler_angles[0],  // pitch
            euler_angles[1]  // yaw
        };
        this->modifyHeadOrientation(head_orientation);

        nose_end_point2D_draw.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, FirstDistance))));
        nose_end_point2D_draw2.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, SecondDistance))));
        nose_end_point2D_draw3.reset(new cv::Point2i(this->getProjectedPoint(cv::Point3f(0.0, 0.0, ThirdDistance))));

        if((!estimateposition.empty())&&(!head_orientation.empty()))
        {
            this->head_arrow_theta = calcHeadArrowAngle(head_orientation);
            this->publishHeadPoseArrow(estimateposition[0], estimateposition[1], this->head_arrow_theta);
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


void CombiDarknetOpenface::modifyHeadOrientation(EulerAngles& head_orientation)
{
    static int init = 0;

    static double lastheadori[3];
    static double lastnose_end_point2D_drawtmptheta;
    double nose_end_point2D_drawtmptheta;

    if(!init)
    {
        std::cout<<"init"<<std::endl;
        lastheadori[2] =  head_orientation[2];
        for(int i=0;i<3;i++)
        {
            lastrotation_value.push_back(rotation_vector.at<double>(0,i));
            lasttranslation_value.push_back(translation_vector.at<double>(0,i));
        }
        init = 1;
        lastnose_end_point2D_drawtmptheta = math_util::radToDeg(atan2(nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x,nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y));
    }

    nose_end_point2D_drawtmptheta = math_util::radToDeg(atan2(nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x,nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y));

    std::cout<<"nose_end_point2D_drawtmptheta:"<<nose_end_point2D_drawtmptheta<<","<<lastnose_end_point2D_drawtmptheta<<","<<lastnose_end_point2D_drawtmptheta-nose_end_point2D_drawtmptheta<<std::endl;

    if(!(((nose_end_point2D_drawtmp->x>0)&&(nose_end_point2D_drawtmp->x<640))&&((nose_end_point2D_drawtmp->y>0)&&(nose_end_point2D_drawtmp->y<480))))
    {
        std::cout<<"modify drawpoint"<<std::endl;
        for(int i=0;i<3;i++)
        {
            head_orientation[i] = lastheadori[i];
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
            lastheadori[i] = head_orientation[i];
            lastrotation_value.push_back(rotation_vector.at<double>(0,i));
            lasttranslation_value.push_back(translation_vector.at<double>(0,i));
        }
        lastnose_end_point2D_drawtmptheta = math_util::radToDeg(atan2(nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x,nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y));
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

    if(darknet_cnt==1) ROS_INFO("start");

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
                classnames.push_back((*itr).Class);
            }
        }

        objectcnt += 1;
    }
    std::cout<<"objectcnt:"<<objectcnt<<std::endl;

    if(personindex)
    {
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
            double xcerror =  boxcenterx.at(i)-320;
            std::cout << "xcerror:"<< xcerror << std::endl;
        }
    }

    std::cout<<"robot_move:"<<robot_move<<std::endl;
    std::cout<<"person_move:"<<person_move<<std::endl;
    std::cout<<"notmeasurement_cnt:"<<notmeasurement_cnt<<std::endl;
    std::cout<<"robot_moving:"<<robot_moving<<std::endl;
    std::cout<<"pose_reset:"<<pose_reset<<std::endl;
    std::cout<<"pose_reset_cnt:"<<pose_reset_cnt<<std::endl;
    std::cout<<"head_arrow_theta:"<<head_arrow_theta<<std::endl;
    std::cout<<"robotyaw:"<<robotyaw<<std::endl;

    double head_arrow_thetatmp = head_arrow_theta+180;
    if(head_arrow_thetatmp>360)
    {
        head_arrow_thetatmp -= 360;
    }
    headrobottheta = robotyaw-head_arrow_thetatmp;
    std::cout<<"headrobottheta:"<<headrobottheta<<std::endl;

    if(((!personbox.empty())&&(!robot_move)&&(!person_move))&&(notmeasurement_cnt<RobotMoveCount)&&(!robot_moving)&&(!pose_reset))
    {
        std::cout << "###########measure timeuse ############"<< std::endl;
        CombiDarknetOpenface::calculateTimeUse(currenttimesec);
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
        std::cout << "###########robot moving############"<< std::endl;
        CombiDarknetOpenface::changeViewPoint(currenttimesec);
    }
    else if(pose_reset)
    {
        std::cout << "###########pose reset############"<< std::endl;
        if(pose_reset_cnt==PoseResetCount)
        {
            std::cout << "###########pose move############"<< std::endl;
            move_mode = RobotPoseReset;
            after_flag = 1;
            CombiDarknetOpenface::changeViewPoint(currenttimesec);
        }
        else
        {
            pose_reset_cnt += 1;
            after_flag = 1;

            std::cout << "###########measure timeuse out of view############"<< std::endl;
            CombiDarknetOpenface::calculateTimeUseOutofView(currenttimesec);
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

void CombiDarknetOpenface::linearLine(double x1,double y1,double x2,double y2,double* a,double* b )
{
    *a = (y2-y1)/(x2-x1);
    double a2 = *a;
    *b = y1-a2*x1;
}

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

                double r = std::sqrt(std::pow(robotpose.at(0)-estimateposition.at(0), 2) + std::pow(robotpose.at(1)-estimateposition.at(1), 2));

                targetrobotpose.at(0) = r*cos(math_util::degToRad(head_arrow_theta))+estimateposition.at(0);
                targetrobotpose.at(1) = r*sin(math_util::degToRad(head_arrow_theta))+estimateposition.at(1);

                double r2 = std::sqrt(std::pow(targetrobotpose.at(0)-estimateposition.at(0), 2) + std::pow(targetrobotpose.at(1)-estimateposition.at(1), 2));

                robot_moving = 1;
                std::cout<<"r:"<<r<<std::endl;
                std::cout<<"r2:"<<r2<<std::endl;
                std::cout<<"robot pose:"<<robotpose.at(0)<<","<<robotpose.at(1)<<std::endl;
                std::cout<<"person pose:"<<estimateposition.at(0)<<","<<estimateposition.at(1)<<std::endl;
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
    else
    {
        std::cout<<"mode none"<<std::endl;
        std::cout<<"move_mode:"<<move_mode<<std::endl;
        std::cout<<"pose_reset:"<<pose_reset<<std::endl;
        std::cout<<"pose_reset_cnt:"<<pose_reset_cnt<<std::endl;
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

        visualization_msgs::Marker targetrobotposearrow;
        targetrobotposearrow.header.frame_id = fixed_frame;
        targetrobotposearrow.header.stamp = ros::Time::now();
        targetrobotposearrow.ns = "basic_shapes";
        targetrobotposearrow.type = visualization_msgs::Marker::ARROW;
        targetrobotposearrow.action = visualization_msgs::Marker::ADD;
        targetrobotposearrow.pose.position.x = targetrobotpose.at(0);
        targetrobotposearrow.pose.position.y = targetrobotpose.at(1);
        targetrobotposearrow.pose.position.z = 0;
        targetrobotposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(math_util::degToRad(targetrobotpose.at(2)));
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
        }
    }
}

void CombiDarknetOpenface::calculateTimeUse(double currenttimesec)
{
    noseendminindex = 0;
    nose_end_point2D_drawmin.clear();
    detectedobjectbox.clear();
    if(nose_end_point2D_draw &&(!personbox.empty()))
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
        double thetanoseendtmpx = nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x;
        double thetanoseendtmpy = nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y;
        double thetanoseendtmp = math_util::radToDeg(atan2(thetanoseendtmpy,thetanoseendtmpx));

        double thetanoseend3x = nose_end_point2D_draw3->x-nose_tip_position_ptr->x;
        double thetanoseend3y = nose_end_point2D_draw3->y-nose_tip_position_ptr->y;
        double thetanoseend3 = math_util::radToDeg(atan2(thetanoseend3y,thetanoseend3x));
        double thetanoseend2x = nose_end_point2D_draw2->x-nose_tip_position_ptr->x;
        double thetanoseend2y = nose_end_point2D_draw2->y-nose_tip_position_ptr->y;
        double thetanoseend2 = math_util::radToDeg(atan2(thetanoseend2y,thetanoseend2x));
        double thetanoseendx = nose_end_point2D_draw->x-nose_tip_position_ptr->x;
        double thetanoseendy = nose_end_point2D_draw->y-nose_tip_position_ptr->y;
        double thetanoseend = math_util::radToDeg(atan2(thetanoseendy,thetanoseendx));

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

        //variable face orintation
        for(int i=0;i<distancestep.size();i++)
        {
            nose_end_point2D.clear();
            nose_end_point3D.clear();
            nose_end_point3D.push_back(cv::Point3d(0,0,distancestep.at(i)));
            projectPoints(nose_end_point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, nose_end_point2D);

            double thetanoseendmintmpx = nose_end_point2D[0].x-nose_tip_position_ptr->x;
            double thetanoseendmintmpy = nose_end_point2D[0].y-nose_tip_position_ptr->y;
            double thetanoseendmintmp = math_util::radToDeg(atan2(thetanoseendmintmpy,thetanoseendmintmpx));

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
                }
                else
                {
                    auto minnoseenddistance = std::min_element(std::begin(noseenddistancetmp), std::end(noseenddistancetmp));
                    minnoseenddistancetmp = *minnoseenddistance;
                    std::vector<float>::iterator citernoseenddist =std::find(noseenddistance.begin(), noseenddistance.end(), minnoseenddistancetmp);
                    if (citernoseenddist != noseenddistance.end())
                    {
                        noseendminindextmp = std::distance(noseenddistance.begin(), citernoseenddist);
                    }
                }
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
            }

            if(noseobjectmindist>persondepthdist)
            {
                std::cout << "object is far" << std::endl;
                noseendminindex = 0;
                object_far = 1;
            }
            else
            {
                object_far = 0;
            }


            if(!noseendminindex)
            {
                double nosetoendmin = std::sqrt(std::pow(nose_end_point2D_drawmin[0]-nose_tip_position_ptr->x, 2) + std::pow(nose_end_point2D_drawmin[1]-nose_tip_position_ptr->y, 2));

                double nosetoendconst3 = std::sqrt(std::pow(nose_end_point2D_draw3->x-nose_tip_position_ptr->x, 2) + std::pow(nose_end_point2D_draw3->y-nose_tip_position_ptr->y, 2));
                double nosetoendconst2 = std::sqrt(std::pow(nose_end_point2D_draw2->x-nose_tip_position_ptr->x, 2) + std::pow(nose_end_point2D_draw2->y-nose_tip_position_ptr->y, 2));
                double nosetoendconsttmp = std::sqrt(std::pow(nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x, 2) + std::pow(nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y, 2));

                int gazeobjectx = boxcenterx.at(noseendminindextmp2)-nose_tip_position_ptr->x;
                int gazeobjecty = boxcentery.at(noseendminindextmp2)-nose_tip_position_ptr->y;
                double thetagazeobject = math_util::radToDeg(atan2(gazeobjecty,gazeobjectx));

                double thetanoseendminx = nose_end_point2D_drawmin[0]-nose_tip_position_ptr->x;
                double thetanoseendminy = nose_end_point2D_drawmin[1]-nose_tip_position_ptr->y;
                double thetanoseendmin = math_util::radToDeg(atan2(thetanoseendminy,thetanoseendminx));

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

                double nosetoend = std::sqrt(std::pow(nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x, 2) + std::pow(nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y, 2));

                std::cout << "######indicator########"<< std::endl;
                std::cout << "nose:" << nose_tip_position_ptr->x << "," << nose_tip_position_ptr->y<< std::endl;
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
                if(objectmovementtmp.empty())
                {
                    std::cout << "all movements are zero" << std::endl;
                    maxmoveindex = 0;
                }
                else
                {
                    auto maxobjectmovement = std::max_element(std::begin(objectmovementtmp), std::end(objectmovementtmp));
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
        }
    }
    std::cout <<"calculate time-use end"<< std::endl;
}

void CombiDarknetOpenface::calculateTimeUseOutofView(double currenttimesec)
{
    detectedobjectbox.empty();
    std::cout<<"calculateTimeUseOutofView"<<std::endl;
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
    }
    activityscoreface.at(noseendminindex) += 1;
    if(noseendminindex)
    {
        detectedobjectbox.push_back(boxxmin2.at(noseendminindex));
        detectedobjectbox.push_back(boxymin2.at(noseendminindex));
        detectedobjectbox.push_back(boxxmax2.at(noseendminindex));
        detectedobjectbox.push_back(boxymax2.at(noseendminindex));
    }
}

//RGB
void CombiDarknetOpenface::onRgbImageUpdated(const sensor_msgs::ImageConstPtr& msg)
{
    int i, j;
    int x1, x2, y1, y2;
    int width = WIDTH;
    int height = HEIGHT;
    cv_bridge::CvImagePtr cv_ptr;
    static int lastdarknetcnt = 0;

    rgb_cnt += 1;
    std::cout<<"rgb_callback:"<<rgb_cnt<<std::endl;
    if (darknet_cnt!=lastdarknetcnt)
    {
        std_msgs::Int16 capturecnt;
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

    if(!after_flag)
    {
        //facescore
        if((!nose_end_point2D_drawmin.empty())&&(!personbox.empty())&&(!activityscoreface.empty()))
        {
            int zerocntface = std::count(activityscoreface.begin(), activityscoreface.end(), 0);
            int maxindexface = 0;

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
                }
            }
            else
            {
                maxindexface = 0;
            }

            if(!nose_end_point2D_drawmin.empty())
            {
                cv::putText(cv_ptr->image, classnames.at(maxindexface), cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
            }
            else
            {
                cv::putText(cv_ptr->image, "cannot measure", cv::Point(20,60), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,0,255), 2,CV_AA);
            }

            cv::circle(cv_ptr->image, *nose_end_point2D_draw3, 5, cv::Scalar(255,255,255), 3);

            double thetanoseendtmpx = nose_end_point2D_drawtmp->x-nose_tip_position_ptr->x;
            double thetanoseendtmpy = nose_end_point2D_drawtmp->y-nose_tip_position_ptr->y;
            double thetanoseendtmp = math_util::radToDeg(atan2(thetanoseendtmpy,thetanoseendtmpx));
            double thetanoseend3x = nose_end_point2D_draw3->x-nose_tip_position_ptr->x;
            double thetanoseend3y = nose_end_point2D_draw3->y-nose_tip_position_ptr->y;
            double thetanoseend3 = math_util::radToDeg(atan2(thetanoseend3y,thetanoseend3x));
            double thetanoseend2x = nose_end_point2D_draw2->x-nose_tip_position_ptr->x;
            double thetanoseend2y = nose_end_point2D_draw2->y-nose_tip_position_ptr->y;
            double thetanoseend2 = math_util::radToDeg(atan2(thetanoseend2y,thetanoseend2x));
            double thetanoseendx = nose_end_point2D_draw->x-nose_tip_position_ptr->x;
            double thetanoseendy = nose_end_point2D_draw->y-nose_tip_position_ptr->y;
            double thetanoseend = math_util::radToDeg(atan2(thetanoseendy,thetanoseendx));

            if((((nose_end_point2D_draw3->x>0)&&(nose_end_point2D_draw3->x<640))&&((nose_end_point2D_draw3->y>0)&&(nose_end_point2D_draw3->y<480)))&&(abs(thetanoseendtmp-thetanoseend3)<10))
            {
                cv::line(cv_ptr->image,*nose_tip_position_ptr, *nose_end_point2D_draw3, cv::Scalar(0,255,0), 2);
            }
            else if((((nose_end_point2D_draw2->x>0)&&(nose_end_point2D_draw2->x<640))&&((nose_end_point2D_draw2->y>0)&&(nose_end_point2D_draw2->y<480)))&&(abs(thetanoseendtmp-thetanoseend2)<10))
            {
                cv::line(cv_ptr->image,*nose_tip_position_ptr, *nose_end_point2D_draw2, cv::Scalar(0,255,0), 2);
            }
            else if((((nose_end_point2D_draw->x>0)&&(nose_end_point2D_draw->x<640))&&((nose_end_point2D_draw->y>0)&&(nose_end_point2D_draw->y<480)))&&(abs(thetanoseendtmp-thetanoseend)<10))
            {
                cv::line(cv_ptr->image,*nose_tip_position_ptr, *nose_end_point2D_draw, cv::Scalar(0,255,0), 2);

            }

            //nose_end_point2D_drawmin
            cv::circle(cv_ptr->image, cv::Point(nose_end_point2D_drawmin[0], nose_end_point2D_drawmin[1]), 7, cv::Scalar(0,0,255), 5);
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

    cv::putText(dataimage, tostr(angularsig), cv::Point(20,120), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(linearxsig), cv::Point(20,150), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(linearysig), cv::Point(20,180), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);

    cv::putText(dataimage, tostr(errortmptheta), cv::Point(140,120), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(errortmpx), cv::Point(140,150), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(errortmpy), cv::Point(140,180), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(head_arrow_angle), cv::Point(20,210), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(head_arrow_theta), cv::Point(20,240), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(headrobottheta), cv::Point(20,270), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);

    cv::putText(dataimage, tostr(robotyaw), cv::Point(20,330), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(targetthetatmp), cv::Point(20,360), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);
    cv::putText(dataimage, tostr(errortmptheta), cv::Point(20,390), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2,CV_AA);


    cv::putText(dataimage, std::to_string(moving_cnt), cv::Point(550,55), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,255), 2,CV_AA);
    cv::putText(dataimage, std::to_string(pose_reset_cnt), cv::Point(550,85), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,0,255), 2,CV_AA);
    cv::putText(dataimage, std::to_string(out_view_mode), cv::Point(550,115), CV_FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255,255,0), 2,CV_AA);

    cv::resize(dataimage, dataimage, cv::Size(), ResizeSize, ResizeSize);

    cv::namedWindow("data", CV_WINDOW_AUTOSIZE);
    cv::imshow("data", dataimage);
    if(display_num==2)
    {
        //two displays
        cvMoveWindow("data",2910,350);
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

void CombiDarknetOpenface::onDepthImageUpdated(const sensor_msgs::ImageConstPtr& msg)
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

            if(nose_tip_position_ptr)
            {
                pointx1 = nose_tip_position_ptr->x-1;
                pointy1 = nose_tip_position_ptr->y-1;
                pointx2 = nose_tip_position_ptr->x+1;
                pointy2 = nose_tip_position_ptr->y+1;
                std::cout<<"point_nose:"<<nose_tip_position_ptr->x<<", "<<nose_tip_position_ptr->x<<std::endl;
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
            persondist = persondepthpoint;
            personangle = AngleofView/2-((double)xc/640)*AngleofView;
            persondisttmp = persondist;
            CombiDarknetOpenface::modifyPersonDistance(&persondist);
            persondepthdist = persondist;
            std::cout<<"persondist,angle:"<<persondist<<","<<personangle<<std::endl;
            if((fixed_frame =="map")&&(robotpose_cnt>0))
            {
                Eigen::MatrixXd Ap = Eigen::MatrixXd(3,3);
                Eigen::MatrixXd Bp = Eigen::MatrixXd(3,3);
                Eigen::MatrixXd Tp = Eigen::MatrixXd(3,3);

                Ap << cos(math_util::degToRad(robotyaw)), sin(math_util::degToRad(robotyaw)), robotpose.at(0),
                    sin(math_util::degToRad(robotyaw)), cos(math_util::degToRad(robotyaw)), robotpose.at(1),
                    0,  0,   1;

                Bp << cos(math_util::degToRad(personangle)), sin(math_util::degToRad(personangle)), persondist*cos(math_util::degToRad(personangle)),
                    sin(math_util::degToRad(personangle)), cos(math_util::degToRad(personangle)), persondist*sin(math_util::degToRad(personangle)),
                    0,  0,   1;

                Tp = Ap*Bp;

                double personmeasurementx = Tp(0,2);
                double personmeasurementy = Tp(1,2);

                std::cout<<"personmeasurement:"<<personmeasurementx<<","<<personmeasurementy<<std::endl;

                this->publishPersonMeasurement(personmeasurementx, personmeasurementy, estimateposition);
                this->publishPersonMarker(personangle,personmeasurementx,personmeasurementy);
            }
            else if(fixed_frame =="base_link")
            {
                double personmeasurementx = persondist * cos(math_util::degToRad(personangle));
                double personmeasurementy = persondist * sin(math_util::degToRad(personangle));
                this->publishPersonMeasurement(personmeasurementx, personmeasurementy, estimateposition);
                this->publishPersonMarker(personangle,personmeasurementx,personmeasurementy);
            }

            cv::rectangle(img, cv::Point(x1ori, y1ori), cv::Point(x2ori, y2ori), cv::Scalar(0, 0, 255), 5, 4);
            if(nose_tip_position_ptr)
            {
                cv::circle(img, *nose_tip_position_ptr, 8, cv::Scalar(0, 0, 0), 3);
            }
            else
            {
                cv::circle(img, cv::Point(xc, yc), 8, cv::Scalar(0, 0, 0), 3);
            }
        }
        if(!detectedobjectbox.empty())
        {
            objectdist = objectdepthpoint;
            noseobjectmindist = objectdist;
            objectangle = AngleofView/2-((double)objectxc/640)*AngleofView;
            objectdisttmp = objectdist;
            CombiDarknetOpenface::modifyObjectDistance(&objectdist);
            noseobjectmindist = objectdist;
            std::cout<<"objectdist,angle:"<<objectdist<<","<<objectangle<<std::endl;
            if((fixed_frame =="map")&&(robotpose_cnt>0))
            {
                Eigen::MatrixXd Ao = Eigen::MatrixXd(3,3);
                Eigen::MatrixXd Bo = Eigen::MatrixXd(3,3);
                Eigen::MatrixXd To = Eigen::MatrixXd(3,3);

                Ao << cos(math_util::degToRad(robotyaw)), sin(math_util::degToRad(robotyaw)), robotpose.at(0),
                    sin(math_util::degToRad(robotyaw)), cos(math_util::degToRad(robotyaw)), robotpose.at(1),
                    0,  0,   1;

                Bo << cos(math_util::degToRad(objectangle)), sin(math_util::degToRad(objectangle)), objectdist*cos(math_util::degToRad(objectangle)),
                    sin(math_util::degToRad(objectangle)), cos(math_util::degToRad(objectangle)), objectdist*sin(math_util::degToRad(objectangle)),
                    0,  0,   1;

                To = Ao*Bo;

                double objectmeasurementx = To(0,2);
                double objectmeasurementy = To(1,2);

                std::cout<<"objectmeasurement:"<<objectmeasurementx<<","<<objectmeasurementy<<std::endl;
                cv::rectangle(img, cv::Point(detectedobjectbox.at(0), detectedobjectbox.at(1)), cv::Point(detectedobjectbox.at(2), detectedobjectbox.at(3)), cv::Scalar(0, 0, 255), 5, 4);

                this->publishObjectMarker(objectmeasurementx,objectmeasurementy);
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
    }
    else
    {
        //one display
        cvMoveWindow("Depth_Image", 1350,0);
    }

    cv::waitKey(10);

    // depthdata<<darknet_cnt<<","
    //     <<currenttimesec<<", "
    //     << persondisttmp<<", "
    //     << persondist<<", "
    //     << xc<<", "
    //     << yc<<", "
    //     <<modify_distance_cnt<<","
    //     <<person_move_cnt<<","
    //     << person_move<<std::endl;

    std::cout<<""<<std::endl;
}

void CombiDarknetOpenface::modifyPersonDistance(double *distance)
{
    static double lastdistance;
    static int init = 0;

    if(!init)
    {
        lastdistance =  *distance;
        init = 1;
    }

    if (*distance == 0)
    {
        modify_distance_cnt += 1;
        std::cout<<"modify_distance_cnt:"<<modify_distance_cnt<<std::endl;
        *distance = lastdistance;
    }
    else
    {
        lastdistance = *distance;
    }
}

void CombiDarknetOpenface::modifyObjectDistance(double *distance)
{
    static double lastdistance;
    static int init = 0;

    if(!init)
    {
        lastdistance =  *distance;
        init = 1;
    }

    if (*distance == 0)
    {
        modify_distance_cnt += 1;
        std::cout<<"modify_distance_cnt:"<<modify_distance_cnt<<std::endl;
        *distance = lastdistance;
    }
    else
    {
        lastdistance = *distance;
    }
}

void CombiDarknetOpenface::onPersonPositionEstimated(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

    // personvelocitydata<<darknet_cnt<<","
    // <<currenttimesec<<", "
    // << estimateposition.at(0)<<", "
    // << estimateposition.at(1)<<", "
    // << personvelocity.at(0)<<", "
    // << personvelocity.at(1)<<", "
    // << person_move_cnt<<", "
    // <<person_move<<std::endl;

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

void CombiDarknetOpenface::onRobotPoseUpdated(const geometry_msgs::PoseStamped::ConstPtr& msg)
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
        robotyawraw = math_util::radToDeg(yaw);//deg

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

        visualization_msgs::Marker robotposearrow;

        robotposearrow.header.frame_id = fixed_frame;
        robotposearrow.header.stamp = ros::Time::now();
        robotposearrow.ns = "basic_shapes";
        robotposearrow.type = visualization_msgs::Marker::ARROW;
        robotposearrow.action = visualization_msgs::Marker::ADD;

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

        // robotvelocitydata<<darknet_cnt<<","
        // <<currenttimesec<<", "
        // << robotpose.at(0)<<", "
        // << robotpose.at(1)<<", "
        // << robotvelocity.at(0)<<", "
        // << robotvelocity.at(1)<<", "
        // << robot_move_cnt<<", "
        // <<robot_move<<std::endl;

        std::cout<<""<<std::endl;

}


void CombiDarknetOpenface::publishPersonMeasurement(double measurement_x, double measurement_y, const std::vector<double>& estimated_position) const
{
    static int kf_failed_cnt = 0;
    geometry_msgs::PoseStamped input_pose;
    input_pose.pose.position.x = measurement_x;
    input_pose.pose.position.y = measurement_y;
    if(estimated_position.empty())
    {
        input_pose.pose.position.z = 1;
        measurement_pub.publish(input_pose);
    }
    else
    {
        double dist_error = std::sqrt(std::pow(measurement_x - estimated_position[0], 2) + std::pow(measurement_y - estimated_position[1], 2));
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

void CombiDarknetOpenface::publishHeadPoseArrow(double position_x, double position_y, double head_arrow_angle_deg) const
{
    visualization_msgs::Marker head_pose_arrow;
    head_pose_arrow.header.frame_id = FIXED_FRAME;
    head_pose_arrow.header.stamp = ros::Time::now();
    head_pose_arrow.ns = "basic_shapes";
    head_pose_arrow.type = visualization_msgs::Marker::ARROW;
    head_pose_arrow.action = visualization_msgs::Marker::ADD;
    head_pose_arrow.pose.position.x = position_x;
    head_pose_arrow.pose.position.y = position_y;
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

void CombiDarknetOpenface::publishPersonMarker(double theta, double measurement_x, double measurement_y) const
{
    visualization_msgs::Marker person_arrow;
    person_arrow.header.frame_id = FIXED_FRAME;
    person_arrow.header.stamp = ros::Time::now();
    person_arrow.ns = "basic_shapes";
    person_arrow.type = visualization_msgs::Marker::ARROW;
    person_arrow.action = visualization_msgs::Marker::ADD;
    person_arrow.pose.position.x = measurement_x;
    person_arrow.pose.position.y = measurement_y;
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
    person_marker.pose.position.x = measurement_x;
    person_marker.pose.position.y = measurement_y;
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

void CombiDarknetOpenface::publishObjectMarker(double measurement_x, double measurement_y) const
{
    visualization_msgs::Marker object_marker;
    object_marker.header.stamp = ros::Time::now();
    object_marker.header.frame_id = FIXED_FRAME;
    object_marker.pose.position.x = measurement_x;
    object_marker.pose.position.y = measurement_y;
    object_marker.lifetime = ros::Duration(1);
    object_marker.scale.x = 0.2;
    object_marker.scale.y = 0.2;
    object_marker.scale.z = 0.01;
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.color.a = 0.75;
    object_marker.color.b =  1.0;
    object_marker_pub.publish(object_marker);
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