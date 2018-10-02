#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "kalman.hpp"

//#include <people_msgs/PositionMeasurement.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/Marker.h>

template <typename T> std::string tostr(const T& t)
{
    std::ostringstream os; os<<t; return os.str();
}

//static std::string fixed_frame = "base_link";
static std::string fixed_frame = "map";


class KalmanTracker{
private:
    ros::Rate r;
    ros::Subscriber person_pos_sub;
    ros::Publisher markers_pub,text_pub,estimate_pos_pub,estimate_arrow_pub;
    ros::NodeHandle nh1;
    ros::Time start,now;
    Eigen::VectorXd x,y;
    Eigen::MatrixXd A; // System dynamics matrix
    Eigen::MatrixXd C; // Output matrix
    Eigen::MatrixXd Q; // Process noise covariance
    Eigen::MatrixXd R; // Measurement noise covariance
    Eigen::MatrixXd P; // Estimate error covariance
    KalmanFilter estimate_X,estimate_Y;
    int n; // Number of states
    int m; // Number of measurements
    int init_f,update_f;
    double dt; // Time step
    double t; // Time
    double sigma_obs;
public:
    KalmanTracker(ros::NodeHandle nh):
    nh1(nh),r(10),n(6),m(2),dt(1.0/10),sigma_obs(0.002),init_f(0)
    {
        x = Eigen::VectorXd(n);
        A = Eigen::MatrixXd(n,n);
        C = Eigen::MatrixXd(m,n);
        Q = Eigen::MatrixXd(n,n);
        R = Eigen::MatrixXd(m,m);
        P = Eigen::MatrixXd(n,n);
        // Discrete LTI projectile motion, measuring position only
        const double dt2 = 0.5 * dt * dt;
        A << 1, 0, dt,  0, dt2,   0,
                 0, 1,  0, dt,   0, dt2,
                 0, 0,  1,  0,  dt,   0,
                 0, 0,  0,  1,   0,  dt,
                 0, 0,  0,  0,   1,   0,
                 0, 0,  0,  0,   0,   1;
        C << 1, 0, 0, 0, 0, 0,
                  0, 1, 0, 0, 0, 0;
        // Reasonable covariance matrices
        Q = Eigen::MatrixXd::Identity(n, n) * 1.0e-6;;
        R = Eigen::MatrixXd::Identity(m, m) * pow(sigma_obs, 2);
        P = Eigen::MatrixXd::Identity(n, n) * 1.0e-6;
        estimate_X=KalmanFilter(dt,A, C, Q, R, P);
        estimate_X.init(0,x);
        //start = now = ros::Time::now();
        
        person_pos_sub = nh1.subscribe("filter_measurement",1,&KalmanTracker::msgCallback_PeopleTracker, this);
        
        markers_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_estimate_marker", 20);//Output of KF
        estimate_pos_pub = nh1.advertise<geometry_msgs::PoseStamped>("estimate_pos", 1);
        estimate_arrow_pub = nh1.advertise<visualization_msgs::Marker>("/visualization_estimate_arrow", 1);
        //text_pub = nh1.advertise<visualization_msgs::Marker>("tracking_text", 20);
        //estimate_pos_pub = nh1.advertise<geometry_msgs::PoseStamped>("human_pose", 20);

        /*
        std::cout << "A: \n" << A << std::endl;
        std::cout << "C: \n" << C << std::endl;
        std::cout << "Q: \n" << Q << std::endl;
        std::cout << "R: \n" << R << std::endl;
        std::cout << "P: \n" << P << std::endl;
        */
    }

    void msgCallback_PeopleTracker(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        
        if(msg->pose.position.z == 1)
        {
        // Best guess of initial states
            //std::cout<<"---Initialization---"<<std::endl;
            //start = now = ros::Time::now();
            x <<msg->pose.position.x, msg->pose.position.y,0.0,0.0,0.0, 0.0;
           
            std::cout<<"init_measurement: "<<msg->pose.position.x<<", "<<msg->pose.position.y<<std::endl;
            estimate_X.init(t,x);
            std::cout<<"init_estimate        : "<<estimate_X.state()[0]<<", "<<estimate_X.state()[1]<<std::endl;
     
            //init_f=1;
        }
        else
        {
            //now = ros::Time::now();
            //std::cout<<"---Update---"<<std::endl;
            Eigen::VectorXd z(m);
            z << msg->pose.position.x,msg->pose.position.y;
            //std::cout<<"update_measurement: "<<msg->pose.position.x<<", "<<msg->pose.position.y<<std::endl;
            estimate_X.update(z);
            //std::cout<<"update_estimate        : "<<estimate_X.state()[0]<<", "<<estimate_X.state()[1]<<std::endl;
        }
        double estimate_x = estimate_X.state()[0];
        double estimate_y = estimate_X.state()[1];
        
        
        //std::cout <<"x_hat=" << estimate_X.state().transpose()<<std::endl;
        //std::cout <<"y_hat=" << estimate_Y.state().transpose()<<std::endl;

        //std::cout<<""<<std::endl;
        
        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = fixed_frame;
        m.scale.x = 0.2;
        m.scale.y = 0.2;
        m.scale.z = 0.05;
        m.color.a = 1.0f;
        m.color.r = 0.0f;
        m.color.g = 1.0f;
        m.color.b = 0.0f;
        
        m.type = m.SPHERE;    
        m.lifetime = ros::Duration(0.5);
        m.pose.position.x = estimate_x;
        m.pose.position.y = estimate_y;

        geometry_msgs::PoseStamped estimate_pos;
        estimate_pos.pose.position.x = estimate_x;
        estimate_pos.pose.position.y = estimate_y;
        estimate_pos.pose.position.z = 0;

        visualization_msgs::Marker estimateposearrow;
        double yaw = atan2(estimate_y,estimate_x);
        //std::cout<<"yaw: "<<yaw<<std::endl;
        
        estimateposearrow.header.frame_id = fixed_frame;;
        estimateposearrow.header.stamp = ros::Time::now();
        estimateposearrow.ns = "basic_shapes";
        estimateposearrow.type = visualization_msgs::Marker::ARROW;
        estimateposearrow.action = visualization_msgs::Marker::ADD;
        estimateposearrow.pose.position.x = estimate_x;
        estimateposearrow.pose.position.y = estimate_y;
        estimateposearrow.pose.orientation=tf::createQuaternionMsgFromYaw(yaw);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        estimateposearrow.scale.x = 0.3;
        estimateposearrow.scale.y = 0.1;
        estimateposearrow.scale.z = 0.1;
        // Set the color -- be sure to set alpha to something non-zero!
        estimateposearrow.color.r = 0.0f;
        estimateposearrow.color.g = 1.0f;
        estimateposearrow.color.b = 0.0f;
        estimateposearrow.color.a = 1.0f;

        estimateposearrow.lifetime = ros::Duration();

        estimate_arrow_pub.publish(estimateposearrow);
        markers_pub.publish(m);
        estimate_pos_pub.publish(estimate_pos);        
        
        //estimate_pos.header.stamp = ros::Time::now();
        //estimate_pos.header.frame_id = fixed_frame;
        //estimate_pos.name = "tracking";
        
        //std::cout<<estimate_X.cov()<<std::endl;
        /*estimate_pos.covariance[0] = (estimate_X.cov()(0,0)+estimate_Y.cov()(0,0))/2;
        estimate_pos.covariance[1] = 0.0;
        estimate_pos.covariance[2] = 0.0;
        estimate_pos.covariance[3] = 0.0;
        estimate_pos.covariance[4] = (estimate_X.cov()(1,1)+estimate_Y.cov()(1,1))/2;
        estimate_pos.covariance[5] = 0.0;
        estimate_pos.covariance[6] = 0.0;
        estimate_pos.covariance[7] = 0.0;
        estimate_pos.covariance[8] = (estimate_X.cov()(2,2)+estimate_Y.cov()(2,2))/2;
        */
        /*
        if(init_f==1)
        {
            markers_pub.publish(m);
            estimate_pos_pub.publish(estimate_pos);
        }
        */

    //r.sleep();
    }
};

// 購読者ノードのメイン関数
int main(int argc, char **argv)
{
    // ノード名の初期化
    ros::init(argc, argv, "person_tracking_kalman");
    // ROSシステムとの通信のためのノードのハンドルを宣言
    ros::NodeHandle nh;
    KalmanTracker kt(nh);
    ros::spin();
    return 0;
}
