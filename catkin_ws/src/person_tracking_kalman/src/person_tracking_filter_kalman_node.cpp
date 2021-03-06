/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "kalman.hpp"

#include <people_msgs/PositionMeasurement.h>
#include <visualization_msgs/Marker.h>

template <typename T> std::string tostr(const T& t)
{
    std::ostringstream os; os<<t; return os.str();
}

static std::string fixed_frame = "odom";

class KalmanTracker{
private:
    ros::Rate r;
    ros::Subscriber person_pos_sub;
    ros::Publisher markers_pub,text_pub,estimate_pos_pub;
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
public:
    KalmanTracker(ros::NodeHandle nh):
    nh1(nh),r(10),n(3),m(1),dt(1.0/10),init_f(0)
    {
        x = Eigen::VectorXd(n);
        y = Eigen::VectorXd(n);
        A = Eigen::MatrixXd(n,n);
        C = Eigen::MatrixXd(m,n);
        Q = Eigen::MatrixXd(n,n);
        R = Eigen::MatrixXd(m,m);
        P = Eigen::MatrixXd(n,n);
        // Discrete LTI projectile motion, measuring position only
        A << 1, dt, 1/2*dt*dt, 0, 1, dt, 0, 0, 1;
        C << 1, 0, 0;
        // Reasonable covariance matrices
        Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
        R << 5;
        P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;
        estimate_X=KalmanFilter(dt,A, C, Q, R, P);
        estimate_Y=KalmanFilter(dt,A, C, Q, R, P);
        estimate_X.init(0,x);
        estimate_Y.init(0,y);
        //start = now = ros::Time::now();
        person_pos_sub = nh1.subscribe("filter_measurement",1,&KalmanTracker::msgCallback_PeopleTracker, this);
        markers_pub = nh1.advertise<visualization_msgs::Marker>("filter_marker", 20);//Output of KF
        estimate_pos_pub = nh1.advertise<people_msgs::PositionMeasurement>("estimate_pos", 20);
        //text_pub = nh1.advertise<visualization_msgs::Marker>("tracking_text", 20);
        //estimate_pos_pub = nh1.advertise<geometry_msgs::PoseStamped>("human_pose", 20);

        std::cout << "A: \n" << A << std::endl;
        std::cout << "C: \n" << C << std::endl;
        std::cout << "Q: \n" << Q << std::endl;
        std::cout << "R: \n" << R << std::endl;
        std::cout << "P: \n" << P << std::endl;
  }

    void msgCallback_PeopleTracker(const people_msgs::PositionMeasurement::ConstPtr& msg )
    {
        if(msg->initialization == 1){
        // Best guess of initial states
            std::cout<<"---Initialization---"<<std::endl;
            //start = now = ros::Time::now();
            x <<msg->pos.x, 0.0, 0.0;
            estimate_X.init(t,x);
            y << msg->pos.y, 0.0, 0.0;
            estimate_Y.init(t,y);
            init_f=1;
        }
        else if(init_f==1){
            //now = ros::Time::now();
            std::cout<<"Update position"<<std::endl;
            Eigen::VectorXd z(m);
            z <<msg->pos.x;
            estimate_X.update(z);
            z <<msg->pos.y;
            estimate_Y.update(z);
        }

        //std::cout <<"x_hat=" << estimate_X.state().transpose()<<std::endl;
        //std::cout <<"y_hat=" << estimate_Y.state().transpose()<<std::endl;

        visualization_msgs::Marker m;
        m.header.stamp = ros::Time::now();
        m.header.frame_id = fixed_frame;
        m.type = m.SPHERE;
        m.scale.x = .6;
        m.scale.y = .6;
        m.scale.z = .6;
        m.color.a = 0.2;
        m.color.g = 0.5;
        m.pose.position.x = estimate_X.state()[0];
        m.pose.position.y = estimate_Y.state()[0];

        people_msgs::PositionMeasurement estimate_pos;
        estimate_pos.header.stamp = ros::Time::now();
        estimate_pos.header.frame_id = fixed_frame;
        estimate_pos.name = "tracking";
        estimate_pos.pos.x = estimate_X.state()[0];
        estimate_pos.pos.y = estimate_Y.state()[0];
        estimate_pos.pos.z = 0;
        //std::cout<<estimate_X.cov()<<std::endl;
        estimate_pos.covariance[0] = (estimate_X.cov()(0,0)+estimate_Y.cov()(0,0))/2;
        estimate_pos.covariance[1] = 0.0;
        estimate_pos.covariance[2] = 0.0;
        estimate_pos.covariance[3] = 0.0;
        estimate_pos.covariance[4] = (estimate_X.cov()(1,1)+estimate_Y.cov()(1,1))/2;
        estimate_pos.covariance[5] = 0.0;
        estimate_pos.covariance[6] = 0.0;
        estimate_pos.covariance[7] = 0.0;
        estimate_pos.covariance[8] = (estimate_X.cov()(2,2)+estimate_Y.cov()(2,2))/2;

        if(init_f==1){
            markers_pub.publish(m);
            estimate_pos_pub.publish(estimate_pos);
        }
    //r.sleep();
    }
};

// 購読者ノードのメイン関数
int main(int argc, char **argv)
{
    // ノード名の初期化
    ros::init(argc, argv, "person_tracking_filter_kalman");
    // ROSシステムとの通信のためのノードのハンドルを宣言
    ros::NodeHandle nh;
    KalmanTracker kt(nh);
    ros::spin();
    return 0;
}
