//! ROS
#include <ros/ros.h>
#include <ros/duration.h>

#include <tf2_ros/transform_listener.h>

//! ROS standard msgs
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Standard libs
#include <cmath>
#include <eigen3/Eigen/Core>
#include <random>
#include <std_srvs/SetBool.h>
#include <random>


class RLLQRController
{
private:

    std::default_random_engine generator;
    std::normal_distribution<double> dist;

    void configSubscribers();
        ros::Subscriber curPosSub;
            void receivePos(const geometry_msgs::PointStamped::ConstPtr& msg);
        ros::Subscriber curVelSub;
            void receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
        
    void configPublishers();
        ros::Publisher vel_pub;
        ros::Publisher reward_pub;
        ros::Publisher gain_pub;
    
    // void configService();
    //     ros::ServiceServer enable_service;
    //     bool enableControl(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);


    Eigen::Vector3d cur_pos;
    Eigen::Vector3d cur_vel;
    Eigen::VectorXd state_x, state_y, old_state_x, old_state_y;
    std::vector<Eigen::VectorXd> phi;
    Eigen::VectorXd augmented_state_x, augmented_state_y, old_augmented_state_x, old_augmented_state_y;
    Eigen::RowVectorXd Kx, Ky, kz;
    Eigen::VectorXd theta;
    std::vector<Eigen::MatrixXd> H;
    Eigen::MatrixXd prls;
    Eigen::Vector3f u, old_u;
    Eigen::Vector3d old_pos;
    Eigen::Vector3d old_vel;
    Eigen::Vector3d Erls;
    Eigen::Vector3d reward;
    
    double K0factor;
    double THETA0factor;
    double PRLS0factor;
    double countk;
    
    // bool control_enabled = false;
    bool flag_first_pos = true;
    bool flag_first_vel = true;
    bool flag_pos = true;
    bool flag_vel = false;
    


    sensor_msgs::Joy vel_msg;

public:

    RLLQRController(/* args */);
    ~RLLQRController();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();
    Eigen::VectorXd fromx2xbar(const Eigen::VectorXd& v);
    Eigen::MatrixXd FromTHETAtoP(const Eigen::VectorXd& theta, int sizeOfAugState);

    void sendCmdVel(double h);
};
