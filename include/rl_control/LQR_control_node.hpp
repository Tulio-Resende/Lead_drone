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

class LQRController
{
private:

    void configSubscribers();
        ros::Subscriber curPosSub;
            void receivePos(const geometry_msgs::PointStamped::ConstPtr& msg);
        ros::Subscriber curVelSub;
            void receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

        
    void configPublishers();
        ros::Publisher vel_pub;
        // ros::Publisher pub2;
    
    Eigen::Vector3d integrator(Eigen::Vector3d int_error, Eigen::Vector3d error, double h);

    Eigen::Vector3d cur_pos;
    Eigen::Vector3d cur_vel;

    // Eigen::Vector3d des_pos;
    // Eigen::Vector3d des_vel = {0.0, 0.0, 0.0};
    // Eigen::Vector3d des_accel = {0.0, 0.0, 0.0};

    // Eigen::Vector3d error; 
    // Eigen::Vector3d d_error;
    // Eigen::Vector3d int_error = {0.0, 0.0, 0.0};

    sensor_msgs::Joy vel_msg;
    // geometry_msgs::Vector3Stamped;

public:

    LQRController(/* args */);
    ~LQRController();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();

    void sendCmdVel(double h);
};
