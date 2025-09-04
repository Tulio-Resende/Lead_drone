//! ROS
#include <ros/ros.h>
#include <ros/duration.h>

//! ROS standard msgs
#include <std_msgs/Float64.h>

// Standard libs
#include <cmath>
#include <eigen3/Eigen/Core>
#include <random>
#include <geometry_msgs/Vector3.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <sensor_msgs/Joy.h>




class ModelSystem
{
private:

    void configPublishers();
        ros::Publisher curPosPub;
        ros::Publisher curVelPub;
        ros::Publisher outModelPub;

    void configSubscribers();
        ros::Subscriber controlSignal;
            void receiveControlSignal(const sensor_msgs::Joy::ConstPtr& msg);

    geometry_msgs::PointStamped cur_pos;
    geometry_msgs::Vector3Stamped cur_vel;
    geometry_msgs::Vector3 output_model;
    geometry_msgs::Vector3 control_signal;   
    
    // Defining Reference Generate System
    Eigen::Matrix<float, 2, 2> Ad;
    Eigen::Matrix<float, 2, 1> Bd;
    Eigen::Matrix<float, 1, 2> Cd;
    Eigen::Matrix<float, 2, 1> state_x;
    Eigen::Matrix<float, 2, 1> state_y;
    Eigen::Matrix<float, 2, 1> state_z;
    Eigen::Matrix<float, 2, 1> state_yaw;

public:

    ModelSystem(/* args */);
    ~ModelSystem();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();

    void sendModelStates(double h);
};
