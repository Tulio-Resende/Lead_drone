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

#include <std_msgs/Float64MultiArray.h>


class ReferenceGenerator
{
private:

    void configPublishers();
        ros::Publisher ref_cmd_pub;
        ros::Publisher out_ref_pub;

    std_msgs::Float64MultiArray ref_msg;
    geometry_msgs::Vector3 output_ref;   
    
    // Defining Reference Generate System
    Eigen::Matrix<float, 4, 4> Am;
    Eigen::Matrix<float, 4, 1> xm;
    Eigen::Matrix<float, 1, 4> Cmx;
    Eigen::Matrix<float, 1, 4> Cmy;

public:

    ReferenceGenerator(/* args */);
    ~ReferenceGenerator();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();

    void sendRefPos(double h);
};
