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
#include <std_srvs/SetBool.h>


class ReferenceGenerator
{
private:

    void configPublishers();
        ros::Publisher ref_cmd_pub;
        ros::Publisher out_ref_pub;
    void configService();
        ros::ServiceClient client_ref;

    std_msgs::Float64MultiArray ref_msg;
    geometry_msgs::Vector3 output_ref;   
    
    // Defining Reference Generate System
    Eigen::Matrix<float, 5, 5> Am;
    Eigen::Matrix<float, 5, 1> xm;
    Eigen::Matrix<float, 1, 5> Cmx;
    Eigen::Matrix<float, 1, 5> Cmy;

    std_srvs::SetBool srv;

public:

    ReferenceGenerator(/* args */);
    ~ReferenceGenerator();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();
    bool enableControl(bool enable);

    void sendRefPos(double h);
};
