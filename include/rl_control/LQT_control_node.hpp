//! ROS
#include <ros/ros.h>
#include <ros/duration.h>

//! ROS standard msgs
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

// Standard libs
#include <cmath>
#include <eigen3/Eigen/Core>
#include <random>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32MultiArray.h>


class LQTController
{
private:

    void configSubscribers();
        ros::Subscriber refGeneratorSub;
            void receiveRef(const std_msgs::Float32MultiArray::ConstPtr& msg);
        ros::Subscriber curPosSub;
            void receivePos(const geometry_msgs::PointStamped::ConstPtr& msg);
        ros::Subscriber curVelSub;
            void receiveVel(const geometry_msgs::Vector3sStamped::ConstPtr& msg);
        
    void configPublishers();
        ros::Publisher vel_pub;
    
    std::array<double,5> ref_msg;    
    sensor_msgs::Joy vel_msg;
    // std_msgs::Float32MultiArray ref_msg;
    Eigen::Vector3d cur_pos;
    Eigen::Vector3d cur_vel;
  


public:

    LQTController(/* args */);
    ~LQTController();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();

    void sendCmdVel(double h);
};
