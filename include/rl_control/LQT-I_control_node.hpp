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
#include <std_msgs/Float64MultiArray.h>


class LQTIController
{
private:

void configSubscribers();
    ros::Subscriber refGeneratorSub;
        void receiveRef(const std_msgs::Float64MultiArray::ConstPtr& msg);
    ros::Subscriber curPosSub;
        void receivePos(const geometry_msgs::PointStamped::ConstPtr& msg);
    ros::Subscriber curVelSub;
        void receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    // ros::Subscriber curRefOutput;
        // void receiveRefOut(const geometry_msgs::Vector3::ConstPtr& msg);
    // ros::Subscriber curModelOutput;
    //     void receiveModelOut(const geometry_msgs::Vector3::ConstPtr& msg);


void configPublishers();
    ros::Publisher vel_pub;

std::array<double,5> ref_msg;    
sensor_msgs::Joy vel_msg;
Eigen::Vector3d cur_pos;
Eigen::Vector3d cur_vel;
Eigen::Vector3d cur_model_output;
Eigen::Vector3d cur_ref_output;
Eigen::Vector3d tilde_mu;
Eigen::Vector3d old_pos;
Eigen::Vector3d tilde_pos;
Eigen::Vector3d old_vel;
Eigen::Vector3d tilde_vel;
Eigen::Vector3d old_u;
Eigen::Vector3f tilde_u;

Eigen::Vector3f u;
double yss;

bool flag = false;

public:

LQTIController(/* args */);
~LQTIController();

ros::NodeHandle handle;
ros::NodeHandle priv_handle;

void configNode();

void sendCmdVel(double h);
};
