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
#include <std_srvs/SetBool.h>

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

    void configService();
        ros::ServiceServer service_model;

    geometry_msgs::PointStamped cur_pos;
    geometry_msgs::Vector3Stamped cur_vel;
    geometry_msgs::Vector3 output_model;
    geometry_msgs::Vector3 control_signal;   
    
    // Defining Reference Generate System
    Eigen::MatrixXd Ad;
    Eigen::MatrixXd Bd;
    Eigen::MatrixXd Cd;
    Eigen::VectorXd state_x;
    Eigen::VectorXd state_y;
    Eigen::VectorXd state_z;
    Eigen::VectorXd state_yaw;

    int count;
    bool control_enabled;

public:

    ModelSystem(ros::NodeHandle& nh);
    ~ModelSystem();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();

    void sendModelStates(double h);

   bool handleEnable(std_srvs::SetBool::Request &req,
                  std_srvs::SetBool::Response &res);

};
