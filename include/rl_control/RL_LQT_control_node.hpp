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
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <utility>
#include <std_msgs/Float64MultiArray.h>

class RLLQTController
{
private:

    std::default_random_engine generator;
    std::normal_distribution<double> dist;

    void configSubscribers();
        ros::Subscriber refGeneratorSub;
            void receiveRef(const std_msgs::Float64MultiArray::ConstPtr& msg);
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


    Eigen::Vector3d cur_pos, cur_vel;
    Eigen::VectorXd ref_msg, old_ref_msg;
    Eigen::VectorXd state_x, state_y, old_state_x, old_state_y;
    std::vector<Eigen::VectorXd> phi;
    Eigen::VectorXd augmented_state_x, augmented_state_y, old_augmented_state_x, old_augmented_state_y;
    Eigen::RowVectorXd Kx, Ky, kz;
    Eigen::VectorXd theta;
    std::vector<Eigen::MatrixXd> H;
    Eigen::MatrixXd prls;
    Eigen::Vector3f u, old_u;
    Eigen::Vector3d old_pos, old_vel, old_ref;
    Eigen::Vector3d Erls, reward;


    Eigen::Vector3d excitation;
    Eigen::VectorXd old_bar_x, old_bar_y;
    Eigen::VectorXd bar_x, bar_y;

    Eigen::Matrix<double, 1, 2> Cd;
    Eigen::Matrix<double, 1, 5> Cmx;
    Eigen::Matrix<double, 1, 5> Cmy;
    Eigen::Matrix<double, 1, 7> Cx;
    Eigen::Matrix<double, 1, 7> Cy;
    Eigen::Matrix<double, 7, 7> Qx;
    Eigen::Matrix<double, 7, 7> Qy;
    Eigen::Matrix<double, 2, 2> Ad;
    Eigen::Matrix<double, 2, 1> Bd;
    Eigen::Matrix<double, 5, 5> Am;
    Eigen::Matrix<double, 5, 1> Bm;
    Eigen::MatrixXd Aa;
    Eigen::MatrixXd Ba;
    Eigen::MatrixXd A_dlyap;
    Eigen::MatrixXd Q_dlyap;

    
    double K0factor, THETA0factor, PRLS0factor;
    double countk, inv_scalar;
    
    // bool control_enabled = false;
    bool flag_first_pos = true;
    bool flag_first_vel = true;
    bool flag_first_ref = true;
    bool flag_pos = false;
    bool flag_vel = false;
    bool flag_ref = false;
    bool gain_update = true;
    bool dlyap_flag = true;

    
    sensor_msgs::Joy vel_msg;

public:

    RLLQTController(/* args */);
    ~RLLQTController();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();

    Eigen::VectorXd fromx2xbar(const Eigen::VectorXd& v);
    Eigen::MatrixXd FromTHETAtoP(const Eigen::VectorXd& theta, int sizeOfAugState);
    Eigen::Vector3d Excitation(double& t);
    Eigen::MatrixXd dlyap(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q);
    Eigen::MatrixXd kronecker(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
    Eigen::MatrixXd dlyap_iterative(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q, 
                               int max_iter = 1000, double tol = 1e-12);
    void UpdateRLS(Eigen::VectorXd& theta, std::vector<Eigen::VectorXd>& phi, Eigen::Vector3d& Erls, Eigen::MatrixXd& prls, double& mu);
    void UpdateGain(Eigen::VectorXd& theta, const Eigen::MatrixXd& A_dlyap, const Eigen::MatrixXd& Q_dlyap);
    void sendCmdVel(double h);

};
