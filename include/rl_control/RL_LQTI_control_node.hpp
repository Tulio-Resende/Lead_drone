//! ROS
#include <ros/ros.h>
#include <ros/duration.h>

//! ROS standard msgs
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

// Standard libs
#include <cmath>
#include <eigen3/Eigen/Core>
#include <random>
#include <std_msgs/Float64MultiArray.h>

struct PlantAxis
{
    std::string name;      // "x", "y", "z", "yaw"
    Eigen::MatrixXd Cm;    // Matriz Cm do eixo
    Eigen::MatrixXd Q;     // Matriz Q resultante
    Eigen::MatrixXd Q_dlyap; // Matriz Q_dlyap resultante

    PlantAxis() = default;
    PlantAxis(const std::string& n, const Eigen::MatrixXd& C)
        : name(n), Cm(C) {}
};

struct AxisSystem {
    Eigen::MatrixXd Ap;
    Eigen::MatrixXd Bp;
    Eigen::MatrixXd Aa;
    Eigen::MatrixXd Ba;
    Eigen::MatrixXd A_dlyap;
};


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


void configPublishers();
    ros::Publisher vel_pub;
    ros::Publisher reward_pub;
    ros::Publisher gain_pub;
    ros::Publisher cost_pub;
        

sensor_msgs::Joy vel_msg;
std_msgs::Float64MultiArray gain_msg_x, gain_msg_y;

Eigen::VectorXd ref_msg; 
Eigen::Vector3d cur_pos, old_pos, tilde_pos;
Eigen::Vector3d cur_vel, old_vel, tilde_vel;
Eigen::Vector3d tilde_mu;
Eigen::VectorXd tilde_state_x, tilde_state_y;
Eigen::VectorXd old_state_x, old_state_y, state_x, state_y;
Eigen::Vector3d u, old_u, tilde_u;
Eigen::RowVectorXd Kx, Ky, kz;
Eigen::VectorXd old_y;

Eigen::MatrixXd Cd;
Eigen::MatrixXd Am, Bm, Cmx, Cmy, Cmz, Cmyaw;
Eigen::MatrixXd Cax, Cay, Caz, Cayaw;
Eigen::MatrixXd Ap, Bp;
Eigen::MatrixXd Aa, Ba;
Eigen::MatrixXd Qx, Qy, Qz, Qyaw;
Eigen::MatrixXd A_dlyap_x, A_dlyap_y;
Eigen::MatrixXd Q_dlyap_x, Q_dlyap_y, Q_dlyap_z, Q_dlyap_yaw;
       
Eigen::VectorXd theta_x, theta_y;
std::vector<Eigen::MatrixXd> H;
std::vector<Eigen::VectorXd> phi;

Eigen::Vector3d Erls, reward;
Eigen::Vector3d cost;

Eigen::VectorXd augmented_state_x, augmented_state_y, old_augmented_state_x, old_augmented_state_y;
Eigen::VectorXd old_bar_x, old_bar_y;
Eigen::VectorXd bar_x, bar_y;
 
Eigen::Vector3d excitation;

double K0factor, THETA0factor, PRLS0factor, ALPHA0factor;
double countk, inv_scalar_x, inv_scalar_y, inv_scalar_z, Qe, R, kpx, kpy, mux, muy, ki;

double yss, gamma;
bool flag_pos = false;
bool flag_vel = false;
bool flag_ref = false;
bool flag_second_time = false;

bool rl = false;
bool gain_update = true;
bool dlyap_flag = false;

public:

LQTIController(ros::NodeHandle& nh);
~LQTIController();

ros::NodeHandle handle;
ros::NodeHandle priv_handle;

void configNode();
void sendCmdVel(double h);


Eigen::VectorXd fromx2xbar(const Eigen::VectorXd& v);
Eigen::MatrixXd dlyap_iterative(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q, int max_iter = 1000, double tol = 1e-12);
double Calc_reward(const Eigen::VectorXd& old_state, const float& old_u, const Eigen::MatrixXd& Q, const double& R);
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> UpdateMatrices(const double& kp);
Eigen::VectorXd UpdateTheta(const Eigen::MatrixXd& H_THETA);
void Calc_Q_lyap(std::vector<PlantAxis>& axes, const Eigen::MatrixXd& Cd, const double& Qe, const double R);
void Calc_reward_all(double& h);
AxisSystem buildAxisSystem(double& kp, const Eigen::RowVectorXd& K);
void totalCost(const Eigen::Vector3d& reward, double& h);

};
