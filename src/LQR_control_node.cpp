#include "rl_control/LQR_control_node.hpp"

/* Constructor */
LQRController::LQRController(/* args */): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");
}

/* Destructor */
LQRController::~LQRController()
{
}

void LQRController::configNode(){
    configSubscribers();
    configPublishers();
    // ROS_INFO("Node configured.");
}

void LQRController::configSubscribers(){
    curPosSub = handle.subscribe("/dji_sdk/local_position", 1, &LQRController::receivePos, this);
    curVelSub = handle.subscribe("/dji_sdk/velocity", 1, &LQRController::receiveVel, this);
}

void LQRController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg){
    if (msg){
        cur_pos(0) = msg->point.x; 
        cur_pos(1) = msg->point.y; 
        cur_pos(2) = msg->point.z;

        // ROS_INFO_STREAM("Current position: " << "(" << cur_pos[0] << ", " << cur_pos[1] << ", " << cur_pos[2] <<")");
    }
}

void LQRController::receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    if (msg){
        cur_vel(0) = msg->vector.x;
        cur_vel(1) = msg->vector.y;
        cur_vel(2) = msg->vector.z;
    }
}

void LQRController::configPublishers(){
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);

}


void LQRController::sendCmdVel(double h){

    double K_x1 = 3.0465, K_x2 = 2.7542 ; // Gain calculated using idare in the Matlab file for the G_x = 1.3/s(s + 1.5)

    Eigen::Vector3f u;

    u.x() = -K_x1*cur_pos.x() - K_x2*cur_vel.x();
    u.y() = -K_x1*cur_pos.y() - K_x2*cur_vel.y();
    u.z() = -K_x1*cur_pos.z() - K_x2*cur_vel.z();


    vel_msg.header.stamp = ros::Time::now();
    vel_msg.header.frame_id = "ground_ENU";
    vel_msg.axes = {u.x(), u.y(), u.z(), 0.0, 73};

    vel_pub.publish(vel_msg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "LQT_control_node");
    ROS_INFO("This node has started.");
    LQRController nh;

    nh.configNode();

    ros::Time start_time = ros::Time::now();

    ros::Rate sampling_rate(50);    // Hertz
    while(ros::ok()){

        // ros::Time current_time = ros::Time::now();

        // double h = (current_time - start_time).toSec();
        double h = 1.0/50.0;
        ros::spinOnce();
        nh.sendCmdVel(h);

        sampling_rate.sleep();
    }
}
