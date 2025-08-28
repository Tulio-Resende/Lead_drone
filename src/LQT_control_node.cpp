#include "rl_control/LQT_control_node.hpp"

/* Constructor */
LQTController::LQTController(/* args */): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");
}

/* Destructor */
LQTController::~LQTController()
{
}

void LQTController::configNode(){
    configSubscribers();
    configPublishers();
    // ROS_INFO("Node configured.");
}

void LQTController::configSubscribers()
{
    refGeneratorSub = handle.subscribe("/rl_control/ref_generator", 1, &LQTController::receiveRef, this);
    curPosSub = handle.subscribe("/dji_sdk/local_position", 1, &LQTController::receivePos, this);
    curVelSub = handle.subscribe("/dji_sdk/velocity", 1, &LQTController::receiveVel, this);
}

void LQTController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (msg){
        cur_pos(0) = msg->point.x; 
        cur_pos(1) = msg->point.y; 
        cur_pos(2) = msg->point.z;

        // ROS_INFO_STREAM("Current position: " << "(" << cur_pos[0] << ", " << cur_pos[1] << ", " << cur_pos[2] <<")");
    }
}

void LQTController::receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    if (msg){
        cur_vel(0) = msg->vector.x;
        cur_vel(1) = msg->vector.y;
        cur_vel(2) = msg->vector.z;
    }
}


void LQTController::receiveRef(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    
    for (int i = 0; i < 5; i++) {
        ref_msg[i] = msg->data[i];
        // ROS_INFO("Node configured. ref[%d] = %f", i, ref_msg[i]);
    }
}
void LQTController::configPublishers(){
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);

}


void LQTController::sendCmdVel(double h){


    // Gain calculated using idare in the Matlab file for the G_x = 1.3/s(s + 1.5)
    double K_x1 = 0.5266, K_x2 = 0.2624 ; 

    double Kx_m1 = -4.6666, Kx_m2 = 0.0719, Kx_m3 = -0.1128 , Kx_m4 = 0.0013, Kx_m5 = -0.0003;

    double Ky_m1 = -0.9052, Ky_m2 = 0.1117, Ky_m3 = -0.0309 , Ky_m4 = 0.0050, Ky_m5 = -0.0003;


    Eigen::Vector3f u;

    u.x() = -K_x1*cur_pos.x() - K_x2*cur_vel.x() - Kx_m1*ref_msg[0] - Kx_m2*ref_msg[1] - Kx_m3*ref_msg[2] - Kx_m4*ref_msg[3] - Kx_m5*ref_msg[4];
    u.y() = -K_x1*cur_pos.y() - K_x2*cur_vel.y() - Ky_m1*ref_msg[0] - Ky_m2*ref_msg[1] - Ky_m3*ref_msg[2] - Ky_m4*ref_msg[3] - Ky_m5*ref_msg[4];

    u.z() = 0.0;

    vel_msg.header.stamp = ros::Time::now();
    vel_msg.header.frame_id = "ground_ENU";
    vel_msg.axes = {u.x(), u.y(), u.z(), 0.0, 73};

    vel_pub.publish(vel_msg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "LQT_control_node");
    ROS_INFO("This node has started.");
    LQTController nh;

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
