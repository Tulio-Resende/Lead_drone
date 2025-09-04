#include "rl_control/LQT-I_control_node.hpp"

/* Constructor */
LQTIController::LQTIController(/* args */): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");
    yss = 2;
    mu.x() = 0;
    mu.y() = 0;
    mu.z() = 0;
}

/* Destructor */
LQTIController::~LQTIController()
{
}

void LQTIController::configNode(){
    configSubscribers();
    configPublishers();
    // ROS_INFO("Node configured.");
}

void LQTIController::configSubscribers()
{
    refGeneratorSub = handle.subscribe("/rl_control/ref_generator", 1, &LQTIController::receiveRef, this);
    curPosSub = handle.subscribe("/dji_sdk/local_position", 1, &LQTIController::receivePos, this);
    curVelSub = handle.subscribe("/dji_sdk/velocity", 1, &LQTIController::receiveVel, this);
    // curRefOutput = handle.subscribe("/rl_control/output_reference", 1, &LQTIController::receiveRefOut, this);
    // curModelOutput = handle.subscribe("/rl_control/output_model", 1, &LQTIController::receiveModelOut, this);
}

// void LQTIController::receiveRefOut(const geometry_msgs::Vector3::ConstPtr& msg)
// {
//     if (msg){
//         cur_ref_output(0) = msg->x; 
//         cur_ref_output(1) = msg->y; 
//         cur_ref_output(2) = msg->z;

//         ROS_INFO("Node configured. ref[%d] = %f", cur_ref_output(0));
//     }
// }

// void LQTIController::receiveModelOut(const geometry_msgs::Vector3::ConstPtr& msg)
// {
//     if (msg){
//         cur_model_output(0) = msg->x; 
//         cur_model_output(1) = msg->y; 
//         cur_model_output(2) = msg->z;

//         ROS_INFO("Node configured. ref[%d] = %f", cur_model_output(0));
//     }
// }

void LQTIController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (msg){
        cur_pos.x() = msg->point.x; 
        cur_pos.y() = msg->point.y; 
        cur_pos.z() = msg->point.z;

        // ROS_INFO_STREAM("Current position: " << "(" << cur_pos[0] << ", " << cur_pos[1] << ", " << cur_pos[2] <<")");
    }
}

void LQTIController::receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    if (msg){
        cur_vel.x() = msg->vector.x;
        cur_vel.y() = msg->vector.y;
        cur_vel.z() = msg->vector.z;
    }
}


void LQTIController::receiveRef(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    
    for (int i = 0; i < 4; i++) {
        ref_msg[i] = msg->data[i];
        // ROS_INFO("Node configured. ref[%d] = %f", i, ref_msg[i]);
    }
}
void LQTIController::configPublishers(){
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);

}


void LQTIController::sendCmdVel(double h){

    // Gain calculated using idare in the Matlab file for the G_x = 1.3/s(s + 1.5) 
    // augmented with the reference model and integral action

    

    double K_x1 = 20.7726, K_x2 = 4.3473; 

    double k_mu = 0;
    
    double Kx_m1 = -9.4852, Kx_m2 = 0.0635, Kx_m3 = -0.2125, Kx_m4 = 0.0013;

    double Ky_m1 = 0.2929, Ky_m2 = 0.5801, Ky_m3 = 0.0089 , Ky_m4 = 0.0177;

    Eigen::Vector3f u;

    // - k_mu*mu.x() 
    // - k_mu*mu.y() 
    u.x() = -K_x1*cur_pos.x() - K_x2*cur_vel.x() - k_mu*mu.x() - Kx_m1*ref_msg[0] - Kx_m2*ref_msg[1] - Kx_m3*ref_msg[2] - Kx_m4*ref_msg[3];
    u.y() = -K_x1*cur_pos.y() - K_x2*cur_vel.y() - k_mu*mu.y() - Ky_m1*ref_msg[0] - Ky_m2*ref_msg[1] - Ky_m3*ref_msg[2] - Ky_m4*ref_msg[3];
    u.z() = 0.0;

    vel_msg.header.stamp = ros::Time::now();
    vel_msg.header.frame_id = "ground_ENU";
    vel_msg.axes = {u.x(), u.y(), u.z(), 0.0, 73};

    vel_pub.publish(vel_msg);

    mu.x() = mu.x() + h*(yss - cur_pos.x());
    mu.y() = mu.y() + h*(yss - cur_pos.y());

    ROS_INFO("mu = %f", mu.x());


}

int main(int argc, char **argv){

    ros::init(argc, argv, "LQT_control_node");
    ROS_INFO("This node has started.");
    LQTIController nh;

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
