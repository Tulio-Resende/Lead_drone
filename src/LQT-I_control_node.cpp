#include "rl_control/LQT-I_control_node.hpp"

/* Constructor */
LQTIController::LQTIController(/* args */): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");
    cur_pos = Eigen::Vector3d::Zero();
    yss = 2;
    tilde_mu.x() = 0;
    tilde_mu.y() = 0;
    // tilde_mu.z() = 0;
    old_pos.x() = 0;
    old_pos.y() = 0;
    old_vel.x() = 0;
    old_vel.y() = 0;
    tilde_pos.x() = 0;   //initial condition
    tilde_pos.y() = 0;
    tilde_vel.x() = 0;
    tilde_vel.y() = 0;
    old_u.x() = 0;
    old_u.y() = 0;
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
}

void LQTIController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (msg){
        cur_pos.x() = msg->point.x; 
        cur_pos.y() = msg->point.y; 
        cur_pos.z() = msg->point.z;

        ROS_INFO("Posx %f", cur_pos.x());
        ROS_INFO("Posy %f", cur_pos.y());

        flag = true;

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
    if (flag){
        tilde_mu.x() = h*(yss - old_pos.x());
        tilde_mu.y() = h*(yss - old_pos.y());

        tilde_pos.x() = cur_pos.x() - old_pos.x();
        tilde_pos.y() = cur_pos.y() - old_pos.y();

        // ROS_INFO_STREAM("Til Pos: " << tilde_pos);

        tilde_vel.x() = cur_vel.x() - old_vel.x();
        tilde_vel.y() = cur_vel.y() - old_vel.y();
    
        double K_x1 = 3.7832 , K_x2 =  1.3718; 

        double k_mu = -3.2953 ;
        // double k_mu = 0;

                                
                    
        double Kx_m1 = -0.4564, Kx_m2 = 0.0077, Kx_m3 = -0.0095, Kx_m4 = 0.0001;

        double Ky_m1 = 0.0156, Ky_m2 = 0.0139, Ky_m3 = 0.0007, Ky_m4 = 0.0006;

        // ROS_INFO_STREAM("u de x: " << -K_x1*tilde_pos.x());
        tilde_u.x() = -K_x1*tilde_pos.x() - K_x2*tilde_vel.x() - k_mu*tilde_mu.x() - Kx_m1*ref_msg[0] - Kx_m2*ref_msg[1] - Kx_m3*ref_msg[2] - Kx_m4*ref_msg[3];
        tilde_u.y() = -K_x1*tilde_pos.y() - K_x2*tilde_vel.y() - k_mu*tilde_mu.y() - Ky_m1*ref_msg[0] - Ky_m2*ref_msg[1] - Ky_m3*ref_msg[2] - Ky_m4*ref_msg[3];
        tilde_u.z() = 0.0;

        // ROS_INFO_STREAM("U til: " << tilde_u);

        u.x() = old_u.x() + tilde_u.x();
        u.y() = old_u.y() + tilde_u.y();
        u.z() = 0.0;

        // ROS_INFO_STREAM("U: " << u);
    
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0, 73};

        vel_pub.publish(vel_msg);


        old_pos.x() = cur_pos.x();
        old_pos.y() = cur_pos.y();
        old_vel.x() = cur_vel.x();
        old_vel.y() = cur_vel.y();
        old_u.x() = u.x();
        old_u.y() = u.y();
    }


 
    // ROS_INFO("mu = %f", mu.x());
    // ROS_INFO("mu = %f", mu.y());


}

int main(int argc, char **argv){

    ros::init(argc, argv, "LQTI_control_node");
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
