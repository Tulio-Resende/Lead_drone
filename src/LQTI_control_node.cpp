#include "rl_control/LQTI_control_node.hpp"
#include "rl_control/param_loader.hpp"


/* Constructor */
LQTIController::LQTIController(ros::NodeHandle& nh): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");

    nh.getParam("yss", yss);
    loadMatrix(nh, "Kx_LQTI", Kx);
    loadMatrix(nh, "Ky_LQTI", Ky);
    loadMatrix(nh, "Cd", Cd);

    int n_state = 2;
    old_u = Eigen::Vector3d::Zero(3);
    tilde_u = Eigen::Vector3d::Zero(3);

    old_xDOF = Eigen::VectorXd::Zero(n_state);
    old_yDOF = Eigen::VectorXd::Zero(n_state);
    z_tilde_xDOF = Eigen::VectorXd::Zero(7);
    z_tilde_yDOF = Eigen::VectorXd::Zero(7);

    old_y = Eigen::VectorXd::Zero(3);

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

void LQTIController::configPublishers(){
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);

}

void LQTIController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (msg){
        cur_pos.x() = msg->point.x; 
        cur_pos.y() = msg->point.y; 
        cur_pos.z() = msg->point.z;

        // ROS_INFO("Posx %f", cur_pos.x());
        // ROS_INFO("Posy %f", cur_pos.y());

        flag_pos = true;

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
    flag_vel = true;
    // ROS_INFO_STREAM("cur vel" << cur_vel);

}

void LQTIController::receiveRef(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    ref_msg.resize(msg->data.size());

    for (int i = 0; i < 4; i++) {
        ref_msg(i) = msg->data[i];
        // ROS_INFO("Node configured. ref[%d] = %f", i, ref_msg[i]);
    }
    flag_ref = true;
    // ROS_INFO_STREAM("ref" << ref_msg);

}

void LQTIController::sendCmdVel(double h){


    if (flag_pos && flag_ref && flag_vel){

        if(flag_second_time)
        {

            old_xDOF << old_pos.x(), old_vel.x();
            old_yDOF << old_pos.y(), old_vel.y();

            old_y.x() = (Cd * old_xDOF)(0);
            old_y.y() = (Cd * old_yDOF)(0);

            tilde_mu.x() = h*(yss - old_y.x());
            tilde_mu.y() = h*(yss - old_y.y());

            tilde_pos = cur_pos - old_pos;
            tilde_vel = cur_vel - old_vel;

            z_tilde_xDOF << tilde_pos.x(), tilde_vel.x(), tilde_mu.x(), ref_msg;
            z_tilde_yDOF << tilde_pos.y(), tilde_vel.y(), tilde_mu.y(), ref_msg;

            // tilde_u.x() = -Kx.segment(0,3).dot(tilde_state_x) - Kx.segment(3,4).dot(ref_msg);
            // tilde_u.y() = -Ky.segment(0,3).dot(tilde_state_y) - Ky.segment(3,4).dot(ref_msg);
            // tilde_u.z() = 0.0;

            tilde_u.x() = - Kx * z_tilde_xDOF;
            tilde_u.y() = - Ky * z_tilde_yDOF;
            tilde_u.z() = 0.0;


        }

        u.x() = old_u.x() + tilde_u.x();
        u.y() = old_u.y() + tilde_u.y();
        u.z() = 0.0;

        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {static_cast<float>(u.x()), static_cast<float>(u.y()), static_cast<float>(u.z()), 0.0, 73};

        vel_pub.publish(vel_msg);

        old_pos = cur_pos;
        old_vel = cur_vel;
        old_u = u;
      
        flag_second_time = true;

    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "LQTI_control_node");
    ROS_INFO("This node has started.");

    ros::NodeHandle nh;
    LQTIController lqti(nh);

    lqti.configNode();

    ros::Time start_time = ros::Time::now();

    ros::Rate sampling_rate(50);    // Hertz
    while(ros::ok()){

        double h = 1.0/50.0;
        ros::spinOnce();
        lqti.sendCmdVel(h);

        sampling_rate.sleep();
    }
}
