#include "rl_control/LQTI_control_node.hpp"
#include "rl_control/param_loader.hpp"


/* Constructor */
LQTIController::LQTIController(ros::NodeHandle& nh): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");

    int n_state = 2;
    cur_pos = Eigen::Vector3d::Zero();
    tilde_mu =  Eigen::Vector3d::Zero();
    old_pos = Eigen::Vector3d::Zero();
    old_vel = Eigen::Vector3d::Zero();
    tilde_pos = Eigen::Vector3d::Zero();
    tilde_vel = Eigen::Vector3d::Zero();
    old_u = Eigen::Vector3f::Zero(3);
    old_state_x = Eigen::VectorXd::Zero(n_state);
    old_state_y = Eigen::VectorXd::Zero(n_state);
    old_y = Eigen::VectorXd::Zero(3);
    tilde_state_x = Eigen::VectorXd::Zero(3);
    tilde_state_y = Eigen::VectorXd::Zero(3);


    // nh.getParam("Qe_LQTI", Qe);
    // nh.getParam("R_LQTI", R);
    // nh.getParam("gamma_LQTI", gamma);
    nh.getParam("yss", yss);

    loadMatrix(nh, "Kx_LQTI", Kx);
    loadMatrix(nh, "Ky_LQTI", Ky);
    loadMatrix(nh, "Cd", Cd);
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

    // Gain calculated using idare in the Matlab file for the G_x = 1.3/s(s + 1.5) 
    // augmented with the reference model and integral action
    if (flag_pos && flag_ref && flag_vel){

        if(flag_first_time)
        {
            old_pos = cur_pos;
            old_vel = cur_vel;
            ROS_INFO_STREAM("old_pos" << old_pos);
            ROS_INFO_STREAM("old_vel"<< old_vel);

            ROS_INFO_STREAM("test");
        }

        old_state_x << old_pos.x(), old_vel.x();
        old_state_y << old_pos.y(), old_vel.x();

        old_y.x() = (Cd * old_state_x)(0);
        old_y.y() = (Cd * old_state_y)(0);

        ROS_INFO_STREAM("old_y" << old_y.x());


        tilde_mu.x() = h*(yss - old_y.x());
        tilde_mu.y() = h*(yss - old_y.y());

        ROS_INFO_STREAM("tilde_mu" << tilde_mu.x());

        tilde_pos = cur_pos - old_pos;
        // tilde_pos.x() = cur_pos.x() - old_pos.x();
        // tilde_pos.y() = cur_pos.y() - old_pos.y();
        ROS_INFO_STREAM("tilde_pos" << tilde_pos);


        tilde_vel = cur_vel - old_vel;

        ROS_INFO_STREAM("tilde_vel" << tilde_vel);

        // tilde_vel.x() = cur_vel.x() - old_vel.x();
        // tilde_vel.y() = cur_vel.y() - old_vel.y();
        tilde_state_x << tilde_pos.x(), tilde_vel.x(), tilde_mu.x();
        tilde_state_y << tilde_pos.y(), tilde_vel.y(), tilde_mu.y();

        ROS_INFO_STREAM("tilde_state_x" << tilde_state_x);

        tilde_u.x() = static_cast<float>(-Kx.segment(0,3).dot(tilde_state_x)
                                 - Kx.segment(3,4).dot(ref_msg));

        tilde_u.y() = static_cast<float>(-Ky.segment(0,3).dot(tilde_state_y)
                                 - Ky.segment(3,4).dot(ref_msg));

        ROS_INFO_STREAM("tilde_u" << tilde_u.x());


        // tilde_u.x() = - Kx.segment(0,3) * tilde_state.x() - Kx.segment(3,7) * ref_msg;
        // tilde_u.y() = - Ky.segment(0,3) * tilde_state.y() - Ky.segment(3,7) * ref_msg;
        tilde_u.z() = 0.0;

        // ROS_INFO_STREAM("U til: " << tilde_u);

        u.x() = old_u.x() + tilde_u.x();
        u.y() = old_u.y() + tilde_u.y();
        u.z() = 0.0;

        ROS_INFO_STREAM("u" << u);

    
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0, 73};

        vel_pub.publish(vel_msg);

        old_pos = cur_pos;
        old_vel = cur_vel;
        old_u =  u;
        flag_first_time = false;
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
