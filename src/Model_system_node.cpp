#include "rl_control/Model_system_node.hpp"

/* Constructor */
ModelSystem::ModelSystem(/* args */): 
priv_handle("~")
{

    Ad << 
    1.0000,    0.0197,
    0,         0.9704;
   
    state_x << 0, 0;  // Initial Condition
    state_z << 0, 0;

    Cd << 1, 0;

    Bd <<
    0.0003, 0.0256;


    ROS_DEBUG("Constructor called.");
}

/* Destructor */
ModelSystem::~ModelSystem()
{
}

void ModelSystem::configNode()
{
    configPublishers();
    configSubscribers();
    // ROS_INFO("Node configured.");
}


void ModelSystem::configSubscribers()
{
    controlSignal = handle.subscribe("/dji_sdk/flight_control_setpoint_generic", 1, &ModelSystem::receiveControlSignal, this);
    
}


void ModelSystem::configPublishers(){
    curPosPub = handle.advertise<geometry_msgs::PointStamped>("/dji_sdk/local_position", 1);
    curVelPub = handle.advertise<geometry_msgs::Vector3Stamped>("/dji_sdk/velocity", 1);
    outModelPub = handle.advertise<geometry_msgs::Vector3>("/rl_control/output_model", 1);

}

void ModelSystem::receiveControlSignal(const sensor_msgs::Joy::ConstPtr& msg)
{
    
    control_signal.x = msg->axes[0];
    control_signal.y = msg->axes[1];
    control_signal.z = msg->axes[2];

    ROS_INFO("Control Signal = %f", control_signal.x);
    ROS_INFO("Control Signal = %f", control_signal.y);

}


void ModelSystem::sendModelStates(double h){

    Eigen::Vector3d out;
    out.x() = Cd * state_x;
    // out.y() = Cd * state_y;

    output_model.x = out.x();
    // outputModel.y = out.y();

    outModelPub.publish(output_model);

    //TODO: Check the initial condition

    state_x = Ad * state_x + Bd * control_signal.x;
    state_y = Ad * state_y + Bd * control_signal.y;

    // ROS_INFO("velocity = %f", state_x(0));

    
    cur_pos.point.x = state_x(0);
    cur_vel.vector.x = state_x(1);

    cur_pos.point.y = state_y(0);
    cur_vel.vector.y = state_y(1);

    
    curPosPub.publish(cur_pos);
    curVelPub.publish(cur_vel);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "Model_System_node");
    ROS_INFO("This node has started.");
    ModelSystem nh;

    nh.configNode();

    ros::Time start_time = ros::Time::now();

    ros::Rate sampling_rate(50);    // Hertz
    while(ros::ok()){

        // ros::Time current_time = ros::Time::now();

        // double h = (current_time - start_time).toSec();
        double h = 1.0/50.0;
        ros::spinOnce();
        nh.sendModelStates(h);

        sampling_rate.sleep();
    }
}
