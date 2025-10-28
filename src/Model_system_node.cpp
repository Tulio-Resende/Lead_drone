#include "rl_control/Model_system_node.hpp"
#include "rl_control/param_loader.hpp"


/* Constructor */
ModelSystem::ModelSystem(ros::NodeHandle& nh): 
priv_handle("~")
{
    count = 0;

    loadMatrix(nh, "Ad", Ad);
    loadMatrix(nh, "Bd", Bd);
    loadMatrix(nh, "Cd", Cd);
    loadVector(nh, "x0x", state_x);
    loadVector(nh, "x0y", state_y);


    ROS_INFO_STREAM("Ad" << Ad);
    ROS_INFO_STREAM("Bd" << Bd);
    ROS_INFO_STREAM("Cd" << Cd);
    ROS_INFO_STREAM("state_x" << state_x);
    ROS_INFO_STREAM("state_y" << state_y);

    // Ad << 
    // 1.0000,    0.0197,
    // 0,         0.9743;
   
    // state_x << 9, 0;  // Initial Condition
    // state_y << 2, 0;

    // Cd << 1, 0;

    // Bd <<
    // 0.0003, 0.0257;


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
    configService();
    // ROS_INFO("Node configured.");
}


void ModelSystem::configSubscribers()
{
    controlSignal = handle.subscribe("/dji_sdk/flight_control_setpoint_generic", 1, &ModelSystem::receiveControlSignal, this);
    
}

void ModelSystem::configService()
{
    service_model = handle.advertiseService("/rl_control/enable_control", &ModelSystem::handleEnable, this);
    
}

bool ModelSystem::handleEnable(std_srvs::SetBool::Request &req,
                  std_srvs::SetBool::Response &res)
{
    if (req.data)
    {
        ROS_INFO("Model Reference On");
        res.success = true;
        res.message = "Model Reference On";
    }
    else
    {
        ROS_WARN("Model Reference Fail!");
        res.success = false;
        res.message = "Model Reference Fail!";
    }

    control_enabled = res.success;

    return true;
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

    if (control_enabled == true)
    {
        // ROS_INFO_STREAM("count" << count);

        output_model.x = (Cd * state_x).value();
        output_model.y = (Cd * state_y).value();

        outModelPub.publish(output_model);

        cur_pos.point.x = state_x(0);
        cur_vel.vector.x = state_x(1);

        cur_pos.point.y = state_y(0);
        cur_vel.vector.y = state_y(1);

        cur_pos.header.stamp = ros::Time::now();
        cur_vel.header.stamp = ros::Time::now();
       
        curPosPub.publish(cur_pos);
        curVelPub.publish(cur_vel);

        // TODO: Check the initial condition

        // if(count > 1000)
        // {
        //     Ad << 
        //         1.0000,    0.0192,
        //         0,         0.9231;

            
        //      Bd << 0.0000, 0.077;

        //     // ROS_INFO_STREAM("Aqui!!!");
        
        // }
        state_x = Ad * state_x + Bd * control_signal.x;
        state_y = Ad * state_y + Bd * control_signal.y;

        // ROS_INFO("velocity = %f", state_x(0));

        // count = count + 1;
    }
    
}


int main(int argc, char **argv){

    ros::init(argc, argv, "Model_System_node");
    ROS_INFO("This node has started.");

    ros::NodeHandle nh;

    ModelSystem model_sys(nh);

    model_sys.configNode();

    ros::Time start_time = ros::Time::now();

    ros::Rate sampling_rate(50);    // Hertz
    while(ros::ok()){

        // ros::Time current_time = ros::Time::now();

        // double h = (current_time - start_time).toSec();
        double h = 1.0/50.0;
        ros::spinOnce();
        model_sys.sendModelStates(h);

        sampling_rate.sleep();
    }
}
