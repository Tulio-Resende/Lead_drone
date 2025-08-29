#include "rl_control/Reference_generator_node.hpp"

/* Constructor */
ReferenceGenerator::ReferenceGenerator(/* args */): 
priv_handle("~")
{

 //Am=expm(Amc*h); Cm=Cmc; xm0=Bmc;
   
    Am << 
    0.999992870110446,	    -0.000712988288670474,	-5.41441701278375e-08,	-5.41441057874235e-06,	0,
    0.0199999524673852,	    0.999992870110446,	    -3.60961305760057e-10,	-5.41441701278375e-08,	0,
    0.000199999762336881,	0.0199999524673852,	    0.999999999998195,	    -3.60961305760057e-10,	0,
    1.33333238268076e-06,	0.000199999762336881,	0.0199999999999928,	    0.999999999998195,	    0,
    6.66666349782453e-09,	1.33333238268076e-06,	0.000199999999999976,	0.0199999999999928,	    1;
   
    xm << 1, 0, 0, 0, 0;  // Initial Condition
    
    Cmx <<
    9.0000, 0, 0.2166, 0, 0.0005;

    Cmy <<
    2.0000, -0.2095, 0.0713, -0.0095, 0.0005;

    ROS_DEBUG("Constructor called.");
}

/* Destructor */
ReferenceGenerator::~ReferenceGenerator()
{
}

void ReferenceGenerator::configNode()
{
    configPublishers();
    // ROS_INFO("Node configured.");
}


void ReferenceGenerator::configPublishers(){
    ref_cmd_pub = handle.advertise<std_msgs::Float32MultiArray>("/rl_control/ref_generator", 1);
    out_ref_pub = handle.advertise<geometry_msgs::Vector3>("/rl_control/output_reference", 1);

}


void ReferenceGenerator::sendRefPos(double h){

    Eigen::Vector3d out;
    out.x() = Cmx * xm;
    out.y() = Cmy * xm;

    output_ref.x = out.x();
    output_ref.y = out.y();

    xm = Am * xm;
    
  

    out_ref_pub.publish(output_ref);

    ref_msg.data.resize(5);
   
    for (int i = 0; i < 5; i++) 
    {
        ref_msg.data[i] = xm(i);  
    }
    ref_cmd_pub.publish(ref_msg);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "Reference_generator_node");
    ROS_INFO("This node has started.");
    ReferenceGenerator nh;

    nh.configNode();

    ros::Time start_time = ros::Time::now();

    ros::Rate sampling_rate(50);    // Hertz
    while(ros::ok()){

        // ros::Time current_time = ros::Time::now();

        // double h = (current_time - start_time).toSec();
        double h = 1.0/50.0;
        ros::spinOnce();
        nh.sendRefPos(h);

        sampling_rate.sleep();
    }
}
