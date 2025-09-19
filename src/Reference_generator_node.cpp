#include "rl_control/Reference_generator_node.hpp"

/* Constructor */
ReferenceGenerator::ReferenceGenerator(/* args */): 
priv_handle("~")
{

 //Am=expm(Amc*h); Cm=Cmc; xm0=Bmc;
   
    // Am << 
    // 0.999992870110446,	    -0.000712988288670474,	-5.41441701278375e-08,	-5.41441057874235e-06,
    // 0.0199999524673852,	    0.999992870110446,	    -3.60961305760061e-10,	-5.41441701278375e-08,
    // 0.000199999762336882,	0.0199999524673852,	    0.999999999998195,	    -3.60961305760061e-10,
    // 1.33333238268077e-06,	0.000199999762336882,	0.0199999999999928,	    0.999999999998195;

    // xm << 1, 0, 0, 0;
    


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

 

    // Cmx <<
    // 7.000, 0, 0.1453, 0;

    // Cmy <<
    // 0, -0.2095, 0, -0.0095;

    ROS_DEBUG("Constructor called.");
}

/* Destructor */
ReferenceGenerator::~ReferenceGenerator()
{
}

void ReferenceGenerator::configNode()
{
    configPublishers();
    configService();
    // ROS_INFO("Node configured.");
}


void ReferenceGenerator::configPublishers(){
    ref_cmd_pub = handle.advertise<std_msgs::Float64MultiArray>("/rl_control/ref_generator", 1);
    out_ref_pub = handle.advertise<geometry_msgs::Vector3>("/rl_control/output_reference", 1);

}

void ReferenceGenerator::configService(){
    client_ref = handle.serviceClient<std_srvs::SetBool>("/rl_control/enable_control");
}

bool ReferenceGenerator::enableControl(bool enable) {
    std_srvs::SetBool srv;
    srv.request.data = enable;

    if (client_ref.call(srv)) {
        ROS_INFO_STREAM("Resposta do serviço: " << srv.response.message);
        return srv.response.success;
    } else {
        ROS_ERROR("Falha ao chamar serviço enable_control");
        return false;
    }
}

void ReferenceGenerator::sendRefPos(double h){

    ref_msg.data.resize(5);
   
    for (int i = 0; i < 5; i++) 
    {
        ref_msg.data[i] = xm(i);  
    }
    ref_cmd_pub.publish(ref_msg);

    Eigen::Vector3d out;
    out.x() = Cmx * xm;
    out.y() = Cmy * xm;

    // output_ref.x = out.x() + 2; //+ yss
    // output_ref.y = out.y() + 2; //+ yss

    output_ref.x = out.x(); //+ yss
    output_ref.y = out.y(); //+ yss


    out_ref_pub.publish(output_ref);

    xm = Am * xm;
     
}




int main(int argc, char **argv){

    ros::init(argc, argv, "Reference_generator_node");
    ROS_INFO("This node has started.");
    ReferenceGenerator nh;

    nh.configNode();

    // // === Chama o serviço apenas uma vez ===
    // if (!nh.enableControl(true)) {
    //     ROS_ERROR("Não foi possível habilitar o controle. Encerrando nó.");
    //     return -1;  // Finaliza se falhar
    // }

    // ROS_INFO("Controle habilitado com sucesso! Iniciando geração de referência...");


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
