#include "rl_control/Reference_generator_node.hpp"

/* Constructor */
ReferenceGenerator::ReferenceGenerator(/* args */): 
priv_handle("~")
{
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
    out_ref_pub = handle.advertise<geometry_msgs::Vector3Stamped>("/rl_control/output_reference", 1);

}

void ReferenceGenerator::configService(){
    client_ref = handle.serviceClient<std_srvs::SetBool>("/rl_control/enable_control");
}

bool ReferenceGenerator::waitForPlant(bool enable) {
    // std_srvs::SetBool srv;
    srv.request.data = enable;

    ROS_INFO("Esperando a planta responder...");
    client_ref.waitForExistence(); // Bloqueia até o serviço aparecer

    if (client_ref.call(srv)) {
        ROS_INFO_STREAM("Resposta do serviço: " << srv.response.message);
        return srv.response.success;
    } else {
        ROS_ERROR("Falha ao chamar serviço enable_control");
        return false;
    }
}

Eigen::Vector3d ReferenceGenerator::Excitation(double& t)
{
    // // white noise
    Eigen::Vector3d noise_white, noise_sin, excitation;

    // noise_white.x() = dist(generator);

    // // sinuidal noise

    // noise_sin.x() =   0.5 * sin(2*M_PI*0.5*t)
    //                 + 0.4 * sin(2*M_PI*2.0*t)
    //                 + 0.3 * sin(2*M_PI*5.0*t)
    //                 + 0.2 * sin(2*M_PI*7.0*t);

    std::default_random_engine generator;
    std::normal_distribution<double> white_dist{0.0, 1.0}; // unit variance, escalar depois
    std::vector<double> sine_freqs = {0.2, 0.5, 1.0, 2.5}; // Hz
    std::vector<double> sine_amps  = {0.12, 0.08, 0.05, 0.03}; // amplitudes
    double exc = 0.0;
    // soma de senóides
    for (size_t i=0;i<sine_freqs.size();++i) {
        exc += sine_amps[i] * sin(2.0*M_PI * sine_freqs[i] * t + 0.0 /*fase opcional*/);
    }
    // ruído branco filtrado (simples: ruído baixo-pass via média móvel)
    double white = white_dist(generator) * 0.05; // escala do ruído
    // opcional: filtro simples (exponential lowpass)
    static double lp_prev = 0.0;
    double alpha = 0.1; // 0..1 (menor = mais filtrado)
    lp_prev = alpha * white + (1.0 - alpha) * lp_prev;
    exc += lp_prev;

    // opcional: adicionar PRBS (simples)
    // static double prbs_last = 0; static double prbs_tlast = 0;
    // if (t - prbs_tlast > prbs_step) { prbs_last = (rand()&1)? +amp : -amp; prbs_tlast = t; } exc += prbs_last;

    excitation.x() = exc;
    
    // ROS_INFO_STREAM("excitation" << excitation);
    return excitation;
}

void ReferenceGenerator::sendRefPos(double h){

    static double t = 0.0;
    t += h;
   
    output_ref.header.stamp = ros::Time::now();
    output_ref.vector.x  = Cmx * xm; //+ yss
    output_ref.vector.y = Cmy * xm; //+ yss
    // output_ref.x = Cmx * xm; //+ yss
    // output_ref.y = Cmy * xm; //+ yss

    // ROS_INFO_STREAM("out" << output_ref);

    out_ref_pub.publish(output_ref);

    ref_msg.data.resize(5);
   
    for (int i = 0; i < 5; i++) 
    {
        ref_msg.data[i] = xm(i);  
    }
    
    ref_cmd_pub.publish(ref_msg);

    xm = Am * xm;
     
}

int main(int argc, char **argv){

    ros::init(argc, argv, "Reference_generator_node");
    ROS_INFO("This node has started.");
    ReferenceGenerator nh;

    nh.configNode();

    // === Chama o serviço apenas uma vez ===
    if (!nh.waitForPlant(true)) {
        ROS_ERROR("Não foi possível habilitar o controle. Encerrando nó.");
        return -1;  // Finaliza se falhar
    }

    ROS_INFO("Controle habilitado com sucesso! Iniciando geração de referência...");

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
