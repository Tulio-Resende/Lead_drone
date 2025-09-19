#include "rl_control/RL_LQT_control_node.hpp"

/* Constructor */
RLLQTController::RLLQTController(/* args */): 
priv_handle("~"), dist(0.0, 0.2)
{
    ROS_DEBUG("Constructor called.");

         
    Cmx <<
    9.0000, 0, 0.2166, 0, 0.0005;

    Cmy <<
    2.0000, -0.2095, 0.0713, -0.0095, 0.0005;

    Cd << 1, 0;


    ref_msg = Eigen::VectorXd::Zero(5);

    int n_state = 2;
    int n_state_aug = n_state + ref_msg.size();
    int n_state_aug_control = n_state + ref_msg.size() + 1;
    int n_param = (n_state_aug_control)*(n_state_aug_control + 1)/2;

    state_x = Eigen::VectorXd::Zero(n_state_aug);
    state_y = Eigen::VectorXd::Zero(n_state_aug);
    old_state_x = Eigen::VectorXd::Zero(n_state_aug);
    old_state_y = Eigen::VectorXd::Zero(n_state_aug);

    augmented_state_x = Eigen::VectorXd::Zero(n_state_aug_control);
    augmented_state_y = Eigen::VectorXd::Zero(n_state_aug_control);
    old_augmented_state_x = Eigen::VectorXd::Zero(n_state_aug_control);
    old_augmented_state_y = Eigen::VectorXd::Zero(n_state_aug_control);

    //TODO: find a better way to choose the vector size
    bar_x = Eigen::VectorXd::Zero(n_param);
    old_bar_x = Eigen::VectorXd::Zero(n_param);
    bar_y = Eigen::VectorXd::Zero(n_param);
    old_bar_y = Eigen::VectorXd::Zero(n_param);



    theta = Eigen::VectorXd::Zero(n_param);

    theta << 52.6334101099504, 10.2162277797894, 4188.82853290319, 1.03540441873538, 2.43711211624183, 0.000346393806324802, 1.54300006451501e-05, 1.00683497461433, 41.2049038660059, -938.638001034116, 9.95443496971824, -22.6452554201491, 0.181374103651703, -0.0569959139571596, 1.06043943182456, -365.189566987842, 5.58149507522191, -8.82355433095529, 0.101742357141362, -0.0223100797616370, 0.528471573951173, -86.2735882200421, 202.070159782202, -1.57122736630554, 0.508218360088234, -9.39701005855973, -2.09592364821546, 0.0378744331055520, -0.00538975260999602, 0.144703357204055, -0.0381754164895553, 0.0122611001906250, -0.227054969366253, -9.82036199459057e-05, 0.00263774819632151, -0.000574166812361116;

    ROS_INFO_STREAM("theta init" << theta);

    Kx = Eigen::RowVectorXd::Zero(n_state_aug);
    Ky = Eigen::RowVectorXd::Zero(n_state_aug);
    // kz = Eigen::RowVectorXd::Zero(n_state_aug);

    K0factor = 1.0/10.0;
    THETA0factor = 0.8;
    PRLS0factor=10e6;
    Kx << 0.5266, 0.2624, -4.6666, 0.0719, -0.1128, 0.0013, -0.0003;
    Ky << 0.5266, 0.2624, -0.9052, 0.1117, -0.0309, 0.0050, -0.0003;
    // kz << 0, 0;

    //initial gain
    Kx = Kx * K0factor; 
    Ky = Ky * K0factor;

    ROS_INFO_STREAM("Initial Kx" << Kx);

    theta = theta * THETA0factor;

    countk = 0;

    prls = Eigen::MatrixXd::Identity(theta.size(),theta.size()) * PRLS0factor;
    ROS_INFO_STREAM("PRLS0factor" << PRLS0factor);
}

/* Destructor */
RLLQTController::~RLLQTController()
{
}

void RLLQTController::configNode()
{
    configSubscribers();
    configPublishers();
}

void RLLQTController::configSubscribers()
{
    curPosSub = handle.subscribe("/dji_sdk/local_position", 1, &RLLQTController::receivePos, this);
    curVelSub = handle.subscribe("/dji_sdk/velocity", 1, &RLLQTController::receiveVel, this);
    refGeneratorSub = handle.subscribe("/rl_control/ref_generator", 1, &RLLQTController::receiveRef, this);

}

void RLLQTController::configPublishers()
{
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);
    reward_pub = handle.advertise<std_msgs::Float64>("/rl_control/reward", 1);

}

void RLLQTController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (msg){
        cur_pos.x() = msg->point.x; 
        cur_pos.y() = msg->point.y; 
        cur_pos.z() = msg->point.z;

        flag_pos = true;
    }
}

void RLLQTController::receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    if (msg)
    {
        cur_vel.x() = msg->vector.x;
        cur_vel.y() = msg->vector.y;
        cur_vel.z() = msg->vector.z;

        flag_vel = true;
    }
}
void RLLQTController::receiveRef(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    
    for (int i = 0; i < 5; i++) {
        ref_msg(i) = msg->data[i];
        // ROS_INFO("ref[%d] = %f", i, ref_msg(i));
    }
    flag_ref = true;
}

Eigen::VectorXd RLLQTController::fromx2xbar(const Eigen::VectorXd& x)
{
    int n = x.size();

    int m = n + (n* (n-1))/2;
    Eigen::VectorXd xbar(m);
    
    int k = 0;

    for (int i = 0; i < n; i++) 
    {
        xbar(k++) = x(i) * x(i);
    }

    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) 
        {
            xbar(k++) = x(i) * x(j);
        }
    }
    return -xbar;
}

Eigen::MatrixXd RLLQTController::FromTHETAtoP(const Eigen::VectorXd& theta, int sizeOfAugState)
{

    int N = (sizeOfAugState * (sizeOfAugState + 1))/2;

    Eigen::MatrixXd indx(N, 2);

    int count = 0;

    for (int i = 0; i < sizeOfAugState; i++)
    {
        indx(count, 0) = i;
        indx(count, 1) = i; 
        count++;
    }
  
    for(int i = 0; i < sizeOfAugState; i++)
    {
        for(int j = i+1; j < sizeOfAugState; j++)
        {   
            indx(count, 0) = i;
            indx(count, 1) = j; 
            count++;
        }
    }

    Eigen::MatrixXd Pout1, Pout2, Pout;

    Pout1 = Eigen::MatrixXd::Zero(sizeOfAugState, sizeOfAugState);
    Pout2 = Eigen::MatrixXd::Zero(sizeOfAugState, sizeOfAugState);
    Pout  = Eigen::MatrixXd::Zero(sizeOfAugState, sizeOfAugState);
    int k = 0;

    for (int i = 0; i < sizeOfAugState; i++)
    {
        Pout1(indx(i, 0), indx(i, 1)) = theta(k);
        k++;
    }

    for (int i = sizeOfAugState; i < N; i++)
    {
        Pout2(indx(i, 0), indx(i, 1)) = theta(k)/2;
        k++;
    }

    Pout = Pout1 + Pout2 + Pout2.transpose();

    return Pout; 
}

void RLLQTController::UpdateRLS(Eigen::VectorXd& theta, std::vector<Eigen::VectorXd>& phi, Eigen::Vector3d& Erls, Eigen::MatrixXd& prls, double& mu)
{
    
    double phiT_P_phi = phi[0].transpose() * prls * phi[0];
    if (phiT_P_phi > 1e-6) // evita divisões por zero ou muito pequenas
    {
        theta = theta + (prls * phi[0] * Erls.x()) / (mu + phiT_P_phi);
        prls = (1.0/mu) * prls - (1.0/mu) * (prls * phi[0] * phi[0].transpose() * prls) / (mu + phiT_P_phi);
    }
    ROS_INFO_STREAM("theta" << theta);

}

void RLLQTController::UpdateGain(Eigen::VectorXd& theta)
{
    // Recovering H from theta
    int z = augmented_state_x.size();

    H.resize(3);
    H[0] = FromTHETAtoP(theta, z);

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(H[0]);
    int rank = lu_decomp.rank();
    
    if (rank == z)
    {
        // Update Gain
        //TODO: is there a better way to do that?
        double inv_scalar = 1.0 / H[0](z-1, z-1);

        switch(gain_update)
        {
            case 1:
                Kx = inv_scalar*H[0].row(z-1).segment(0,z-1);
                ROS_INFO_STREAM("Updated Kx: " << Kx);
                break;
            default:
                Eigen::RowVectorXd Kxx;
                Kxx = inv_scalar*H[0].row(z-1).segment(0,z-1);
                ROS_INFO_STREAM("Updated Kxx: " << Kxx);
                break;
        }

        countk = 1;
    }
    else
    {
        ROS_INFO_STREAM("rank ruim" << rank);            
    }
}

Eigen::Vector3d RLLQTController::Excitation(double& t)
{
    // white noise
    Eigen::Vector3d noise_white, noise_sin, excitation;

    noise_white.x() = dist(generator);

    // sinuidal noise

    noise_sin.x() =   0.5 * sin(2*M_PI*0.5*t)
                    + 0.4 * sin(2*M_PI*2.0*t)
                    + 0.3 * sin(2*M_PI*5.0*t)
                    + 0.2 * sin(2*M_PI*7.0*t);

    
    // ROS_INFO_STREAM("excitation" << excitation);
    return excitation = noise_white + noise_sin;
}

void RLLQTController::sendCmdVel(double h){

    if (flag_pos && flag_vel && flag_ref)
    {
        double Qe=1, R=1, gamma = 0.5, mu = 1.0;
        static double t = 0.0;
        t += h;

        if (flag_first_pos)
        {
            old_pos = cur_pos;
            flag_first_pos = false;
        }

        if (flag_first_vel)
        {
            old_vel = cur_vel;
            flag_first_vel = false;
        }
        if (flag_first_ref)
        {
            old_ref_msg = ref_msg;
            flag_first_ref = false;
        }

        old_state_x << old_pos.x(), old_vel.x(), old_ref_msg;
        old_state_y << old_pos.y(), old_vel.y(), old_ref_msg;

        old_u.x() = - Kx * old_state_x;
        old_u.y() = - Ky * old_state_y;

        old_augmented_state_x << old_state_x, old_u.x();
        old_augmented_state_y << old_state_y, old_u.y();

        Eigen::VectorXd old_bar_x(theta.size());
        old_bar_x = fromx2xbar(old_augmented_state_x);
        old_bar_y = fromx2xbar(old_augmented_state_y);

        state_x << cur_pos.x(), cur_vel.x(), ref_msg;
        state_y << cur_pos.y(), cur_vel.y(), ref_msg;
        

        // excitation = Excitation(t);

        u.x() = - Kx * state_x;
        u.y() = - Ky * state_y;
        u.z() = 0.0;

        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0f, 73};

        vel_pub.publish(vel_msg);


        augmented_state_x << state_x, u.x();
        augmented_state_y << state_x, u.y();

        Eigen::VectorXd bar_x(theta.size());
        bar_x = fromx2xbar(augmented_state_x);
        bar_y = fromx2xbar(augmented_state_y);
        
        phi.resize(3);

        phi[0] = old_bar_x - gamma * bar_x;
        phi[1] = old_bar_y - gamma * bar_y;

        //REWARD

        Cx << Cd, Cmx;
        Cy << Cd, Cmy;

        Qx = Cx.transpose() * Qe * Cx;
        Qy = Cy.transpose() * Qe * Cy;


        reward.x() = - state_x.transpose() * Qx * state_x - u.x() * R * u.x();
        reward.y() = - state_x.transpose() * Qy * state_x - u.y() * R * u.y();

        ROS_INFO_STREAM("reward" << reward);

        geometry_msgs::Vector3 reward_msg;
        reward_msg.x = reward.x();
        reward_msg.y = reward.y();
        reward_pub.publish(reward_msg);


        //error
        Erls.x() = reward.x() - phi[0].transpose() * theta;
        Erls.y() = reward.y() - phi[1].transpose() * theta;

        // ROS_INFO_STREAM("erls" << Erls); 


       
        // RLS
        UpdateRLS(theta, phi, Erls, prls, mu);

        if (countk > 50)
        {
            UpdateGain(theta);
        }

        // Update old pos
        old_pos = cur_pos;

        //Update old vel
        old_vel = cur_vel;

        countk++;

    }

}

int main(int argc, char **argv){

    ros::init(argc, argv, "RL_LQT_control_node");
    ROS_INFO("This node has started.");
    RLLQTController nh;

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
