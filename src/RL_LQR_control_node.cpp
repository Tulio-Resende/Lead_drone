#include "rl_control/RL_LQR_control_node.hpp"

/* Constructor */
RLLQRController::RLLQRController(/* args */): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");

    state_x = Eigen::VectorXd::Zero(2);
    state_y = Eigen::VectorXd::Zero(2);
    old_state_x = Eigen::VectorXd::Zero(2);
    old_state_y = Eigen::VectorXd::Zero(2);

    augmented_state_x = Eigen::VectorXd::Zero(3);
    augmented_state_y = Eigen::VectorXd::Zero(3);
    old_augmented_state_x = Eigen::VectorXd::Zero(3);
    old_augmented_state_y = Eigen::VectorXd::Zero(3);


    theta = Eigen::VectorXd::Zero(6);

    Kx = Eigen::RowVectorXd::Zero(2);
    Ky = Eigen::RowVectorXd::Zero(2);
    kz = Eigen::RowVectorXd::Zero(2);

    K0factor = 1.0/10.0;
    THETA0factor = 0.8;
    PRLS0factor=1e6;
    Kx << 0.4880, 0.5139;
    Ky <<  0.4880, 0.5139;
    kz << 0,0;

    //initial gain
    Kx = Kx * K0factor ; 
    Ky = Ky * K0factor;

    countk = 0;

    // //pensar numa forma melhor de pegar o tamanho da identidade (tem que ser do tamanho da matriz Aa)
    prls = Eigen::MatrixXd::Identity(6,6) * PRLS0factor;

}

/* Destructor */
RLLQRController::~RLLQRController()
{
}

void RLLQRController::configNode(){
    configSubscribers();
    configPublishers();
    // configService();
    // ROS_INFO("Node configured.");
}

void RLLQRController::configSubscribers(){
    curPosSub = handle.subscribe("/dji_sdk/local_position", 1, &RLLQRController::receivePos, this);
    curVelSub = handle.subscribe("/dji_sdk/velocity", 1, &RLLQRController::receiveVel, this);
}

void RLLQRController::configPublishers(){
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);
    reward_pub = handle.advertise<std_msgs::Float64>("/rl_control/reward", 1);

}

// void RLLQRController::configService(){
//     enable_service = handle.advertiseService("/rl_control/enable_control", &RLLQRController::enableControl, this);
// }


void RLLQRController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg){
    if (msg){
        cur_pos.x() = msg->point.x; 
        cur_pos.y() = msg->point.y; 
        cur_pos.z() = msg->point.z;

        flag_pos = true;
        // ROS_INFO_STREAM("Current position: " << "(" << cur_pos[0] << ", " << cur_pos[1] << ", " << cur_pos[2] <<")");
    }
}

void RLLQRController::receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    if (msg)
    {
        cur_vel.x() = msg->vector.x;
        cur_vel.y() = msg->vector.y;
        cur_vel.z() = msg->vector.z;

        flag_vel = true;
    }
}

// bool RLLQRController::enableControl(std_srvs::SetBool::Request &req,
//                                     std_srvs::SetBool::Response &res)
// {
//     control_enabled = req.data;
//     res.success = true;
//     res.message = control_enabled ? "Controle habilitado." : "Controle desabilitado.";
//     return true;
// }


Eigen::VectorXd RLLQRController::fromx2xbar(const Eigen::VectorXd& x)
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

    return xbar;

}

Eigen::MatrixXd RLLQRController::FromTHETAtoP(const Eigen::VectorXd& theta, int sizeOfAugState)
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


void RLLQRController::sendCmdVel(double h){

    if (flag_pos && flag_vel)
    {
        double Q=1, R=1, gamma = 0.5, mu = 1;

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

        //ROS_INFO_STREAM("old position:" << old_pos);

        old_state_x << old_pos.x(), old_vel.x();
        old_state_y << old_pos.y(), old_vel.y();

        old_u.x() = - Kx * old_state_x;
        old_u.y() = - Ky * old_state_y;

        old_augmented_state_x << old_pos.x(), old_vel.x(), old_u.x();
        old_augmented_state_y << old_pos.y(), old_vel.y(), old_u.y();

        // TODO: Find a better way
        int n = old_augmented_state_x.size();
        int m = n + (n* (n-1))/2;
        Eigen::VectorXd old_bar_x(m);
        old_bar_x = fromx2xbar(old_augmented_state_x);


        state_x << cur_pos.x(), cur_vel.x();
        state_y << cur_pos.y(), cur_vel.y();

        Eigen::Vector3d noise_vec;
        noise_vec(1) = dist(generator);

        u.x() = - Kx * state_x + noise_vec(1);
        // u.y() = - Ky * state_y + noise_vec(1);
        u.y() = 0.0;
        u.z() = 0.0;

        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0f, 73};

        vel_pub.publish(vel_msg);

        augmented_state_x << state_x, u.x();
        augmented_state_y << state_x, u.y();

        Eigen::VectorXd bar_x(m);
        bar_x = fromx2xbar(augmented_state_x);
        
        phi.resize(3);

        phi[0] = old_bar_x - gamma * bar_x;

        reward.x() = - state_x.transpose() * Q * state_x - u.x() * R * u.x();

        std_msgs::Float64 msg;
        msg.data = reward.x();
        reward_pub.publish(msg);

        Erls.x() = reward.x() - phi[0].transpose() * theta; 
     
        ROS_INFO_STREAM("Theta" << theta);
        // RLS
        theta = theta + prls * phi[0] * Erls.x()/(mu + phi[0].transpose() * prls* phi[0]);
        prls = (1/mu)*prls - (1/mu)*(prls*phi[0]*phi[0].transpose()*prls)/(mu + phi[0].transpose()*prls*phi[0]);


        if (countk > 100)
        {
            // Recovering H from theta
            int z = augmented_state_x.size();

            H.resize(3);
            H[0] = FromTHETAtoP(theta, z);
           
            // Update Gain
            //TODO
            double inv_scalar = 1.0 / H[0](z-1, z-1); // is there a better way to do that?
            Eigen::RowVectorXd Kxx;

            // Kxx = inv_scalar*H[0].row(z-1).segment(0,z-1);
            Kx = inv_scalar*H[0].row(z-1).segment(0,z-1);

            
            // ROS_INFO_STREAM("Updated Kxx: " << Kxx);
            ROS_INFO_STREAM("Updated Kxx: " << Kx);

            countk = 1;
        }

        // Update old pos
        old_pos.x() = cur_pos.x();
        old_pos.y() = cur_pos.y();
        old_pos.z() = cur_pos.z();

        //Update old vel
        old_vel.x() = cur_vel.x();
        old_vel.y() = cur_vel.y();
        old_vel.z() = cur_vel.z();
        countk++;

    }
    


}

int main(int argc, char **argv){

    ros::init(argc, argv, "RL_LQT_control_node");
    ROS_INFO("This node has started.");
    RLLQRController nh;

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
