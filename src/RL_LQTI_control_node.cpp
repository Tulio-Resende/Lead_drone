#include "rl_control/RL_LQTI_control_node.hpp"
#include "rl_control/param_loader.hpp"


/* Constructor */
RLLQTIController::RLLQTIController(ros::NodeHandle& nh): 
priv_handle("~")
{
    ROS_DEBUG("Constructor called.");

    nh.getParam("yss", yss);
    nh.getParam("Qe_LQTI", Qe);
    nh.getParam("R", R);
    nh.getParam("kpx", kpx);
    nh.getParam("kpy", kpy);
    nh.getParam("ki", ki);
    nh.getParam("gamma", gamma);
    nh.getParam("K0factor", K0factor);
    nh.getParam("h", h);

    
    loadMatrix(nh, "Kx_LQTI", Kx);
    loadMatrix(nh, "Ky_LQTI", Ky);
    loadMatrix(nh, "Am_LQTI", Am);
    loadMatrix(nh, "Cmx_LQTI", Cmx);
    loadMatrix(nh, "Cmy_LQTI", Cmy);
    loadMatrix(nh, "Bm_LQTI", Bm);
    loadMatrix(nh, "Cd", Cd);


    ROS_INFO_STREAM("Qe_LQTI" << Qe);
    ROS_INFO_STREAM("Am_LQTI" << Am);
    ROS_INFO_STREAM("Cmx_LQTI" << Cmx);
    ROS_INFO_STREAM("Cmy_LQTI" << Cmy);



    //initial gain
    Kx = Kx * K0factor; 
    Ky = Ky * K0factor;

    std::vector<PlantAxis> axes = {
    {"x", Cmx},
    {"y", Cmy},
    // {"z", Cmz},
    // {"yaw", Cmyaw}
    };

    // Q_i e Q_dlyap_i
    Calc_Q_lyap(axes, Cd, Qe, R, h);

    Qx = axes[0].Q;
    Qy = axes[1].Q;
    // // Qz = axes[2].Q;
    // // Qyaw = axes[3].Q;

    Q_dlyap_x = axes[0].Q_dlyap;
    Q_dlyap_y = axes[1].Q_dlyap;
    // Q_dlyap_z = axes[2].Q_dlyap;
    // Q_dlyap_yaw = axes[3].Q_dlyap;

    ref_msg = Eigen::VectorXd::Zero(4);

    int n_state = 2;
    int n_state_aug = n_state + ref_msg.size() + 1;
    int n_state_aug_control = n_state_aug + 1;
    int n_param = (n_state_aug_control)*(n_state_aug_control + 1)/2;


    old_u = Eigen::Vector3d::Zero(3);
    tilde_u = Eigen::Vector3d::Zero(3);

    old_xDOF = Eigen::VectorXd::Zero(n_state);
    old_yDOF = Eigen::VectorXd::Zero(n_state);

    old_y = Eigen::VectorXd::Zero(3);

    z_tilde_xDOF = Eigen::VectorXd::Zero(n_state_aug);
    z_tilde_yDOF = Eigen::VectorXd::Zero(n_state_aug);
    old_z_tilde_xDOF = Eigen::VectorXd::Zero(n_state_aug);
    old_z_tilde_yDOF = Eigen::VectorXd::Zero(n_state_aug);

    augmented_state_x = Eigen::VectorXd::Zero(n_state_aug_control);
    augmented_state_y = Eigen::VectorXd::Zero(n_state_aug_control);
    old_augmented_state_x = Eigen::VectorXd::Zero(n_state_aug_control);
    old_augmented_state_y = Eigen::VectorXd::Zero(n_state_aug_control);


    bar_x = Eigen::VectorXd::Zero(n_param);
    old_bar_x = Eigen::VectorXd::Zero(n_param);
    bar_y = Eigen::VectorXd::Zero(n_param);
    old_bar_y = Eigen::VectorXd::Zero(n_param);

    theta_x = Eigen::VectorXd::Zero(n_param);
    theta_y = Eigen::VectorXd::Zero(n_param);

    countk = 0;

}

/* Destructor */
RLLQTIController::~RLLQTIController()
{
}

void RLLQTIController::configNode(){
    configSubscribers();
    configPublishers();
    // ROS_INFO("Node configured.");
}

void RLLQTIController::configSubscribers()
{
    refGeneratorSub = handle.subscribe("/rl_control/ref_generator", 1, &RLLQTIController::receiveRef, this);
    curPosSub = handle.subscribe("/dji_sdk/local_position", 1, &RLLQTIController::receivePos, this);
    curVelSub = handle.subscribe("/dji_sdk/velocity", 1, &RLLQTIController::receiveVel, this);
}

void RLLQTIController::configPublishers(){
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);
    reward_pub = handle.advertise<geometry_msgs::Vector3>("/rl_control/reward", 1);
    gain_pub = handle.advertise<std_msgs::Float64MultiArray>("/rl_control/gain", 1);
    cost_pub = handle.advertise<geometry_msgs::Vector3>("/rl_control/cost", 1);
    kp_pub = handle.advertise<geometry_msgs::Vector3>("/rl_control/kp", 1);
}

void RLLQTIController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg)
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

void RLLQTIController::receiveVel(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    if (msg){
        cur_vel.x() = msg->vector.x;
        cur_vel.y() = msg->vector.y;
        cur_vel.z() = msg->vector.z;
    }
    flag_vel = true;
    // ROS_INFO_STREAM("cur vel" << cur_vel);

}

void RLLQTIController::receiveRef(const std_msgs::Float64MultiArray::ConstPtr& msg)
{   
    ref_msg.resize(msg->data.size());

    for (int i = 0; i < 4; i++) {
        ref_msg(i) = msg->data[i];
        // ROS_INFO("Node configured. ref[%d] = %f", i, ref_msg[i]);
    }
    flag_ref = true;
    // ROS_INFO_STREAM("ref" << ref_msg);

}

Eigen::VectorXd RLLQTIController::fromx2xbar(const Eigen::VectorXd& x)
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
            xbar(k++) = 2 * x(i) * x(j);
        }
    }
    return -xbar;
}

Eigen::MatrixXd RLLQTIController::dlyap_iterative(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q, 
                               int max_iter, double tol) {
    int n = A.rows();
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd P_old;
    Eigen::MatrixXd A_trans = A.transpose();
    
    for (int i = 0; i < max_iter; i++) {
        P_old = P;
        P = A * P * A_trans + Q;
        
        if ((P - P_old).norm() < tol) {
            std::cout << "Convergiu em " << i+1 << " iterações\n";
            break;
        }
    }

    return P;
}

double RLLQTIController::Calc_reward(const Eigen::VectorXd& old_state, const float& old_u, const Eigen::MatrixXd& Q, const double& R)
{

    // Calcula custo quadrático de estado
    double state_term = (old_state.transpose() * Q * old_state)(0, 0);

    // Custo quadrático de controle (usa norma ao quadrado)
    double control_term = old_u* R * old_u;

    // Retorna reward (negativo do custo)
    return -(state_term + control_term);
}

void RLLQTIController::Calc_reward_all(double& t)
{
    reward.x() = Calc_reward(old_z_tilde_xDOF, old_u.x(), Qx, R);
    reward.y() = Calc_reward(old_z_tilde_yDOF, old_u.y(), Qy, R);
    // reward.z() = Calc_reward(old_state_z, old_u.z(), Qz, R);
    // reward.yaw() = Calc_reward(old_state_yaw, old_u.z(), Qyaw, R); // se tiver yaw separado

    // Publica no tópico
    geometry_msgs::Vector3 reward_msg;
    reward_msg.x = reward.x();
    reward_msg.y = reward.y();
    // reward_msg.z = reward.z();
    reward_pub.publish(reward_msg);

    totalCost(reward, t);

}
void RLLQTIController::totalCost(const Eigen::Vector3d& reward, double& t)
{
    geometry_msgs::Vector3 cost_msg;
    cost = std::pow(gamma, t) * reward;

    cost_msg.x = cost.x();
    cost_msg.y = cost.y();
    // cost_msg.z = cost.z();
    cost_pub.publish(cost_msg);

}

Eigen::VectorXd RLLQTIController::UpdateTheta(const Eigen::MatrixXd& H_THETA)
{
    int count_H = 0;
    
    Eigen::VectorXd theta(36);
    Eigen::VectorXd idx = Eigen::VectorXd::Zero(36);
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(36,36);
    
    Eigen::MatrixXd P_permuted(idx.size(), P.cols());

    idx << 1, 9, 16, 22 ,27, 31, 34, 36, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 23, 24, 25, 26, 28, 29, 30, 32, 33, 35;
    idx = idx.array() - 1;
    
    for(int i = 0; i < idx.size(); i++) {
        P_permuted.row(i) = P.row(idx(i));
    }
    P = P_permuted;
    
    // Extrair elementos triangulares superiores
    for(int i = 0; i < 8; i++) {
        for(int j = i; j < 8; j++) {
            if(count_H < 36) {
                theta(count_H++) = H_THETA(i, j);
            }
        }
    }
    theta = P * theta;

    return theta;
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> RLLQTIController::UpdateMatrices(const double& kp)
{
    Eigen::Matrix<double, 2, 2> Ap;
    Eigen::Matrix<double, 2, 1> Bp;

    Ap <<
    1.0000,    -(std::exp(-kp/50.0)- 1)/kp,
    0,         std::exp(-kp/50.0);


    Bp <<
    (kp/50 + exp(-kp/50) - 1)/kp, 
             1 - exp(-kp/50);   

    return std::make_pair(Ap, Bp);
}

inline void RLLQTIController::Calc_Q_lyap(std::vector<PlantAxis>& axes, const MatrixXd& Cd, const double& Qe, const double& R, const double& h)
{
    for (auto& axis : axes)
    {
        // Monta a matriz aumentada C_a = [Cd  -Cm_i]
        MatrixXd C_a(Cd.rows(), Cd.cols() + axis.Cm.cols() + 1);
        C_a << Cd, -1/h, -axis.Cm;

        // Calcula Q_i = C_aᵀ * Qe * C_a
        axis.Q = C_a.transpose() * Qe * C_a;

        // Monta Q_dlyap_i
        axis.Q_dlyap.setZero(axis.Q.rows() + 1, axis.Q.cols() + 1);
        axis.Q_dlyap.block(0, 0, axis.Q.rows(), axis.Q.cols()) = axis.Q;
        axis.Q_dlyap(axis.Q.rows(), axis.Q.cols()) = R;

        ROS_INFO_STREAM("✔ Q_" << axis.name << " calculado:\n" << axis.Q);
    }
}

AxisSystem RLLQTIController::buildAxisSystem(double& kp, const Eigen::RowVectorXd& K, double& h)
{
    AxisSystem sys;

    // === Atualiza Ap e Bp ===
    auto matrices = UpdateMatrices(kp);
    sys.Ap = matrices.first;
    sys.Bp = matrices.second;

    // === Monta Aa e Ba ===
    sys.Aa.resize(7, 7);
    sys.Ba.resize(7, 1);



    sys.Aa.setZero();

    // linha 0–1
    sys.Aa.block<2,2>(0,0) = sys.Ap;
    sys.Aa.block<2,5>(0,2) = Eigen::Matrix<double,2,5>::Zero();

    // linha 2
    sys.Aa.block<1,2>(2,0) = h * Cd;
    sys.Aa(2,2) = 1.0;
    sys.Aa.block<1,4>(2,3) = Eigen::Matrix<double,1,4>::Zero();

    // linha 3–6
    sys.Aa.block<4,3>(3,0) = Eigen::Matrix<double,4,3>::Zero();
    sys.Aa.block<4,4>(3,3) = Am;

    sys.Ba << sys.Bp, 0, Bm;  // idem, Bm membro da classe

    ROS_INFO_STREAM("Aqui");

    // === Monta A_dlyap ===
    sys.A_dlyap.resize(8, 8);
    sys.A_dlyap.setZero();

    sys.A_dlyap.topLeftCorner(7, 7)     = sys.Aa;
    sys.A_dlyap.topRightCorner(7, 1)    = sys.Ba;
    sys.A_dlyap.bottomLeftCorner(1, 7)  = -K * sys.Aa;
    sys.A_dlyap.bottomRightCorner(1, 1) = -K * sys.Ba;

    return sys;
}


void RLLQTIController::sendCmdVel(double h){

    if (flag_pos && flag_ref && flag_vel){     
        static double t = 0.0;

        if(flag_second_time)
        {

            old_xDOF << old_pos.x(), old_vel.x();
            old_yDOF << old_pos.y(), old_vel.y();

            ROS_INFO_STREAM("old_xDOF");

            old_y.x() = (Cd * old_xDOF)(0);
            old_y.y() = (Cd * old_yDOF)(0);


            tilde_mu.x() = h*(yss - old_y.x());
            tilde_mu.y() = h*(yss - old_y.y());
            ROS_INFO_STREAM("tilde_mu");


            tilde_pos = cur_pos - old_pos;
            tilde_vel = cur_vel - old_vel;

            ROS_INFO_STREAM("tilde_vel");


            z_tilde_xDOF << tilde_pos.x(), tilde_vel.x(), tilde_mu.x(), ref_msg;
            z_tilde_yDOF << tilde_pos.y(), tilde_vel.y(), tilde_mu.y(), ref_msg;

            ROS_INFO_STREAM("z_tilde_xDOF" << z_tilde_xDOF);


            tilde_u.x() = -Kx * z_tilde_xDOF;
            tilde_u.y() = -Ky * z_tilde_yDOF;
            tilde_u.z() = 0.0;

            // L_{k}
            augmented_state_x << z_tilde_xDOF, tilde_u.x();
            augmented_state_y << z_tilde_yDOF, tilde_u.y();

            ROS_INFO_STREAM("augmented_state_x" << augmented_state_x);



            if(rl)
            {
                // Bar_L_{k}
                old_bar_x = fromx2xbar(old_augmented_state_x);
                old_bar_y = fromx2xbar(old_augmented_state_y);
                ROS_INFO_STREAM("old_bar_y");

                // Bar_L_{k+1}
                bar_x = fromx2xbar(augmented_state_x);
                bar_y = fromx2xbar(augmented_state_y);

                // Phi
                phi.resize(3);
                phi[0] = old_bar_x - std::pow(gamma, h) * bar_x;
                phi[1] = old_bar_y - std::pow(gamma, h) * bar_y;
                
                ROS_INFO_STREAM("AQUI");

                // Reward
                // Calc_reward_all(h);

                ROS_INFO_STREAM("reward");


                auto sys_x = buildAxisSystem(kpx, Kx, h);
                auto sys_y = buildAxisSystem(kpy, Ky, h);
                // auto sys_z   = buildAxisSystem(kp_z, Kz);
                // auto sys_yaw = buildAxisSystem(kp_yaw, Kyaw);

                ROS_INFO_STREAM("sys_x");


                A_dlyap_x   = sys_x.A_dlyap;
                A_dlyap_y   = sys_y.A_dlyap;
                // A_dlyap_z   = sys_z.A_dlyap;
                // A_dlyap_yaw = sys_yaw.A_dlyap;

                ROS_INFO_STREAM("A_dlyap_x");


                H.resize(3);
                H[0] = dlyap_iterative(std::sqrt(std::pow(gamma, h))*A_dlyap_x.transpose(), Q_dlyap_x);
                H[1] = dlyap_iterative(std::sqrt(std::pow(gamma, h))*A_dlyap_y.transpose(), Q_dlyap_y);
                // ROS_INFO_STREAM("H" << H[0]);

                theta_x = UpdateTheta(H[0]);
                theta_y = UpdateTheta(H[1]);
                // ROS_INFO_STREAM("theta" << theta);


                // error
                Erls.x() = reward.x() - phi[0].transpose() * theta_x;
                Erls.y() = reward.y() - phi[1].transpose() * theta_y;
                ROS_INFO_STREAM("erls" << Erls.x());
                ROS_INFO_STREAM("erls_y" << Erls.y());


                geometry_msgs::Vector3 kp_msg;
                kp_msg.x = kpx;
                kp_msg.y = kpy;
                // cost_msg.z = cost.z();
                kp_pub.publish(kp_msg);

                kpx = kpx + ki * h * (Erls.x());
                kpy = kpy + ki * h * (Erls.y());

                if (countk > 400)
                {
                    
                    int z = augmented_state_x.size();
                    // ROS_INFO_STREAM("H_hat: \n" << H[0]);

                    inv_scalar_x = 1.0 / H[0](z-1, z-1);
                    inv_scalar_y = 1.0 / H[1](z-1, z-1);

                    Kx = inv_scalar_x * H[0].row(z-1).segment(0,z-1);
                    Ky = inv_scalar_y * H[1].row(z-1).segment(0,z-1);
                    ROS_INFO_STREAM("Updated Ky: " << Ky);

                    gain_msg_y.data.resize(z_tilde_xDOF.size());
                    gain_msg_x.data.resize(z_tilde_xDOF.size());

                    for (int i = 0; i < z_tilde_xDOF.size(); i++) 
                    {
                        gain_msg_x.data[i] = Kx[i]; 
                        gain_msg_y.data[i] = Kx[i];   
                    }
                    gain_pub.publish(gain_msg_x); 

                    countk = 0;
                
                }
                old_augmented_state_x = augmented_state_x;
                old_augmented_state_y = augmented_state_y;
                t += h;    
            }


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
        rl = true;
        countk = countk + 1;

    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "RL_LQTI_control_node");
    ROS_INFO("This node has started.");

    ros::NodeHandle nh;
    RLLQTIController lqti(nh);

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
