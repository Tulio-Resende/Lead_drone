#include "rl_control/RL_LQT_control_node.hpp"
#include "rl_control/param_loader.hpp"

/* Constructor */
RLLQTController::RLLQTController(ros::NodeHandle& nh): 
priv_handle("~"), dist(0.0, 0.2)
{
    ROS_DEBUG("Constructor called.");
       

    nh.getParam("Qe", Qe);
    nh.getParam("R", R);
    nh.getParam("kpx", kpx);
    nh.getParam("kpy", kpy);
    nh.getParam("ki", ki);
    nh.getParam("K0factor", K0factor);
    mux = kpx/20;
    muy = kpy/20;

    loadMatrix(nh, "Am", Am);
    loadMatrix(nh, "Cmx", Cmx);
    loadMatrix(nh, "Cmy", Cmy);
    loadMatrix(nh, "Bm", Bm);
    loadMatrix(nh, "Cd", Cd);
    loadMatrix(nh, "Kx", Kx);
    loadMatrix(nh, "Ky", Ky);


    //initial gain
    Kx = Kx * K0factor; 
    Ky = Ky * K0factor;


    // theta = theta * THETA0factor;
    // prls = Eigen::MatrixXd::Identity(alpha.size(),alpha.size()) * PRLS0factor;
    // theta << 52.6334101099504, 10.2162277797894, 4188.82853290319, 1.03540441873538, 2.43711211624183, 0.000346393806324802, 1.54300006451501e-05, 1.00683497461433, 41.2049038660059, -938.638001034116, 9.95443496971824, -22.6452554201491, 0.181374103651703, -0.0569959139571596, 1.06043943182456, -365.189566987842, 5.58149507522191, -8.82355433095529, 0.101742357141362, -0.0223100797616370, 0.528471573951173, -86.2735882200421, 202.070159782202, -1.57122736630554, 0.508218360088234, -9.39701005855973, -2.09592364821546, 0.0378744331055520, -0.00538975260999602, 0.144703357204055, -0.0381754164895553, 0.0122611001906250, -0.227054969366253, -9.82036199459057e-05, 0.00263774819632151, -0.000574166812361116;
    // THETA0factor = 0.8;
    // PRLS0factor=10e2;



    std::vector<PlantAxis> axes = {
    {"x", Cmx},
    {"y", Cmy},
    // {"z", Cmz},
    // {"yaw", Cmyaw}
    };

    // Calcula todos Q_i e Q_dlyap_i
    Calc_Q_lyap(axes, Cd, Qe, R);

    // Armazena localmente se quiser usar depois
    Qx = axes[0].Q;
    Qy = axes[1].Q;
    // Qz = axes[2].Q;
    // Qyaw = axes[3].Q;

    Q_dlyap_x = axes[0].Q_dlyap;
    Q_dlyap_y = axes[1].Q_dlyap;
    // Q_dlyap_z = axes[2].Q_dlyap;
    // Q_dlyap_yaw = axes[3].Q_dlyap;


    Ba = Eigen::MatrixXd::Zero(7,1);
    Aa = Eigen::MatrixXd::Zero(7,7);
    A_dlyap_x = Eigen::MatrixXd::Zero(8,8);
    A_dlyap_y = Eigen::MatrixXd::Zero(8,8);

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

    bar_x = Eigen::VectorXd::Zero(n_param);
    old_bar_x = Eigen::VectorXd::Zero(n_param);
    bar_y = Eigen::VectorXd::Zero(n_param);
    old_bar_y = Eigen::VectorXd::Zero(n_param);

    
    theta_x = Eigen::VectorXd::Zero(n_param);
    theta_y = Eigen::VectorXd::Zero(n_param);


    ROS_INFO_STREAM("Initial Kx" << Kx);
    ROS_INFO_STREAM("Initial Ky" << Ky);
    ROS_INFO_STREAM("Initial mux" << mux);
    ROS_INFO_STREAM("Initial kpx" << kpx);
     ROS_INFO_STREAM("Initial muy" << muy);
    ROS_INFO_STREAM("Initial kpy" << kpy);

     countk = 0;
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
    gain_pub = handle.advertise<std_msgs::Float64MultiArray>("/rl_control/gain", 1);


}
void RLLQTController::receivePos(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if (msg){
        cur_pos.x() = msg->point.x; 
        cur_pos.y() = msg->point.y; 
        cur_pos.z() = msg->point.z;

        flag_pos = true;
        // ROS_INFO_STREAM("vel "<< cur_vel);

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

        // ROS_INFO_STREAM("vel "<< cur_vel);

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

// Eigen::VectorXd RLLQTController::fromx2xbar(const Eigen::VectorXd& x)
// {
//     int n = x.size();

//     int m = n + (n* (n-1))/2;
//     Eigen::VectorXd xbar(m);
    
//     int k = 0;

//     for (int i = 0; i < n; i++) 
//     {
//         xbar(k++) = x(i) * x(i);
//     }

//     for (int i = 0; i < n - 1; i++) {
//         for (int j = i + 1; j < n; j++) 
//         {
//             xbar(k++) = x(i) * x(j);
//         }
//     }
//     return -xbar;
// }

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
            xbar(k++) = 2 * x(i) * x(j);
        }
    }
    return -xbar;
}

// Eigen::MatrixXd RLLQTController::FromTHETAtoP(const Eigen::VectorXd& theta, int sizeOfAugState)
// {

//     int N = (sizeOfAugState * (sizeOfAugState + 1))/2;

//     Eigen::MatrixXd indx(N, 2);

//     int count = 0;

//     for (int i = 0; i < sizeOfAugState; i++)
//     {
//         indx(count, 0) = i;
//         indx(count, 1) = i; 
//         count++;
//     }
  
//     for(int i = 0; i < sizeOfAugState; i++)
//     {
//         for(int j = i+1; j < sizeOfAugState; j++)
//         {   
//             indx(count, 0) = i;
//             indx(count, 1) = j; 
//             count++;
//         }
//     }

//     Eigen::MatrixXd Pout1, Pout2, Pout;

//     Pout1 = Eigen::MatrixXd::Zero(sizeOfAugState, sizeOfAugState);
//     Pout2 = Eigen::MatrixXd::Zero(sizeOfAugState, sizeOfAugState);
//     Pout  = Eigen::MatrixXd::Zero(sizeOfAugState, sizeOfAugState);
//     int k = 0;

//     for (int i = 0; i < sizeOfAugState; i++)
//     {
//         Pout1(indx(i, 0), indx(i, 1)) = theta(k);
//         k++;
//     }

//     for (int i = sizeOfAugState; i < N; i++)
//     {
//         Pout2(indx(i, 0), indx(i, 1)) = theta(k)/2;
//         k++;
//     }

//     Pout = Pout1 + Pout2 + Pout2.transpose();

//     return Pout; 
// }

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
        Pout2(indx(i, 0), indx(i, 1)) = theta(k);
        k++;
    }

    Pout = Pout1 + Pout2 + Pout2.transpose();

    return Pout; 
}

// void RLLQTController::UpdateRLSALPHA(Eigen::VectorXd& alpha, std::vector<Eigen::VectorXd>& phi, Eigen::Vector3d& Erls, Eigen::MatrixXd& prls, double& mu)
// {
//     double phiT_P_phi = phi[0].transpose()* N * prls * N.transpose()* phi[0];
//     if (phiT_P_phi > 1e-12) 
//     {
//         alpha = alpha + (prls * N.transpose() * phi[0] * Erls.x()) / (mu + phiT_P_phi);
//         prls = (1.0/mu) * prls - (1.0/mu) * (prls * N.transpose() * phi[0] * phi[0].transpose() * N * prls) / (mu + phiT_P_phi);
//     }
// }

// void RLLQTController::UpdateRLS(Eigen::VectorXd& theta, std::vector<Eigen::VectorXd>& phi, Eigen::Vector3d& Erls, Eigen::MatrixXd& prls, double& mu)
// {
//     double phiT_P_phi = phi[0].transpose() * prls * phi[0];
//     if (phiT_P_phi > 1e-6) // evita divisões por zero ou muito pequenas
//     {
//         theta = theta + (prls * phi[0] * Erls.x()) / (mu + phiT_P_phi);
//         prls = (1.0/mu) * prls - (1.0/mu) * (prls * phi[0] * phi[0].transpose() * prls) / (mu + phiT_P_phi);
//     }
//     // ROS_INFO_STREAM("theta" << theta);
// }

// void RLLQTController::UpdateGain(Eigen::VectorXd& theta, const Eigen::MatrixXd& A_dlyap, const Eigen::MatrixXd& Q_dlyap)
// {
//     // Recovering H from theta
//     int z = augmented_state_x.size();
 
//     H.resize(3);
//     H[0] = FromTHETAtoP(theta, z);
//     ROS_INFO_STREAM("H_hat: \n" << H[0]);

//     // Eigen::MatrixXd H0 = H[0];

//     // // --- Calcula autovalores ---
//     // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(H0);
//     // Eigen::VectorXd eigvals = eig.eigenvalues();

//     // std::cout << "Autovalores originais:\n" << eigvals.transpose() << "\n";

//     // // --- Pega o menor autovalor ---
//     // double lambda_min = eigvals.minCoeff();
//     // std::cout << "Menor autovalor = " << lambda_min << "\n";

//     // // --- Se for <= 0, faz o shift ---
//     // double eps = 1e-8;
//     // double factor = (lambda_min < eps) ? (-lambda_min + eps) : 0.0;
//     // H0 = H0 + factor * Eigen::MatrixXd::Identity(H0.rows(), H0.cols());
//     // H[0] = H0;
//     // std::cout << "\nShift aplicado (factor) = " << factor << "\n";
//     // std::cout << "Matriz H_pd = \n" << H[0] << "\n";

//     // // --- Verifica autovalores após o shift ---
//     // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig_pd(H0);
//     // std::cout << "\nAutovalores após shift:\n" << eig_pd.eigenvalues().transpose() << "\n";

//     // Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(H0);
//     // int rank = lu_decomp.rank();
    
//     if (true)
//     {
//         // Update Gain
//         //TODO: is there a better way to do that?
        
//         switch(dlyap_flag)
//         {
//             case 0:
//                 inv_scalar = 1.0 / H[0](z-1, z-1);
//                 switch(gain_update)
//                 {
//                 case 1:
//                     ROS_INFO_STREAM("Gain updated directly");
//                     Kx = inv_scalar*H[0].row(z-1).segment(0,z-1);
//                     ROS_INFO_STREAM("Updated Kx: " << Kx);
//                     // H[0] = dlyap_iterative(A_dlyap, Q_dlyap);
//                     // ROS_INFO_STREAM("H: \n" << H[0]);

                                
//                     gain_msg.data.resize(state_x.size());
//                     for (int i = 0; i < state_x.size(); i++) 
//                     {
//                         gain_msg.data[i] = Kx[i];  
//                     }
//                     gain_pub.publish(gain_msg); 
//                     break;
//                 default:
//                     ROS_INFO_STREAM("Gain is not updated directly");
//                     Eigen::RowVectorXd K_diff;
//                     K_diff = inv_scalar*H[0].row(z-1).segment(0,z-1);
//                     ROS_INFO_STREAM("Updated Kxx: " << K_diff);
//                     break;
//                 }
//                 break;
//             default:
//                 ROS_INFO_STREAM("DLYAP");
//                 H[0] = dlyap_iterative(A_dlyap, Q_dlyap);
//                 ROS_INFO_STREAM("H: \n" << H[0]);
//                 inv_scalar = 1.0 / H[0](z-1, z-1);
//                 Kx = inv_scalar*H[0].row(z-1).segment(0,z-1);
//                 ROS_INFO_STREAM("Updated Kx: " << Kx);
//                 break;
//         }

//         countk = 0;
//     }
//     else
//     {
//         // ROS_INFO_STREAM("Poor rank" << rank);            
//     }
// }

Eigen::MatrixXd RLLQTController::kronecker(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
{
    int rowsA = A.rows(), colsA = A.cols();
    int rowsB = B.rows(), colsB = B.cols();

    // Kronecker product: A^T ⊗ A
    
    Eigen::MatrixXd kron(rowsA * rowsB, colsA * colsB);

    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsA; j++) {
            kron.block(i * rowsB, j * colsB, rowsB, colsB) = A(i, j) * B;
        }
    }
    return kron;
}

Eigen::MatrixXd RLLQTController::dlyap_iterative(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q, 
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

Eigen::MatrixXd RLLQTController::dlyap(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q) {

    int rowsA = A.rows(), colsA = A.cols();
    Eigen::MatrixXd kron = Eigen::MatrixXd::Zero(rowsA*rowsA, colsA*colsA);

    // Verificar se Q é quadrada e tem mesma dimensão que A
    assert(Q.rows() == rowsA && Q.cols() == rowsA);
    
    
    // Matriz identidade de dimensão n² × n²
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(rowsA*rowsA, rowsA*rowsA);

    kron = kronecker(A, A);
    Eigen::MatrixXd M = I - kron;
    
    // Vectorizar Q
    Eigen::VectorXd vecQ = Eigen::Map<const Eigen::VectorXd>(Q.data(), Q.size());
    
    // Resolver o sistema linear (mais estável que calcular inversa)
    Eigen::VectorXd vecP = M.colPivHouseholderQr().solve(vecQ);
    
    // Recuperar a matriz P
    return Eigen::Map<const Eigen::MatrixXd>(vecP.data(), rowsA, rowsA);
}

Eigen::Vector3d RLLQTController::Excitation(double& t)
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

double RLLQTController::Calc_reward(const Eigen::VectorXd& old_state, const float& old_u, const Eigen::MatrixXd& Q, const double& R)
{

    // Calcula custo quadrático de estado
    double state_term = (old_state.transpose() * Q * old_state)(0, 0);

    // Custo quadrático de controle (usa norma ao quadrado)
    double control_term = old_u* R * old_u;

    // Retorna reward (negativo do custo)
    return -(state_term + control_term);
}

void RLLQTController::Calc_reward_all()
{
    reward.x() = Calc_reward(old_state_x, old_u.x(), Qx, R);
    reward.y() = Calc_reward(old_state_y, old_u.y(), Qy, R);
    // reward.z() = Calc_reward(old_state_z, old_u.z(), Qz, R);
    // reward.z() = Calc_reward(old_state_yaw, old_u.z(), Qyaw, R); // se tiver yaw separado

    // Publica no tópico
    geometry_msgs::Vector3 reward_msg;
    reward_msg.x = reward.x();
    reward_msg.y = reward.y();
    // reward_msg.z = reward.z();
    reward_pub.publish(reward_msg);
}

Eigen::VectorXd RLLQTController::UpdateTheta(const Eigen::MatrixXd& H_THETA)
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

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> RLLQTController::UpdateMatrices(const double& kp)
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

inline void RLLQTController::Calc_Q_lyap(
    std::vector<PlantAxis>& axes,
    const MatrixXd& Cd,
    const double& Qe,
    const double R)
{
    for (auto& axis : axes)
    {
        // Monta a matriz aumentada C_a = [Cd  -Cm_i]
        MatrixXd C_a(Cd.rows(), Cd.cols() + axis.Cm.cols());
        C_a << Cd, -axis.Cm;

        // Calcula Q_i = C_aᵀ * Qe * C_a
        axis.Q = C_a.transpose() * Qe * C_a;

        // Monta Q_dlyap_i
        axis.Q_dlyap.setZero(axis.Q.rows() + 1, axis.Q.cols() + 1);
        axis.Q_dlyap.block(0, 0, axis.Q.rows(), axis.Q.cols()) = axis.Q;
        axis.Q_dlyap(axis.Q.rows(), axis.Q.cols()) = R;

        ROS_INFO_STREAM("✔ Q_" << axis.name << " calculado:\n" << axis.Q);
    }
}

AxisSystem RLLQTController::buildAxisSystem(double kp, const Eigen::RowVectorXd& K)
{
    AxisSystem sys;

    // === Atualiza Ap e Bp ===
    auto matrices = UpdateMatrices(kp);
    sys.Ap = matrices.first;
    sys.Bp = matrices.second;

    // === Monta Aa e Ba ===
    sys.Aa.resize(7, 7);
    sys.Ba.resize(7, 1);

    sys.Aa << sys.Ap, Eigen::Matrix<double, 2, 5>::Zero(),
               Eigen::Matrix<double, 5, 2>::Zero(), Am;  // usa o Am membro da classe

    sys.Ba << sys.Bp, Bm;  // idem, Bm membro da classe

    // === Monta A_dlyap ===
    sys.A_dlyap.resize(8, 8);
    sys.A_dlyap.setZero();

    sys.A_dlyap.topLeftCorner(7, 7)     = sys.Aa;
    sys.A_dlyap.topRightCorner(7, 1)    = sys.Ba;
    sys.A_dlyap.bottomLeftCorner(1, 7)  = -K * sys.Aa;
    sys.A_dlyap.bottomRightCorner(1, 1) = -K * sys.Ba;

    return sys;
}

void RLLQTController::sendCmdVel(double h){

    if (flag_pos && flag_vel && flag_ref)
    {
        double gamma = 0.5;
        static double t = 0.0;
        t += h;

        ROS_INFO_STREAM("mux" << mux);
        ROS_INFO_STREAM("kpx" << kpx);
        ROS_INFO_STREAM("muy" << muy);
        ROS_INFO_STREAM("kpy" << kpy);

    
        // x_{k}
        state_x << cur_pos.x(), cur_vel.x(), ref_msg;
        state_y << cur_pos.y(), cur_vel.y(), ref_msg;

        // excitation = Excitation(t); 
        // u_{k}
        u.x() = - Kx * state_x;
        u.y() = - Ky * state_y;
        u.z() = 0.0;

        // L_{k}
        augmented_state_x << state_x, u.x();
        augmented_state_y << state_y, u.y();

        if(rl)
        {
            // Bar_L_{k-1}
            old_bar_x = fromx2xbar(old_augmented_state_x);
            old_bar_y = fromx2xbar(old_augmented_state_y);

            // Bar_L_{k}
            bar_x = fromx2xbar(augmented_state_x);
            bar_y = fromx2xbar(augmented_state_y);

            // Phi
            phi.resize(3);
            phi[0] = old_bar_x - std::pow(gamma, h) * bar_x;
            phi[1] = old_bar_y - std::pow(gamma, h) * bar_y;

            // Reward
            Calc_reward_all();

            auto sys_x   = buildAxisSystem(kpx, Kx);
            auto sys_y   = buildAxisSystem(kpy, Ky);
            // auto sys_z   = buildAxisSystem(kp_z, Kz);
            // auto sys_yaw = buildAxisSystem(kp_yaw, Kyaw);

            A_dlyap_x   = sys_x.A_dlyap;
            A_dlyap_y   = sys_y.A_dlyap;
            // A_dlyap_z   = sys_z.A_dlyap;
            // A_dlyap_yaw = sys_yaw.A_dlyap;

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

            mux = mux + h * (Erls.x());
            muy = muy + h * (Erls.y());

            kpx = mux * ki;
            kpy = muy * ki;

            // RLS
            // UpdateRLS(theta, phi, Erls, prls, mu);


            if (countk > 400)
            {
                
                // UpdateGain(theta, A_dlyap.transpose()*sqrt(pow(gamma, h)), Q_dlyap);
                int z = augmented_state_x.size();
                // ROS_INFO_STREAM("H_hat: \n" << H[0]);

                inv_scalar_x = 1.0 / H[0](z-1, z-1);
                inv_scalar_y = 1.0 / H[1](z-1, z-1);

                Kx = inv_scalar_x * H[0].row(z-1).segment(0,z-1);
                Ky = inv_scalar_y * H[1].row(z-1).segment(0,z-1);
                ROS_INFO_STREAM("Updated Ky: " << Ky);

                gain_msg_y.data.resize(state_x.size());
                gain_msg_x.data.resize(state_x.size());

                for (int i = 0; i < state_x.size(); i++) 
                {
                    gain_msg_x.data[i] = Kx[i]; 
                    gain_msg_y.data[i] = Kx[i];   
                }
                gain_pub.publish(gain_msg_x); 

                countk = 0;
            
            }

        }
              
        // Control Signal Pub
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0f, 73};
        vel_pub.publish(vel_msg);

        // Update param
        old_augmented_state_x = augmented_state_x;
        old_augmented_state_y = augmented_state_y;
        old_state_x = state_x;
        old_state_y = state_y;

        old_u = u;
        rl = true;

        countk++;     
    }
    
}

int main(int argc, char **argv){

    ros::init(argc, argv, "RL_LQT_control_node");
    ROS_INFO("This node has started.");

    ros::NodeHandle nh;
    
    RLLQTController rl_lqt(nh);

    rl_lqt.configNode();

    ros::Time start_time = ros::Time::now();

    ros::Rate sampling_rate(50);    // Hertz
    while(ros::ok()){

        // ros::Time current_time = ros::Time::now();

        // double h = (current_time - start_time).toSec();
        double h = 1.0/50.0;
        ros::spinOnce();
        rl_lqt.sendCmdVel(h);

        sampling_rate.sleep();
    }
}
