#include "rl_control/RL_LQT_control_node.hpp"

/* Constructor */
RLLQTController::RLLQTController(/* args */): 
priv_handle("~"), dist(0.0, 0.2)
{
    ROS_DEBUG("Constructor called.");
       
    //TODO: find a better way to do that (maybe param)
    // Reward calculation

    Ad << 
    1.0000,    0.0197,
    0,         0.9704;

    Cd << 1, 0;

    Bd <<
    0.0003, 0.0256;

    Am << 
    0.999992870110446,	    -0.000712988288670474,	-5.41441701278375e-08,	-5.41441057874235e-06,	0,
    0.0199999524673852,	    0.999992870110446,	    -3.60961305760057e-10,	-5.41441701278375e-08,	0,
    0.000199999762336881,	0.0199999524673852,	    0.999999999998195,	    -3.60961305760057e-10,	0,
    1.33333238268076e-06,	0.000199999762336881,	0.0199999999999928,	    0.999999999998195,	    0,
    6.66666349782453e-09,	1.33333238268076e-06,	0.000199999999999976,	0.0199999999999928,	    1;

    Cmx <<
    9.0000, 0, 0.2166, 0, 0.0005;

    Cmy <<
    2.0000, -0.2095, 0.0713, -0.0095, 0.0005;

    Bm.setZero();

    Ba = Eigen::MatrixXd::Zero(7,1);
    Aa = Eigen::MatrixXd::Zero(7,7);
    A_dlyap = Eigen::MatrixXd::Zero(8,8);
    Q_dlyap = Eigen::MatrixXd::Zero(8,8);


    Ba << Bd, 
          Bm;
    
    Aa << Ad, Eigen::Matrix<double,2,5>::Zero(),
          Eigen::Matrix<double,5,2>::Zero(), Am;

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

    Kx = Eigen::RowVectorXd::Zero(n_state_aug);
    Ky = Eigen::RowVectorXd::Zero(n_state_aug);
    // kz = Eigen::RowVectorXd::Zero(n_state_aug);

    K0factor = 1.0/20.0;
    THETA0factor = 0.8;
    PRLS0factor=10e-6;
    Kx << 0.526620279669378,	0.262442002550444,	-4.66660887607784,	0.0718605138143464,	-0.112756794852737,	0.00130992082259127,	-0.000285134518981841;
    // Kx << 0.5266, 0.2624, -4.6666, 0.0719, -0.1128, 0.0013, -0.0003;
    Ky << 0.5266, 0.2624, -0.9052, 0.1117, -0.0309, 0.0050, -0.0003;
    // kz << 0, 0;

    //initial gain
    Kx = Kx * K0factor; 
    // Ky = Ky * K0factor;

    ROS_INFO_STREAM("Initial Kx" << Kx);
    ROS_INFO_STREAM("Initial Ky" << Ky);

    theta = theta * THETA0factor;

    countk = 0;

    prls = Eigen::MatrixXd::Identity(theta.size(),theta.size()) * PRLS0factor;
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
    // ROS_INFO_STREAM("theta" << theta);

}

void RLLQTController::UpdateGain(Eigen::VectorXd& theta, const Eigen::MatrixXd& A_dlyap, const Eigen::MatrixXd& Q_dlyap)
{
    // Recovering H from theta
    int z = augmented_state_x.size();
 
    H.resize(3);
    H[0] = FromTHETAtoP(theta, z);
    ROS_INFO_STREAM("H_hat: \n" << H[0]);


    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(H[0]);
    int rank = lu_decomp.rank();
    
    if (dlyap_flag || rank == z)
    {
        // Update Gain
        //TODO: is there a better way to do that?
        
        switch(dlyap_flag)
        {
            case 0:
                inv_scalar = 1.0 / H[0](z-1, z-1);
                switch(gain_update)
                {
                case 1:
                    ROS_INFO_STREAM("TA ENTRANDO AQUI");
                    Kx = inv_scalar*H[0].row(z-1).segment(0,z-1);
                    ROS_INFO_STREAM("Updated Kx: " << Kx);
                    // H[0] = dlyap_iterative(A_dlyap, Q_dlyap);
                    // ROS_INFO_STREAM("H: \n" << H[0]);

                                
                    gain_msg.data.resize(state_x.size());
                    for (int i = 0; i < state_x.size(); i++) 
                    {
                        gain_msg.data[i] = Kx[i];  
                    }
        gain_pub.publish(gain_msg); 
                    break;
                default:
                    Eigen::RowVectorXd K_diff;
                    K_diff = inv_scalar*H[0].row(z-1).segment(0,z-1);
                    ROS_INFO_STREAM("Updated Kxx: " << K_diff);
                    break;
                }
                break;
            default:
                ROS_INFO_STREAM("DLYAP");
                H[0] = dlyap_iterative(A_dlyap, Q_dlyap);
                ROS_INFO_STREAM("H: \n" << H[0]);
                inv_scalar = 1.0 / H[0](z-1, z-1);
                Kx = inv_scalar*H[0].row(z-1).segment(0,z-1);
                ROS_INFO_STREAM("Updated Kx: " << Kx);
                break;
        }

        countk = 1;
    }
    else
    {
        ROS_INFO_STREAM("rank ruim" << rank);            
    }
}

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
        double gamma = 0.5, mu = 1.0, Qe = 1, R = 1;
        static double t = 0.0;
        t += h;

         // x_{k}
        state_x << cur_pos.x(), cur_vel.x(), ref_msg;
        state_y << cur_pos.y(), cur_vel.y(), ref_msg;


        // u_{k}
        u.x() = - Kx * state_x;
        u.y() = - Ky * state_y;
        u.z() = 0.0;

        // L_{k}
        augmented_state_x << state_x, u.x();
        // augmented_state_y << state_x, u.y();
        // ROS_INFO_STREAM("aug" << augmented_state_x);
        // ROS_INFO_STREAM("old aug" << old_augmented_state_x);

        if(rl)
        {
            // Bar_L_{k-1}
            Eigen::VectorXd old_bar_x(theta.size());
            old_bar_x = fromx2xbar(old_augmented_state_x);
            // old_bar_y = fromx2xbar(old_augmented_state_y);

            // Bar_L_{k}
            Eigen::VectorXd bar_x(theta.size());
            bar_x = fromx2xbar(augmented_state_x);
            // bar_y = fromx2xbar(augmented_state_y);

            phi.resize(3);

            // Phi
            phi[0] = old_bar_x - pow(gamma, h) * bar_x;
            // phi[1] = old_bar_y - gamma * bar_y;


            // Reward
            reward.x() = - old_state_x.transpose() * Qx * old_state_x - old_u.x() * R * old_u.x();
            // reward.y() = - state_x.transpose() * Qy * state_x - u.y() * R * u.y();
            // ROS_INFO_STREAM("reward x"  << reward.x());

            // Reward Pub
            geometry_msgs::Vector3 reward_msg;
            reward_msg.x = reward.x();
            reward_msg.y = reward.y();
            reward_pub.publish(reward_msg);

            // error
            Erls.x() = reward.x() - phi[0].transpose() * theta;
            // Erls.y() = reward.y() - phi[1].transpose() * theta;
            ROS_INFO_STREAM("erls" << Erls.x());

            // RLS
            UpdateRLS(theta, phi, Erls, prls, mu);

            if (countk > 100)
            {
                UpdateGain(theta, A_dlyap.transpose()*sqrt(pow(gamma, h)), Q_dlyap);
                double left_value = (-old_augmented_state_x.transpose()*H[0]*old_augmented_state_x);
                double right_quadratic = (augmented_state_x.transpose()*H[0]*augmented_state_x);
                double right_value = reward.x() - pow(gamma, h) * right_quadratic;

                // ROS_INFO("Left side: %f", left_value);
                // ROS_INFO("Right side: %f", right_value);
                // ROS_INFO("Difference: %f", left_value - right_value);
            
            }

        }
   
        // Control Signal Pub
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0f, 73};
        vel_pub.publish(vel_msg);


        
        // // Aug C
        // Cx << Cd, -Cmx;
        // Cy << Cd, -Cmy;

        // // Modifield Q LQR
        // Qx = Cx.transpose() * Qe * Cx;
        // Qy = Cy.transpose() * Qe * Cy;

        // // Test Dlyap                            
        // A_dlyap.topLeftCorner(7, 7) = Aa;
        // A_dlyap.topRightCorner(7, 1) = Ba;
        // A_dlyap.bottomLeftCorner(1, 7) = -Kx*Aa;
        // A_dlyap.bottomRightCorner(1, 1) = -Kx*Ba;
        // Q_dlyap.block<7,7>(0,0) = Qx;  
        // Q_dlyap(7,7) = R;

       
        // ROS_INFO_STREAM("Q_dlyap \n" << Q_dlyap);
        // ROS_INFO_STREAM("A_dlyap \n" << A_dlyap);

        // Update param
        old_augmented_state_x = augmented_state_x;
        // old_augmented_state_y = augmented_state_y;
        
        old_u = u;
        rl = true;

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
