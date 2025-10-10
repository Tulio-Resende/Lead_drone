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

    u_p = Eigen::VectorXd::Zero(36);
    N = Eigen::MatrixXd::Zero(36,15);
    ROS_INFO_STREAM("u_p" << u_p);
    ROS_INFO_STREAM("N" << N);

    u_p << -0.0001, -1.1845, 1.2695, 1.6097, -0.0768, 0.0000, 0.0000, 1.0308,
       -0.0238, -0.1990, -0.1103, -0.0065, -0.0022, 0.0003, -0.9151,
       -19.8014, -10.9792, -0.6509, -0.2236, 0.0337,
       -91.0551, -2.0276, -1.5823, 0.1939, 0.2745,
       -11.1943, -0.0336, 0.0152, -0.0092, -0.1084,
       0.0052, 0.0066, -0.2748, -0.0002, -0.0025, 0.0002;
    ROS_INFO_STREAM("u_p" << u_p);
    N << 
-0.0519, 0.5594, -0.2841, -0.3758, -0.1439, -0.2856, -0.3313, 0.4850, -0.0568, -0.0010, 0.0478, -0.0144, 0.0423, -0.0773, -0.0402,
 0.0184, -0.2026, 0.1403, -0.0190, -0.1929, 0.0673, -0.8902, -0.3086, 0.0582, -0.0019, -0.0629, 0.0379, 0.0035, -0.0031, 0.0086,
-0.0048, 0.0140, -0.0162, -0.0280, -0.0139, -0.0124, -0.0362, 0.0002, -0.3852, 0.3996, 0.1772, 0.1194, -0.5718, 0.3529, 0.4377,
-0.0001, 0.0027, -0.0027, -0.0057, -0.0030, -0.0025, 0.0056, -0.0234, -0.0633, 0.0334, 0.0078, 0.0451, 0.0918, -0.0266, 0.0402,
 0.0003, -0.0006, 0.0014, 0.0030, 0.0029, 0.0011, -0.0026, 0.0123, 0.0121, -0.0032, -0.0188, -0.0005, 0.0023, 0.0390, -0.0081,
 0.0000, 0.0000, -0.0000, -0.0000, 0.0000, 0.0000, 0.0000, -0.0001, -0.0009, 0.0001, -0.0007, 0.0005, 0.0007, -0.0001, 0.0002,
 0.0000, -0.0000, 0.0000, 0.0000, 0.0000, 0.0000, -0.0000, 0.0000, 0.0000, -0.0000, 0.0000, -0.0001, -0.0000, 0.0000, -0.0000,
 0.0000, -0.0001, 0.0001, -0.0000, -0.0001, 0.0000, -0.0006, -0.0002, 0.0001, -0.0000, -0.0001, 0.0000, -0.0000, -0.0000, 0.0000,
 0.7506, 0.3172, 0.5222, -0.1953, 0.1426, 0.0440, 0.0192, -0.0473, 0.0131, 0.0016, 0.0073, 0.0037, -0.0086, 0.0023, 0.0054,
 0.0424, 0.2296, 0.0065, 0.5641, 0.1678, -0.7321, -0.0687, -0.2380, 0.0239, -0.0059, -0.0164, 0.0054, -0.0172, 0.0233, -0.0012,
 0.0245, 0.5163, -0.2402, 0.5148, 0.1220, 0.6097, -0.1358, -0.0410, -0.0036, 0.0138, 0.0505, -0.0024, 0.0074, -0.0205, 0.0086,
 0.6258, -0.2440, -0.5888, 0.1218, -0.4225, -0.0509, 0.0653, -0.0056, 0.0065, -0.0056, -0.0315, 0.0030, -0.0031, 0.0245, -0.0098,
 0.1429, -0.1667, -0.4300, -0.2672, 0.8096, -0.0130, -0.1539, -0.1236, 0.0133, -0.0007, 0.0054, 0.0118, -0.0100, -0.0044, -0.0161,
-0.0778, 0.1939, -0.0905, -0.2164, -0.1304, 0.0026, 0.1028, -0.4811, -0.2927, -0.0138, 0.2114, 0.1042, -0.0604, 0.2503, -0.6592,
 0.0019, -0.0070, 0.0076, 0.0136, 0.0073, -0.0001, 0.0123, 0.0220, 0.1896, 0.8614, -0.2532, -0.1732, 0.0107, -0.1836, -0.3049,
-0.1109, 0.3090, -0.1641, -0.3065, -0.1473, -0.0070, 0.1692, -0.5226, 0.3546, -0.0199, -0.3193, -0.0907, -0.0221, 0.0355, 0.4057,
 0.0001, -0.0069, 0.0014, -0.0070, -0.0072, -0.0070, 0.0079, -0.0205, -0.0317, -0.0196, 0.0192, 0.0267, -0.1053, -0.1664, -0.0110,
-0.0068, 0.0102, 0.0061, 0.0006, 0.0115, 0.0051, -0.0034, 0.0087, -0.0854, -0.0152, -0.1505, -0.0443, 0.0525, 0.0865, 0.0046,
-0.0036, 0.0043, 0.0020, -0.0004, -0.0107, -0.0022, 0.0088, 0.0054, 0.0822, 0.0345, 0.0512, 0.1747, 0.0276, 0.0026, 0.0067,
 0.0016, -0.0049, 0.0023, 0.0039, 0.0016, 0.0007, -0.0063, 0.0073, 0.0083, -0.0229, 0.0091, -0.1088, -0.0460, 0.0019, -0.0074,
 0.0235, -0.0649, 0.0329, 0.0636, 0.0330, 0.0012, -0.0260, 0.1138, -0.0846, 0.0098, 0.0934, 0.0109, 0.0218, 0.0107, -0.0698,
-0.0036, 0.0124, 0.0087, 0.0314, 0.0187, 0.0238, 0.0111, 0.0916, 0.0159, -0.2371, -0.5103, 0.3053, -0.6420, -0.2928, -0.2459,
 0.0052, -0.0116, 0.0274, 0.0612, 0.0606, 0.0231, -0.0520, 0.2489, 0.2250, -0.0545, -0.3985, 0.0221, 0.0613, 0.7999, -0.1545,
-0.0085, -0.0111, -0.0033, -0.0065, -0.0180, -0.0105, 0.0004, 0.0596, 0.7184, 0.0227, 0.5336, 0.2137, -0.2915, 0.0687, -0.1014,
 0.0069, -0.0242, 0.0115, 0.0136, 0.0020, 0.0059, -0.0424, 0.0200, 0.0426, -0.1845, 0.0902, -0.8637, -0.3732, 0.0356, -0.1123,
-0.0000, 0.0002, -0.0003, -0.0001, 0.0002, -0.0002, 0.0012, 0.0002, -0.0008, 0.0016, 0.0026, -0.0012, 0.0020, 0.0018, 0.0016,
-0.0003, 0.0017, -0.0009, -0.0014, -0.0011, -0.0001, 0.0032, -0.0074, -0.0239, 0.0012, -0.0147, 0.0288, -0.0006, -0.0345, 0.0046,
 0.0001, 0.0010, -0.0002, -0.0002, 0.0005, 0.0002, 0.0011, -0.0026, -0.0249, 0.0040, -0.0194, 0.0143, 0.0193, -0.0033, 0.0064,
-0.0002, 0.0008, -0.0004, -0.0004, -0.0001, -0.0002, 0.0014, -0.0006, -0.0014, 0.0060, -0.0029, 0.0280, 0.0120, -0.0012, 0.0036,
-0.0000, 0.0001, -0.0001, -0.0002, -0.0002, -0.0000, 0.0002, -0.0007, -0.0007, -0.0005, 0.0003, 0.0006, -0.0027, -0.0042, -0.0001,
-0.0002, -0.0003, -0.0001, -0.0002, -0.0005, -0.0003, 0.0000, 0.0013, 0.0171, 0.0006, 0.0130, 0.0051, -0.0070, 0.0012, -0.0023,
 0.0002, -0.0006, 0.0003, 0.0003, 0.0001, 0.0001, -0.0010, 0.0005, 0.0011, -0.0045, 0.0022, -0.0211, -0.0091, 0.0009, -0.0028,
 0.0001, 0.0000, 0.0001, 0.0002, 0.0003, 0.0001, -0.0001, 0.0005, -0.0023, -0.0003, -0.0035, -0.0010, 0.0014, 0.0022, -0.0001,
-0.0000, 0.0000, -0.0000, -0.0000, -0.0000, -0.0000, 0.0000, -0.0000, 0.0000, 0.0001, -0.0000, 0.0005, 0.0002, -0.0000, 0.0001,
-0.0001, 0.0001, -0.0001, -0.0001, -0.0001, -0.0001, 0.0002, 0.0001, 0.0021, 0.0009, 0.0013, 0.0043, 0.0007, 0.0001, 0.0002,
 0.0000, -0.0001, 0.0000, 0.0000, 0.0000, 0.0000, -0.0001, 0.0001, 0.0001, -0.0006, 0.0003, -0.0027, -0.0012, 0.0001, -0.0004;
    
    theta = Eigen::VectorXd::Zero(n_param);
    alpha = Eigen::VectorXd::Zero(15);

    theta << 52.6334101099504, 10.2162277797894, 4188.82853290319, 1.03540441873538, 2.43711211624183, 0.000346393806324802, 1.54300006451501e-05, 1.00683497461433, 41.2049038660059, -938.638001034116, 9.95443496971824, -22.6452554201491, 0.181374103651703, -0.0569959139571596, 1.06043943182456, -365.189566987842, 5.58149507522191, -8.82355433095529, 0.101742357141362, -0.0223100797616370, 0.528471573951173, -86.2735882200421, 202.070159782202, -1.57122736630554, 0.508218360088234, -9.39701005855973, -2.09592364821546, 0.0378744331055520, -0.00538975260999602, 0.144703357204055, -0.0381754164895553, 0.0122611001906250, -0.227054969366253, -9.82036199459057e-05, 0.00263774819632151, -0.000574166812361116;
    alpha << -13.0999772196331, -67.9763961021685, -35.8240110287052, -344.065827192036, -106.035875684254, 284.221578905001,-183.443454278577, 250.413710818642, -1669.91124815472, 1685.55702611508, 792.646825267457, 502.901989343099, -2347.31579553679, 1548.98241544928, 1753.15081058907;

    Eigen::VectorXd test = Eigen::VectorXd::Zero(32);

    test = u_p + N * alpha;
    ROS_INFO_STREAM("test" << test);


    Kx = Eigen::RowVectorXd::Zero(n_state_aug);
    Ky = Eigen::RowVectorXd::Zero(n_state_aug);
    // kz = Eigen::RowVectorXd::Zero(n_state_aug);

    K0factor = 1.0/20.0*20.0;
    THETA0factor = 0.8;
    ALPHA0factor = 0.8;

    PRLS0factor=10e-6;
    Kx << 0.526620279669378,	0.262442002550444,	-4.66660887607784,	0.0718605138143464,	-0.112756794852737,	0.00130992082259127,	-0.000285134518981841;
    Ky << 0.5266, 0.2624, -0.9052, 0.1117, -0.0309, 0.0050, -0.0003;
    // kz << 0, 0;

    //initial gain
    Kx = Kx * K0factor; 
    // Ky = Ky * K0factor;

    ROS_INFO_STREAM("Initial Kx" << Kx);
    ROS_INFO_STREAM("Initial Ky" << Ky);

    theta = theta * THETA0factor;
    alpha = alpha * ALPHA0factor;

    ROS_INFO_STREAM("initial alpha" << alpha);


    countk = 0;
    prls = Eigen::MatrixXd::Identity(theta.size(),theta.size()) * PRLS0factor;

    // prls = Eigen::MatrixXd::Identity(alpha.size(),alpha.size()) * PRLS0factor;
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
//         Pout2(indx(i, 0), indx(i, 1)) = theta(k);
//         k++;
//     }

//     Pout = Pout1 + Pout2 + Pout2.transpose();

//     return Pout; 
// }

// void RLLQTController::UpdateRLS(Eigen::VectorXd& alpha, std::vector<Eigen::VectorXd>& phi, Eigen::Vector3d& Erls, Eigen::MatrixXd& prls, double& mu)
// {
//     double phiT_P_phi = phi[0].transpose()* N * prls * N.transpose()* phi[0];
//     if (phiT_P_phi > 1e-12) 
//     {
//         alpha = alpha + (prls * N.transpose() * phi[0] * Erls.x()) / (mu + phiT_P_phi);
//         prls = (1.0/mu) * prls - (1.0/mu) * (prls * N.transpose() * phi[0] * phi[0].transpose() * N * prls) / (mu + phiT_P_phi);
//     }
// }

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

        countk = 0;
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

void RLLQTController::Calc_reward(Eigen::VectorXd& old_state, Eigen::Vector3f& old_u, double& Qe, double& R)
{
     reward.x() = - old_state.transpose() * Qx * old_state - old_u.x() * R * old_u.x();
    // reward.y() = - state_x.transpose() * Qy * state_x - u.y() * R * u.y();
    // ROS_INFO_STREAM("reward x"  << reward.x());

    // Reward Pub
    geometry_msgs::Vector3 reward_msg;
    reward_msg.x = reward.x();
    reward_msg.y = reward.y();
    reward_pub.publish(reward_msg);
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
        u.x() = - Kx * state_x + excitation.x();
        u.y() = - Ky * state_y;
        u.z() = 0.0;

        // L_{k}
        augmented_state_x << state_x, u.x();

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

            // Phi
            phi.resize(3);
            phi[0] = old_bar_x - pow(gamma, h) * bar_x;
            // phi[1] = old_bar_y - gamma * bar_y;

            // Reward
            Calc_reward(old_state_x, old_u, Qe, R);

            // error
            // Erls.x() = reward.x() - phi[0].transpose() * u_p - phi[0].transpose() * N * alpha;
            Erls.x() = reward.x() -  phi[0].transpose() * theta;
            // Erls.y() = reward.y() - phi[1].transpose() * theta;
            ROS_INFO_STREAM("erls" << Erls.x());

            // RLS
            UpdateRLS(theta, phi, Erls, prls, mu);

            if (countk > 100)
            {
                // theta = u_p + N*alpha;
                ROS_INFO_STREAM("theta"<< theta);
                UpdateGain(theta, A_dlyap.transpose()*sqrt(pow(gamma, h)), Q_dlyap);
            
            }

        }
   
        // Control Signal Pub
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0f, 73};
        vel_pub.publish(vel_msg);

        // Aug C
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
        old_state_x = state_x;
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
