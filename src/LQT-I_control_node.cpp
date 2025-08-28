#include "rl_control/LQT-I_control_node.hpp"



int main(int argc, char** argv) {
    ros::init(argc, argv, "lqt_i_control_node");
    ros::NodeHandle nh;
    ros::spin();
    return 0;
}
