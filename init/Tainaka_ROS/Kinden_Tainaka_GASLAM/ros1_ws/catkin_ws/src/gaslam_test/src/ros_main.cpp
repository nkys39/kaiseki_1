
#include "gaslam_test_node.hpp"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaslam_test_node");
    GaSlamNode gaslam_node;
    gaslam_node.onInitialize();
    gaslam_node.onActivate();
    gaslam_node.onExecute();
    ros::spin();
    gaslam_node.onDeactivate();
    return 0;
} 
