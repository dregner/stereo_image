#include <darknet_disparity.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "darknet_distance_node");
    DarknetDisparity DarknetDistance;
    while (ros::ok()) {
        ros::spinOnce();

    }
    return 0;
}
