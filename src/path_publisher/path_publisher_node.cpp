#include "path_publisher.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "path_publisher_node");

    path_publisher_ros_tool::PathPublisher path_publisher(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return EXIT_SUCCESS;
}
