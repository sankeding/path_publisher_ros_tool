#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include "road_map/RoadMap.hpp"

#include "path_publisher_ros_tool/PathPublisherInterface.h"

namespace path_publisher_ros_tool {

class PathPublisher {

    using Interface = PathPublisherInterface;

    using Msg = std_msgs::Header;

public:
    PathPublisher(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackTimer(const ros::TimerEvent&);
    void reconfigureRequest(const Interface::Config&, uint32_t);

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    ros::Timer timer_;

    nav_msgs::Path::Ptr path_{new nav_msgs::Path};
    RoadMap map_{0., 0.};
};
} // namespace path_publisher_ros_tool
