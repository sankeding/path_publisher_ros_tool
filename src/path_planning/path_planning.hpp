#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "path_publisher_ros_tool/PathPlanningInterface.h"

namespace path_publisher_ros_tool {

class PathPlanning {

    using Interface = PathPlanningInterface;

    using Msg = std_msgs::Header;

public:
    PathPlanning(ros::NodeHandle, ros::NodeHandle);

private:
    void callbackSubscriber(const Msg::ConstPtr& msg);
    void reconfigureRequest(const Interface::Config&, uint32_t);

    /**************************************/
    void pathPublishCallback(const ros::TimerEvent&);

    ros::Timer planned_path_publish_timer_;
    ros::Publisher planned_path_publisher;


    /***********************************/

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace path_publisher_ros_tool
