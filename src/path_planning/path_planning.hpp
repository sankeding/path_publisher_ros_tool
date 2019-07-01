#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>


#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "road_map/RoadMap.hpp"

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TransformStamped.h>

#include "path_publisher_ros_tool/PathPlanningInterface.h"

namespace path_publisher_ros_tool {

class PathPlanning {

    using Interface = PathPlanningInterface;

    using Msg = std_msgs::Header;

public:
    PathPlanning(ros::NodeHandle, ros::NodeHandle);

private:
    //void callbackSubscriber(const Msg::ConstPtr& msg);
    void reconfigureRequest(const Interface::Config&, uint32_t);

    /**************************************/
    void initializePath(const std::string&);
    void callbackSubscriber(const nav_msgs::OccupancyGrid::ConstPtr&);
    void updatePath(const ros::TimerEvent&);
    void publishPath(const ros::TimerEvent&);
    double getPotential(const double);
    double cross2d(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::Timer update_timer_, publish_timer_;

    double theta_;
    //bool activate_;
    nav_msgs::Path path_;
    nav_msgs::OccupancyGrid map_;

    /***********************************/

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
};
} // namespace path_publisher_ros_tool
