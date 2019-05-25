#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include "road_map/RoadMap.hpp"
#include <random>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

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
    void samplePath();
    void samplingPath();
    bool imageGenerator(Eigen::Affine3d&, const ros::TimerEvent&, cv_bridge::CvImagePtr);
    void clipPath(std::vector<Eigen::Vector2d>::iterator& source_start,
    			std::vector<Eigen::Vector2d>::iterator& source_end,
				std::vector<Eigen::Vector2d>& source,
				std::vector<Eigen::Vector2d>& dest,
				nav_msgs::Path::Ptr& path_ptr);
    void setCliper(std::vector<Eigen::Vector2d>::iterator& it, std::vector<Eigen::Vector2d>& source, std::vector<Eigen::Vector2d>::iterator& start, std::vector<Eigen::Vector2d>::iterator& it_end);
    void pubnewpath(const ros::Time&);


    ros::Subscriber sign_subscriber_;      //
	std::string sign_;
	void signCallback(const std_msgs::String::ConstPtr& msg);                         //

    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    ros::ServiceClient reset_episode_client_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    ros::Timer timer_;

    nav_msgs::Path::Ptr path_{new nav_msgs::Path};
    nav_msgs::Path::Ptr part_of_path_{new nav_msgs::Path};
    std::vector<std::vector<Eigen::Vector3d>> samplePath_{5};
    RoadMap map_{49.01439, 8.41722};
    Eigen::Vector3d center_;
    int switcher{1};
    double timerecoder_;
    std::vector<Eigen::Vector2d> path_vector_;
    std::vector<Eigen::Vector2d> path_vector_whole_;
    std::vector<Eigen::Vector2d>::iterator prev_pos_index_;
    std::vector<Eigen::Vector2d>::iterator prev_pos_whole_index_;
    bool sample_flag_ = false;
    bool in_reset_{false};

};
} // namespace path_publisher_ros_tool
