#pragma once

#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
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
#include <std_msgs/Int8.h>
#include <string>
#include <algorithm>

#include "path_publisher_ros_tool/PathPublisherInterface.h"

#include <neverdrive_state_control_ros_tool/State.h>

namespace path_publisher_ros_tool {

class PathPublisher {

    using Interface = PathPublisherInterface;
    using Msg = std_msgs::Header;

public:
    PathPublisher(ros::NodeHandle, ros::NodeHandle);

private:

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


    /***********************/
    void pathPublishCallback(const ros::TimerEvent&);
    void switchWantedMapCallback(const ros::TimerEvent&);
	void readAllMaps( const std::string& );
    void setSignCallback(const std_msgs::Int8::ConstPtr& msg);                      //
    void detectObstacleCallback(const std_msgs::String& msg);
    void initialPartOfPath(const int actual_map);
    void getVehiclePose();
    bool nearCenter();
    int nearTurnPoint();                        //return 0:A 1:B 2:C 3:D



	const std::vector<std::string> all_maps_name_{"1_A_B_forward", "2_C_D_forward", "3_A_C_right", "4_B_D_left",
                                                  "5_A_left",      "6_B_right",     "7_C_left",    "8_D_right",
                                                  "9_A_B_forward_reverse","10_C_D_forward_reverse","11_exit" };
	std::vector<std::vector<Eigen::Vector2d>> all_path_vector_whole_{all_maps_name_.size()};


	ros::Timer switch_wanted_map_timer_;
    ros::Timer path_publish_timer_;
    ros::Subscriber set_path_subscriber_;  //
    ros::Subscriber detect_obstacle_subscriber_;
    std::string obstacle_detection_;
	int set_map_;
	int sign_;
	int wanted_map_;
	int actual_map_;
	int actual_map_index_;       //actual_map_index = actual_map - 1

    Eigen::Affine3d vehicle_pose_;
    Eigen::Vector3d center_;


    Eigen::Vector2d map_center_, map_A_, map_B_, map_C_, map_D_;
    bool near_center_, pass_center_;





/****************************/
    Interface interface_;
    dynamic_reconfigure::Server<Interface::Config> reconfigureServer_;

    ros::ServiceClient reset_episode_client_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_{tfBuffer_};
    tf2_ros::TransformBroadcaster tfBroadcaster_;


    nav_msgs::Path::Ptr path_{new nav_msgs::Path};
    nav_msgs::Path::Ptr part_of_path_{new nav_msgs::Path};
    std::vector<std::vector<Eigen::Vector3d>> samplePath_{5};
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
