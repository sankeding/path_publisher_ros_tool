#include "path_publisher.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>

namespace path_publisher_ros_tool {

PathPublisher::PathPublisher(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    /**
     * Initialization
     */
    interface_.fromParamServer();


    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/PathPublisher.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&PathPublisher::reconfigureRequest, this, _1, _2));

//  initial path_
    center_ = Eigen::Vector3d(interface_.center_x, interface_.center_y, 0.);
    path_->header.frame_id = interface_.frame_id_map;
	path_->header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_ros;
	pose_ros.pose.orientation.x = 0.0;
	pose_ros.pose.orientation.y = 0.0;
	pose_ros.pose.orientation.z = 0.0;
	pose_ros.pose.orientation.w = 0.0;
	pose_ros.pose.position.z = 0.0;
	pose_ros.header = path_->header;
    if (interface_.mode != "train" && interface_.mode != "test")
    {
		ROS_ERROR_STREAM("please check you spell of mode.");
		ros::shutdown();
		return;
    }else if (interface_.mode == "test"){
//  load path from .osm file
		map_.loadFromFile(interface_.path_to_map + interface_.map_name);
		double x, y;
		for (int i = 0; i > (int)map_.trajectories.at(1).size(); i++){
			map_.getVertexMeters(1, i, x, y);
			pose_ros.pose.position.x = x;
			pose_ros.pose.position.y = y;
			path_->poses.emplace_back(pose_ros);
		}
    }

    timer_ = nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &PathPublisher::callbackTimer, this);

    rosinterface_handler::showNodeInfo();
}

void PathPublisher::samplePath(){
//delete all the exist data
	for (auto& onePath: samplePath_) onePath.clear();
//all sample path has length of 3*pi/2 (m), generate in vehicle coordination, assume that vehicle point to x positive
	double delta = interface_.delta_sigma;
	static std::normal_distribution<double> n(0, interface_.radius_noise);
	static std::default_random_engine e;
	double noise = n(e), r = 0.;
	noise = boost::algorithm::clamp(noise, -0.4, 0.4);
	ROS_DEBUG_STREAM("noise of radius: " << noise);
//generate first kind of sample path, radius 1.5 (m)
	r = 1.5 + noise;
	for(double angle = 0.; angle < M_PI; angle += delta){
		samplePath_[0].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r - r, 0.0));
	}
//	generate second kind of sample path, radius 3 (m)
	r = 3. + noise;
	for(double angle = M_PI/4.; angle < M_PI*3./4.; angle += delta){
		samplePath_[1].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r - r, 0.0));
	}
//	generate third kind of sample path, radius infinit (m)
	for(double d = 3.*M_PI/4.; d > -3*M_PI/4.; d -= 0.02){
		samplePath_[2].push_back(Eigen::Vector3d(d, 0., 0.));
	}
//	generate fourth kind of sample path, radius 3 (m)
	r = 3. + noise;
	for(double angle = 7.*M_PI/4.; angle > 5.*M_PI/4.; angle -= delta){
		samplePath_[3].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r + r, 0.0));
	}

//	generate fifth kind of sample path, radius 1.5 (m)
	r = 1.5 + noise;
	for(double angle = 2.*M_PI; angle > M_PI; angle -= delta){
		samplePath_[4].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r + r, 0.0));
	}
	ROS_DEBUG_STREAM("first path of sample path has length: " << samplePath_[0].size());
}

void PathPublisher::callbackTimer(const ros::TimerEvent& timer_event) {
	path_->header.stamp = timer_event.current_expected;
	if (interface_.mode == "test"){
		interface_.path_publisher.publish(path_);
	}else if (interface_.mode == "train"){
		samplePath();
		path_.reset(new nav_msgs::Path);
		path_->header.frame_id = interface_.frame_id_map;
		path_->header.stamp = timer_event.current_expected;
//initial pose message
		geometry_msgs::PoseStamped pose_ros;
		pose_ros.pose.orientation.x = 0.0;
		pose_ros.pose.orientation.y = 0.0;
		pose_ros.pose.orientation.z = 0.0;
		pose_ros.pose.orientation.w = 0.0;
		pose_ros.pose.position.z = 0.0;
		pose_ros.header = path_->header;
//get the vehicle position
		Eigen::Affine3d vehicle_pose;
		try {
			const geometry_msgs::TransformStamped tf_ros =
				tfBuffer_.lookupTransform(interface_.frame_id_map, interface_.frame_id_vehicle, ros::Time(0));
			vehicle_pose = tf2::transformToEigen(tf_ros);
		} catch (const tf2::TransformException& e){
			ROS_WARN_STREAM(e.what());
			return;
		}
//transform the path to map frame
		static std::normal_distribution<double> n(0, M_PI*interface_.rotation_noise/360.);
		static std::default_random_engine e;
		double noise = n(e);
		noise = boost::algorithm::clamp(noise, -M_PI*2./18., M_PI*2./18.);
		ROS_DEBUG_STREAM("noise of angle: " << noise);
		Eigen::Affine3d NoiseTransform(Eigen::AngleAxisd(noise, Eigen::Vector3d::UnitZ()));
		Eigen::Matrix4d NewTransform = (vehicle_pose*NoiseTransform).matrix();
//find the sample path whose end position is the closet to center of map
		auto const& path_vector = boost::range::min_element(
				samplePath_, [&](const std::vector<Eigen::Vector3d>& lp,
								const std::vector<Eigen::Vector3d>& rp){
			Eigen::Vector3d lpP(lp.front()), rpP(rp.front());
			return ((NewTransform * Eigen::Vector4d(lpP[0], lpP[1], lpP[2], 1)).head<3>() - center_).squaredNorm()<
					((NewTransform * Eigen::Vector4d(rpP[0], rpP[1], rpP[2], 1)).head<3>() - center_).squaredNorm();
		});
//		nav_msgs::Path::Ptr path;
		for(const auto& p: *path_vector){
			Eigen::Vector4d p_to_transform(p[0], p[1], p[2], 1);
			p_to_transform = NewTransform * p_to_transform;
			pose_ros.pose.position.x = p_to_transform[0];
			pose_ros.pose.position.y = p_to_transform[1];
			path_->poses.emplace_back(pose_ros);
		}
		ROS_DEBUG_STREAM("publish a path of " << path_->poses.size() << " long.");
		interface_.path_publisher.publish(path_);
	}
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPublisher::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
