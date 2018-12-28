#include "path_publisher.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include "path_publisher_ros_tool/rl_state_reward.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

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

	reset_episode_client_ = nhPrivate.serviceClient<ResetEpisode>(interface_.episode_service);

    if (interface_.mode != "train" && interface_.mode != "test")
    {
		ROS_ERROR_STREAM("please check you spell of mode.");
		ros::shutdown();
		return;
    }else{
    	//    initial vehicle pose
		Eigen::Affine3d vehicle_pose;
		while(1){
			try {
				const geometry_msgs::TransformStamped tf_ros =
					tfBuffer_.lookupTransform(interface_.frame_id_map, interface_.frame_id_vehicle, ros::Time(0));
				vehicle_pose = tf2::transformToEigen(tf_ros);
				break;
			} catch (const tf2::TransformException& e){
				ROS_WARN_STREAM(e.what());
			}
		}
		if (interface_.mode == "test"){

//  load path from .osm file
		map_.loadFromFile(interface_.path_to_map + interface_.map_name);
		double x, y;
		for (int i = 0; i < (int)map_.trajectories.at(1).size(); i++){
			map_.getVertexMeters(1, i, x, y);
			pose_ros.pose.position.x = x;
			pose_ros.pose.position.y = y;
			path_vector_whole_.emplace_back(Eigen::Vector2d(x, y));
			path_->poses.emplace_back(pose_ros);
		}
		ROS_DEBUG_STREAM("load road finished." << std::endl <<
				"the whole path length: " << path_->poses.size());

//		set prev_pos_index mark
		const Eigen::Vector3d vehicle_position = vehicle_pose.translation();
		Eigen::Vector3d vehicle_frame_unit_x = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
		vehicle_frame_unit_x.z() = 0.0;
		vehicle_frame_unit_x = vehicle_frame_unit_x.normalized();
		const Eigen::Vector2d pos2d = (vehicle_position + vehicle_frame_unit_x * interface_.kos_shift).head<2>();
		//   find the closet point to vehicle on path
		prev_pos_whole_index_ = boost::range::min_element(
				path_vector_whole_, [&pos2d](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
			return (le - pos2d).squaredNorm() < (re - pos2d).squaredNorm();
		});
		std::vector<Eigen::Vector2d>::iterator path_start, path_end;
		setCliper(prev_pos_whole_index_, path_vector_whole_, path_start, path_end);
		clipPath(path_start, path_end, path_vector_whole_, path_vector_, part_of_path_);
		ROS_DEBUG_STREAM("whole path length: " << path_vector_whole_.size() << std::endl <<
							"part of path length: " << path_vector_.size() << std::endl <<
							"path messsage length: " << part_of_path_->poses.size());
		prev_pos_index_ = boost::range::min_element(
						path_vector_, [&pos2d](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
					return (le - pos2d).squaredNorm() < (re - pos2d).squaredNorm();
				});
		}else if (interface_.mode == "train"){
	//		generate a new path and publish
			samplePath();
	//initial pose message
			geometry_msgs::PoseStamped pose_ros;
			pose_ros.pose.orientation.x = 0.0;
			pose_ros.pose.orientation.y = 0.0;
			pose_ros.pose.orientation.z = 0.0;
			pose_ros.pose.orientation.w = 0.0;
			pose_ros.pose.position.z = 0.0;
			pose_ros.header = path_->header;
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
				Eigen::Vector3d lpP(lp.back()), rpP(rp.back());
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
				path_vector_.emplace_back(Eigen::Vector2d(p_to_transform[0], p_to_transform[1]));
			}
		}
	}

    timer_ = nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &PathPublisher::callbackTimer, this);

    rosinterface_handler::showNodeInfo();
}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	const double vz = boost::math::sign(a.cross(b).z());
	return vz * std::acos(a.normalized().dot(b.normalized()));
}

void PathPublisher::samplePath(){
//delete all the exist data
	for (auto& onePath: samplePath_) onePath.clear();
//all sample path has base length of 3*pi/2 (m), generate in vehicle coordination, assume that vehicle point to x positive
	double delta = interface_.delta_sigma;
	static std::normal_distribution<double> n(0, interface_.radius_noise);
	static std::normal_distribution<double> la(0, interface_.shift_noise);
	static std::default_random_engine e;
	double noise = n(e), r = 0., la_shift = la(e);
	la_shift = boost::algorithm::clamp(la_shift, -2. * interface_.shift_noise, 2.*interface_.shift_noise);
	noise = boost::algorithm::clamp(noise, -0.4, 0.4);
//	ROS_DEBUG_STREAM("noise of radius: " << noise);
	double multiplier = 1.;
	if (interface_.env == "carla") multiplier = 7.;
	if (interface_.env != "carla" and interface_.env != "anicar")
		ROS_WARN_STREAM("parameter:env not well defined! Using anicar instead");
	la_shift *= multiplier;
//generate first kind of sample path, radius 1.5 (m)
	r = (1.5 + noise) * multiplier;
	for(double angle = 3.*M_PI/4.; angle > -M_PI/4.; angle -= delta){
		samplePath_[0].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r - r + la_shift, 0.0));
	}
//	generate second kind of sample path, radius 3 (m)
	r = (3. + noise) * multiplier;
	for(double angle = 5.*M_PI/8.; angle > M_PI/8.; angle -= delta){
		samplePath_[1].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r - r + la_shift, 0.0));
	}
//	generate third kind of sample path, radius infinit (m)
	for(double d = multiplier * -3.*M_PI/8.; d < multiplier * 9.*M_PI/8.; d += 0.02){
		samplePath_[2].push_back(Eigen::Vector3d(d, la_shift, 0.));
	}
//	generate fourth kind of sample path, radius 3 (m)
	r = (3. + noise) * multiplier;
	for(double angle = 11.*M_PI/8.; angle < 15.*M_PI/8.; angle += delta){
		samplePath_[3].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r + r + la_shift, 0.0));
	}

//	generate fifth kind of sample path, radius 1.5 (m)
	r = (1.5 + noise) * multiplier;
	for(double angle = 5.*M_PI/4.; angle < 9.*M_PI/4.; angle += delta){
		samplePath_[4].push_back(Eigen::Vector3d(std::cos(angle)*r, std::sin(angle)*r + r + la_shift, 0.0));
	}
//	ROS_DEBUG_STREAM("first path of sample path has length: " << samplePath_[0].size());
}

bool PathPublisher::imageGenerator(Eigen::Affine3d& vehicle_pose, const ros::TimerEvent& timer_event){
	const Eigen::Vector3d vehicle_position = vehicle_pose.translation();
	Eigen::Vector3d vehicle_frame_unit_x = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
	vehicle_frame_unit_x.z() = 0.0;
	vehicle_frame_unit_x = vehicle_frame_unit_x.normalized();
	const Eigen::Vector2d vehicle_pose2d = (vehicle_position + vehicle_frame_unit_x * interface_.kos_shift).head<2>();
	Eigen::Matrix3d map_to_vehicle(vehicle_pose.rotation().inverse());
	std::vector<Eigen::Vector2d> points_list;
	for (auto p: path_vector_){
//		find the points within image covered area, get the relativ position
		if ((p - vehicle_pose2d).norm() < interface_.local_scope * 1.41 / 2){
			p = p - vehicle_pose2d;
			Eigen::Vector3d pose3d(Eigen::Vector3d::Zero());
			pose3d.head<2>() = p;
			pose3d = map_to_vehicle * pose3d;
			points_list.emplace_back(pose3d.head<2>());
		}
	}

	ROS_ERROR_STREAM(points_list.size() << " points in local scope found");

	if (points_list.size() <= interface_.least_points) return false;

//	publish the local image only when using anicar
	if (interface_.env == "anicar" || interface_.env == "carla"){
        ROS_ERROR_STREAM("publishing image");

        const int img_cells = std::round(interface_.local_scope / interface_.point_distance);
		cv::Mat img(img_cells, img_cells, CV_32FC1, cv::Scalar(0));

		int center_col = img_cells / 2;
		int center_row = img_cells / 2;
	//	fill the pixel
		for (const auto& p: points_list){
			int rel_col = std::round(p.y() / interface_.point_distance);
			if (center_col + rel_col >= img_cells or center_col + rel_col < 0) continue;
			int rel_row = std::round(p.x() / interface_.point_distance);
			if (center_row + rel_row >= img_cells or center_row + rel_row < 0) continue;
			float* imgrow = img.ptr<float>(center_row + rel_row);
			imgrow[center_col + rel_col] = 255.;
		}
	//	cv::imshow("Local Path", img);
	//	cv::waitKey(1);
		cv_bridge::CvImagePtr cv_ptr{new cv_bridge::CvImage};
		cv_ptr->header.stamp = timer_event.current_expected;
	//	cv_ptr->header.frame_id = interface_.frame_id_vehicle;
		cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		cv_ptr->image = img;
		cv_ptr->toImageMsg();
		interface_.image_publisher.publish(cv_ptr);
	}
	return true;
}

void PathPublisher::callbackTimer(const ros::TimerEvent& timer_event) {
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
	//shift vehicle position towards x direction kos_shift long
	const Eigen::Vector3d vehicle_position = vehicle_pose.translation();
	Eigen::Vector3d vehicle_frame_unit_x = vehicle_pose.rotation() * Eigen::Vector3d::UnitX();
	vehicle_frame_unit_x.z() = 0.0;
	vehicle_frame_unit_x = vehicle_frame_unit_x.normalized();
	const Eigen::Vector2d shifted_vehicle_position = (vehicle_position + vehicle_frame_unit_x * interface_.kos_shift).head<2>();
//   find the closet point to vehicle on path
	auto it = boost::range::min_element(
			path_vector_, [&shifted_vehicle_position](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
		return (le - shifted_vehicle_position).squaredNorm() < (re - shifted_vehicle_position).squaredNorm();
	});
    path_publisher_ros_tool::rl_state_reward reward_msg;
	const double dis = (shifted_vehicle_position - *it).norm();
	Eigen::Vector3d target_direction;
	target_direction.head<2>() = *(it + 3) - *(it - 3);
	target_direction.z() = 0.;
	Eigen::Vector3d shifted_v_p_3d;
	shifted_v_p_3d.head<2>() = shifted_vehicle_position;
	shifted_v_p_3d.z() = 0.;
	Eigen::Vector3d closet_3d;
	closet_3d.head<2>() = *it;
	closet_3d.z() = 0.;
	 const double vz_dist =
	        boost::math::sign(target_direction.cross(closet_3d - shifted_v_p_3d).z());
	reward_msg.angle = signedAngleBetween(vehicle_frame_unit_x, target_direction);
	reward_msg.dis = vz_dist * dis;
    reward_msg.done = true;//that means stop the episode and start a new one


    interface_.reward_publisher.publish(reward_msg);
//	generate the image of local path
	bool path_in_scope = PathPublisher::imageGenerator(vehicle_pose, timer_event);

//	if vehicle dosen't move, generate a new path
	bool vehicle_stuck = false;
	if (not sample_flag_) timerecoder_=timer_event.current_expected.toSec();
	else if (timer_event.current_expected.toSec() - timerecoder_ > interface_.stuck_time) vehicle_stuck = true;

	//	TODO: check vehicle get out of boundary or not

	if (interface_.mode == "test"){
//    decide to pubnish a new path or not
//		initial path at least once
		int index_distance = std::distance(prev_pos_index_, it);
		if (sample_flag_){
			//		if still not drive so far abandon to pubnish new path
			if (std::abs(index_distance * interface_.point_distance) < interface_.drive_distance) return;
		}else{
			sample_flag_ = true;
			interface_.path_publisher.publish(part_of_path_);
			return;
		}
//		initial a part of path to publish
		path_->header.stamp = timer_event.current_expected;
//		clip a part of path to publish
		std::vector<Eigen::Vector2d>::iterator path_start;
		std::vector<Eigen::Vector2d>::iterator path_end;
//		reset prev pose whole index
		for(int i = 0; i < index_distance; i++){
			if (prev_pos_whole_index_ != path_vector_whole_.end() - 1)
				prev_pos_whole_index_++;
			else
				prev_pos_whole_index_ = path_vector_whole_.begin();
		}
		setCliper(prev_pos_whole_index_, path_vector_whole_, path_start, path_end);
		ROS_DEBUG_STREAM("pos index moving: " << index_distance << std::endl<<
								"current whole path index mark: " << std::distance(path_vector_whole_.begin(), prev_pos_whole_index_) << std::endl <<
								"current path start index: " << std::distance(path_vector_whole_.begin(), path_start) << std::endl <<
								"current path end index: " << std::distance(path_vector_whole_.begin(), path_end));
		clipPath(path_start, path_end, path_vector_whole_, path_vector_, part_of_path_);
		//		set new mark of prev pos index
		prev_pos_index_ = boost::range::min_element(
					path_vector_, [&shifted_vehicle_position](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
				return (le - shifted_vehicle_position).squaredNorm() < (re - shifted_vehicle_position).squaredNorm();
			});
		interface_.path_publisher.publish(part_of_path_);
	}else if (interface_.mode == "train"){
//		flag to ensure sample path at least to be initialized once, and path in the image scope
		if (sample_flag_ and path_in_scope and not vehicle_stuck){
	//		if still not drive enough far abandon to pubnish new path
			int index_distance = std::distance(path_vector_.begin(), it);
			ROS_DEBUG_STREAM("local path length: " << path_vector_.size() << std::endl <<
							"current distance: " << index_distance);
			if ((index_distance / double(path_vector_.size())) < 0.7) return;
		}else sample_flag_ = true;

//		generate a new path and publish
		samplePath();
		path_.reset(new nav_msgs::Path);
		path_vector_.clear();
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
//transform the path to map frame
		static std::normal_distribution<double> n(0, M_PI*interface_.rotation_noise/360.);
		static std::default_random_engine e;
		double noise = n(e);
		noise = boost::algorithm::clamp(noise, -M_PI*2./18., M_PI*2./18.);
//		ROS_DEBUG_STREAM("noise of angle: " << noise);
		Eigen::Affine3d NoiseTransform(Eigen::AngleAxisd(noise, Eigen::Vector3d::UnitZ()));
		Eigen::Matrix4d NewTransform = (vehicle_pose*NoiseTransform).matrix();
		std::vector<std::vector<Eigen::Vector3d>>::iterator path_vector;
//find the sample path whose end position is the closet to center of map
		if (interface_.env == "carla"){
			static std::uniform_int_distribution<int> which_path(0, samplePath_.size() - 1);
			path_vector = samplePath_.begin() + which_path(e);
		}else{
			path_vector = boost::range::min_element(
					samplePath_, [&](const std::vector<Eigen::Vector3d>& lp,
									const std::vector<Eigen::Vector3d>& rp){
				Eigen::Vector3d lpP(lp.back()), rpP(rp.back());
				return ((NewTransform * Eigen::Vector4d(lpP[0], lpP[1], lpP[2], 1)).head<3>() - center_).squaredNorm()<
						((NewTransform * Eigen::Vector4d(rpP[0], rpP[1], rpP[2], 1)).head<3>() - center_).squaredNorm();
			});
		}
//		nav_msgs::Path::Ptr path;
		for(const auto& p: *path_vector){
			Eigen::Vector4d p_to_transform(p[0], p[1], p[2], 1);
			p_to_transform = NewTransform * p_to_transform;
			pose_ros.pose.position.x = p_to_transform[0];
			pose_ros.pose.position.y = p_to_transform[1];
			path_->poses.emplace_back(pose_ros);
			path_vector_.emplace_back(Eigen::Vector2d(p_to_transform[0], p_to_transform[1]));
		}
//		ROS_DEBUG_STREAM("publish a path of " << path_->poses.size() << " long.");

//		TODO: call reset episode service
		interface_.path_publisher.publish(path_);
		timerecoder_=timer_event.current_expected.toSec();
	}
}

void PathPublisher::clipPath(std::vector<Eigen::Vector2d>::iterator& source_start,
							std::vector<Eigen::Vector2d>::iterator& source_end,
							std::vector<Eigen::Vector2d>& source,
							std::vector<Eigen::Vector2d>& dest,
							nav_msgs::Path::Ptr& path_ptr){
	//initial pose message
	geometry_msgs::PoseStamped pose_ros;
	pose_ros.pose.orientation.x = 0.0;
	pose_ros.pose.orientation.y = 0.0;
	pose_ros.pose.orientation.z = 0.0;
	pose_ros.pose.orientation.w = 0.0;
	pose_ros.pose.position.z = 0.0;
	pose_ros.header = path_->header;
	dest.clear();
	path_ptr.reset(new nav_msgs::Path);
	path_ptr->header = path_->header;
	for(auto ele = source_start; ele != source_end;){
		dest.emplace_back(*ele);
		pose_ros.pose.position.x = (*ele)[0];
		pose_ros.pose.position.y = (*ele)[1];
		path_ptr->poses.emplace_back(pose_ros);
		if (ele == source.end() - 1){
			ele = source.begin();
		}else ele++;
	}
}

void PathPublisher::setCliper(std::vector<Eigen::Vector2d>::iterator& it, std::vector<Eigen::Vector2d>& source, std::vector<Eigen::Vector2d>::iterator& it_start, std::vector<Eigen::Vector2d>::iterator& it_end){
//		clip a part of path to publish
	auto path_start = it;
	int index_dis = 1;
//		find the beginning of this part first
	for(--path_start; path_start != it;){
		if ((index_dis * interface_.point_distance) > interface_.path_length/4.0)
			break;
//			link end to start
		if (path_start != source.begin()){
			path_start--;
			index_dis++;
		}else{
			path_start = source.end() - 1;
			index_dis++;
		}
	}
	it_start = path_start;
//		find the end of this part
	auto path_end = it;
	index_dis = 1;
	for(++path_end; path_end != it;){
		if((index_dis * interface_.point_distance) > 3.0*interface_.path_length/4.0)
			break;
//			link end to start
		if (path_end != source.end() - 1){
			path_end++;
			index_dis++;
		}else{
			index_dis++;
			path_end = source.begin();
		}
	}
	it_end = path_end;
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPublisher::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
