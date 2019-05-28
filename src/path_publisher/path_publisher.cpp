#include "path_publisher.hpp"
#include <tf2_eigen/tf2_eigen.h>
#include <cmath>
#include <boost/algorithm/clamp.hpp>
#include <boost/math/special_functions/sign.hpp>
#include <boost/range/algorithm/min_element.hpp>


namespace path_publisher_ros_tool {

PathPublisher::PathPublisher(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate}, map_center_(7.0, 3.1),
          map_A_(8.5, 5.45), map_B_(4.56, 3.36), map_C_(6.3, 0.24), map_D_(9.72, 2.07),
          near_center_{false},pass_center_{false},sign_{"NO_SIGN"}{

    /**
     * Initialization
     */
    interface_.fromParamServer();
    wanted_map_ = interface_.wanted_map;
    actual_map_ = interface_.actual_map;
    actual_map_index_ = interface_.actual_map -1;

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/PathPublisher.if file.
     * Don't forget to register your callbacks here!
     */

    set_path_subscriber_ = nhPrivate.subscribe("/set_path_topic",
    								        1,
    								        &PathPublisher::setPath_Callback,
    								        this);                                           //

    reconfigureServer_.setCallback(boost::bind(&PathPublisher::reconfigureRequest, this, _1, _2));

//  initial path_
    center_ = Eigen::Vector3d(interface_.center_x, interface_.center_y, 0.);             //default x=3,y=5
    readAllMaps(interface_.path_to_map);   //load maps to two-dimensional vector
    initialPartOfPath(actual_map_);

    path_publish_timer_ = nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &PathPublisher::pathPublishCallback, this);
    switch_wanted_map_timer_ = nhPrivate.createTimer(ros::Rate(10), &PathPublisher::switchWantedMapCallback, this);

    rosinterface_handler::showNodeInfo();
}


void PathPublisher::setPath_Callback(const std_msgs::Int8::ConstPtr& msg){
	set_path_ = msg->data;         //wanted_map = msg->data
	ROS_DEBUG_STREAM("received set_path info:"<< set_path_ <<std::endl);
}


void PathPublisher::pathPublishCallback(const ros::TimerEvent & timerEvent) {

    int index_distance{0};    //how many points the car go through

    //get the vehicle position
    getVehiclePose();
    //shift vehicle position towards x direction kos_shift long
    const Eigen::Vector3d vehicle_position = vehicle_pose_.translation();
    Eigen::Vector3d vehicle_frame_unit_x = vehicle_pose_.rotation() * Eigen::Vector3d::UnitX();
    vehicle_frame_unit_x.z() = 0.0;
    vehicle_frame_unit_x = vehicle_frame_unit_x.normalized();
    const Eigen::Vector2d shifted_vehicle_position = (vehicle_position + vehicle_frame_unit_x * interface_.kos_shift).head<2>();


    //wanted_map_ = interface_.wanted_map;
    if(actual_map_ == wanted_map_){

        //only when the vehicle go enough distance, publish new path
        if(sample_flag_){
            //   find the closet point to vehicle on path (this moment)
            auto it = boost::range::min_element(
                    path_vector_, [&shifted_vehicle_position](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                        return (le - shifted_vehicle_position).squaredNorm() < (re - shifted_vehicle_position).squaredNorm();
                    });
            index_distance = std::distance(prev_pos_index_, it);             //how many points the car go through
            if (std::abs(index_distance * interface_.point_distance) < interface_.drive_distance) return;
        }else{
            //after initial or change path, publish the initial part_of_path_ first and set sample to true
            sample_flag_ = true;
            interface_.path_publisher.publish(part_of_path_);
            return;
        }

        //set new path to publish
        //clip a part of path to publish
        std::vector<Eigen::Vector2d>::iterator path_start;
        std::vector<Eigen::Vector2d>::iterator path_end;
        //reset prev pose whole index
        for(int i = 0; i < index_distance; i++){
            if (prev_pos_whole_index_ != all_path_vector_whole_[actual_map_index_].end() - 1)
                prev_pos_whole_index_++;
            else
                prev_pos_whole_index_ = all_path_vector_whole_[actual_map_index_].begin();
        }
        setCliper(prev_pos_whole_index_, all_path_vector_whole_[actual_map_index_], path_start, path_end);
        ROS_DEBUG_STREAM("pos index moving: " << index_distance << std::endl<<
                                              "current whole path index mark: " << std::distance(all_path_vector_whole_[actual_map_index_].begin(), prev_pos_whole_index_) << std::endl <<
                                              "current path start index: " << std::distance(all_path_vector_whole_[actual_map_index_].begin(), path_start) << std::endl <<
                                              "current path end index: " << std::distance(all_path_vector_whole_[actual_map_index_].begin(), path_end));
        clipPath(path_start, path_end, all_path_vector_whole_[actual_map_index_], path_vector_, part_of_path_);
        //		set new mark of prev pos index
        prev_pos_index_ = boost::range::min_element(
                path_vector_, [&shifted_vehicle_position](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                    return (le - shifted_vehicle_position).squaredNorm() < (re - shifted_vehicle_position).squaredNorm();
                });

        interface_.path_publisher.publish(part_of_path_);

        ROS_INFO_STREAM("actual map: " << actual_map_ << std::endl << "actual prev_pos_index point: " << *prev_pos_index_<< std::endl);
    }
    else{
        actual_map_ = wanted_map_;
        actual_map_index_ = actual_map_ - 1;
        initialPartOfPath(actual_map_);
        sample_flag_ = false;
    }


    ROS_INFO_STREAM("pathPublishCallback running");


}

void PathPublisher::readAllMaps(const std::string & mapPath) {
    std::vector<std::vector<Eigen::Vector2d>>::iterator iter = all_path_vector_whole_.begin();
    int count = 1;
    for(const auto& mapName: all_maps_name_){
        RoadMap map{49.01439, 8.41722};
        map.loadFromFile(mapPath + mapName + ".osm");
        double x, y;
        //save the first point
        map.getVertexMeters(1, 0, x, y);
        (*iter).emplace_back(Eigen::Vector2d(x, y));

        double accumulated_length = 0.;
        for(int i = 1; i < (int)map.trajectories.at(1).size(); i++){
            map.getVertexMeters(1, i, x, y);
            accumulated_length += ((*iter).back() - Eigen::Vector2d(x, y)).norm();
            if(accumulated_length > interface_.point_distance){
                (*iter).emplace_back(Eigen::Vector2d(x, y));
                accumulated_length = 0;
            }
        }

        ROS_INFO_STREAM("load map Nr." << count << " finished." << std::endl <<
                         "the whole path length: " << (*iter).size() << std::endl);
        iter++;
        count++;
    }
}

void PathPublisher::initialPartOfPath( const int actual_map ){

    actual_map_index_ = actual_map - 1;
    path_->header.frame_id = interface_.frame_id_map;
    path_->header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_ros;
    pose_ros.pose.orientation.x = 0.0;
    pose_ros.pose.orientation.y = 0.0;
    pose_ros.pose.orientation.z = 0.0;
    pose_ros.pose.orientation.w = 0.0;
    pose_ros.pose.position.z = 0.0;
    pose_ros.header = path_->header;

    getVehiclePose();           //save vehicle pose to vehicle_pose_

    //set prev_pos_index mark
    const Eigen::Vector3d vehicle_position = vehicle_pose_.translation();
    Eigen::Vector3d vehicle_frame_unit_x = vehicle_pose_.rotation() * Eigen::Vector3d::UnitX();
    vehicle_frame_unit_x.z() = 0.0;
    vehicle_frame_unit_x = vehicle_frame_unit_x.normalized();
    const Eigen::Vector2d pos2d = (vehicle_position + vehicle_frame_unit_x * interface_.kos_shift).head<2>();   //2d pose of the center of the car

    //find the closet point(on path) to center of vehicle
    prev_pos_whole_index_ = boost::range::min_element(
            all_path_vector_whole_[actual_map_index_], [&pos2d](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                return (le - pos2d).squaredNorm() < (re - pos2d).squaredNorm();
            });

    std::vector<Eigen::Vector2d>::iterator path_start, path_end;
    setCliper(prev_pos_whole_index_, all_path_vector_whole_[actual_map_index_], path_start, path_end);
    clipPath(path_start, path_end, all_path_vector_whole_[actual_map_index_], path_vector_, part_of_path_);             //give value to path_vector and part_of_path_

    ROS_DEBUG_STREAM("initial whole path length: " << all_path_vector_whole_[actual_map_index_].size() << std::endl <<
                                           "initial part of path length: " << path_vector_.size() << std::endl <<
                                           "initial path message length: " << part_of_path_->poses.size());

    prev_pos_index_ = boost::range::min_element(
            path_vector_, [&pos2d](const Eigen::Vector2d& le, const Eigen::Vector2d& re){
                return (le - pos2d).squaredNorm() < (re - pos2d).squaredNorm();
            });
}


void PathPublisher::getVehiclePose() {
    while(1){
        try {
            const geometry_msgs::TransformStamped tf_ros =
                    tfBuffer_.lookupTransform(interface_.frame_id_map, interface_.frame_id_vehicle, ros::Time(0));
            vehicle_pose_ = tf2::transformToEigen(tf_ros);
            break;
        } catch (const tf2::TransformException& e){
            ROS_WARN_STREAM(e.what());
        }
    }


}

double signedAngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const double vz = boost::math::sign(a.cross(b).z());
    return vz * std::acos(a.normalized().dot(b.normalized()));
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


/*******Functions relative to switch the maps (use which logic)***********/
bool PathPublisher::nearCenter() {
    getVehiclePose();
    const Eigen::Vector2d vector_position = vehicle_pose_.translation().head<2>();
    return  (vector_position - map_center_).norm() < interface_.near_center_distance;
}

int PathPublisher::nearTurnPoint() {
    getVehiclePose();
    const Eigen::Vector2d vector_position = vehicle_pose_.translation().head<2>();
    std::vector<double> distance;
    distance.push_back((vector_position - map_A_).norm());
    distance.push_back((vector_position - map_B_).norm());
    distance.push_back((vector_position - map_C_).norm());
    distance.push_back((vector_position - map_D_).norm());
    auto minIter = std::min_element(distance.begin(), distance.end());

    ROS_DEBUG_STREAM("nearst A(0) B(1) C(2) D(3) is: " << (minIter - distance.begin()) );

    return (minIter - distance.begin());
}


void PathPublisher::switchWantedMapCallback(const ros::TimerEvent& timerEvent){

    sign_ = interface_.sign;
    if(nearCenter() && sign_!="NO_SIGN" ){
        switch(nearTurnPoint()){
            case 0: //A
                if(sign_=="LEFT_SIGN"){ wanted_map_=5;}
                if(sign_=="RIGHT_SIGN"){ wanted_map_=3;}
            case 1: //B
                if(sign_=="LEFT_SIGN"){ wanted_map_=4;}
                if(sign_=="RIGHT_SIGN"){ wanted_map_=6;}
            case 2: //C
                if(sign_=="LEFT_SIGN"){ wanted_map_=7;}
                if(sign_=="RIGHT_SIGN"){ wanted_map_=3;}
            case 3: //D
                if(sign_=="LEFT_SIGN"){ wanted_map_=4;}
                if(sign_=="RIGHT_SIGN"){ wanted_map_=8;}
            default: break;
        }

    }
    if(!nearCenter() && actual_map_!=1 && actual_map_!=2){
        switch(nearTurnPoint()){
            case 0: //A
                wanted_map_=2;
            case 1: //B
                wanted_map_=2;
            case 2: //C
                wanted_map_=1;
            case 3: //D
                wanted_map_=1;
            default: break;
        }
    }
}



/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPublisher::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
