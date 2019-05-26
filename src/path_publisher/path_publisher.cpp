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

    set_path_subscriber_ = nhPrivate.subscribe("/set_path_topic",
    								        1,
    								        &PathPublisher::setPath_Callback,
    								        this);                                           //

    reconfigureServer_.setCallback(boost::bind(&PathPublisher::reconfigureRequest, this, _1, _2));

//  initial path_
    readAllMaps(interface_.path_to_map);

    rosinterface_handler::showNodeInfo();
}


void PathPublisher::setPath_Callback(const std_msgs::Int8::ConstPtr& msg){
	set_path_ = msg->data;
	ROS_DEBUG_STREAM("received set_path info:"<< set_path_ <<std::endl);
}

void PathPublisher::readAllMaps(const std::string & mapPath) {
    std::vector<std::vector<Eigen::Vector2d>>::iterator iter = all_path_vecotr_whole_.begin();
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





/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPublisher::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
