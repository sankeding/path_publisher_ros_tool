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
    std::string map_path = "../res/pathes";
    readAllMaps(map_path);

    rosinterface_handler::showNodeInfo();
}


void PathPublisher::setPath_Callback(const std_msgs::Int8::ConstPtr& msg){
	set_path_ = msg->data;
	ROS_DEBUG_STREAM("received set_path info:"<< set_path_ <<std::endl);
}

void PathPublisher::readAllMaps(const std::string & mapPath) {
    for (const auto& path: all_maps_name_){
        

    }

}





/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPublisher::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
