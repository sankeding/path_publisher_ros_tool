#include "path_publisher.hpp"

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

    map_.loadFromFile(interface_.path_to_map + interface_.map_name);

    timer_ = nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &PathPublisher::callbackTimer, this);

    rosinterface_handler::showNodeInfo();
}

void PathPublisher::callbackTimer(const ros::TimerEvent& timer_event) {

//    Msg::Ptr newMsg = boost::make_shared<Msg>(*msg);
//    interface_.path_publisher.publish(newMsg);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPublisher::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
