#include "path_planning.hpp"

namespace path_publisher_ros_tool {

PathPlanning::PathPlanning(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    /**
     * Initialization
     */
    interface_.fromParamServer();

    /**
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/PathPlanning.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&PathPlanning::reconfigureRequest, this, _1, _2));
    interface_.dummy_subscriber->registerCallback(&PathPlanning::callbackSubscriber, this);

    rosinterface_handler::showNodeInfo();
}

void PathPlanning::callbackSubscriber(const Msg::ConstPtr& msg) {

    // do your stuff here...
    Msg::Ptr newMsg = boost::make_shared<Msg>(*msg);
    interface_.dummy_publisher.publish(newMsg);
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPlanning::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
