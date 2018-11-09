#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "path_publisher.hpp"

namespace path_publisher_ros_tool {

class PathPublisherNodelet : public nodelet::Nodelet {

    inline void onInit() override {
        impl_ = std::make_unique<PathPublisher>(getNodeHandle(), getPrivateNodeHandle());
    }
    std::unique_ptr<PathPublisher> impl_;
};
} // namespace path_publisher_ros_tool

PLUGINLIB_EXPORT_CLASS(path_publisher_ros_tool::PathPublisherNodelet, nodelet::Nodelet);
