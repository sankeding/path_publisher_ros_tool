#include "path_planning.hpp"

namespace path_publisher_ros_tool {

PathPlanning::PathPlanning(ros::NodeHandle nhPublic, ros::NodeHandle nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate},tfListener_{tfBuffer_} {

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


    subscriber_ = nhPrivate.subscribe(interface_.map_topic,
                                      interface_.map_queue_size,
                                      &PathPlanning::callbackSubscriber,
                                      this);

    publisher_ = nhPrivate.advertise<nav_msgs::Path>(interface_.path_topic,
                                                     interface_.path_queue_size);
    update_timer_ = nhPrivate.createTimer(ros::Rate(interface_.update_path_rate),
                                          &PathPlanning::updatePath,
                                          this);
    publish_timer_ = nhPrivate.createTimer(ros::Rate(interface_.publish_path_rate),
                                           &PathPlanning::publishPath,
                                           this);

    std::string data_path;
    nhPrivate.getParam("exit_map_path", data_path);
    initializePath(data_path);

    Eigen::Vector3d path_end((path_.poses.end() - 1) -> pose.position.x,
                             (path_.poses.end() - 1) -> pose.position.y,
                             0.0);
    Eigen::Vector3d path_direction = path_end - Eigen::Vector3d((path_.poses.end() - 2) -> pose.position.x,
                                                                (path_.poses.end() - 2) -> pose.position.y,
                                                                0.0);
    theta_ = atan2(cross2d(Eigen::Vector3d::UnitX(), path_direction),          //tan(theta)=|a|*|b|*sin(theta)/(|a|*|b|*cos(theta))
                   path_direction.dot(Eigen::Vector3d::UnitX()));



    rosinterface_handler::showNodeInfo();
}

void PathPlanning::callbackSubscriber(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = *msg;
}

void PathPlanning::publishPath(const ros::TimerEvent& event) {
    //if (!activate_) { return; }
    publisher_.publish(path_);
}

void PathPlanning::initializePath(const std::string& store_path) {

    RoadMap map{49.01439, 8.41722};
    map.loadFromFile(store_path + "11_exit.osm");

    path_.header.frame_id = interface_.map_frame_id;

    geometry_msgs::PoseStamped pose;
    pose.header = path_.header;
    double x, y;
    for(int i = 0; i < (int)map.trajectories.at(1).size(); i++){
        map.getVertexMeters(1, i, x, y);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path_.poses.emplace_back(pose);
    }

}


void PathPlanning::updatePath(const ros::TimerEvent& event) {
   // if (!activate_) { return; }

    Eigen::Affine3d vehicle_pose;
    while(1){
        try {
            const geometry_msgs::TransformStamped tf_ros =
                    tfBuffer_.lookupTransform(interface_.map_frame_id, interface_.vehicle_frame_id, ros::Time(0));
            vehicle_pose = tf2::transformToEigen(tf_ros);
            break;
        } catch (const tf2::TransformException& e){
            ROS_WARN_STREAM(e.what());
        }
    }
    Eigen::Vector3d vehicle_position = vehicle_pose.translation();
    Eigen::Vector3d path_end((path_.poses.end() - 1) -> pose.position.x,
                             (path_.poses.end() - 1) -> pose.position.y,
                             0.0);
    if ( (vehicle_position - path_end).norm() > interface_.planning_margin ) {
        ROS_INFO_STREAM_THROTTLE(1, "Not start planning yet.");
        return;
    }

    std::vector<Eigen::Vector3d> obstacle_points;
    for (size_t i = 0; i < map_.info.height; i++)
        for (size_t j = 0; j < map_.info.width; j++) {
            int32_t grid_value = map_.data[i * map_.info.width + j];
            if (grid_value < interface_.probability_threshold) { continue; }

            double x{-1}, y{-1};
            x = (j + 0.5) * map_.info.resolution + map_.info.origin.position.x;
            y = (i + 0.5) * map_.info.resolution + map_.info.origin.position.y;
            obstacle_points.emplace_back(x, y, 0.0);
        }

    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(theta_, Eigen::Vector3d::UnitZ());

    geometry_msgs::PoseStamped new_path_point;
    new_path_point.header = path_ .header;
    new_path_point.pose.position.x = -1.0;
    new_path_point.pose.position.y = -1.0;

    double min_potential{DBL_MAX}, lat{-interface_.search_width / 2};
    while (lat <= interface_.search_width / 2) {
        double potential{0.0};

        potential += 0.1 * fabs(lat);

        Eigen::Vector3d search_point = rotation * Eigen::Vector3d(interface_.search_interval, lat, 0.0) + path_end;
        for (auto& point : obstacle_points) {
            potential += getPotential((search_point - point).norm());
        }

        if (potential < min_potential) {
            new_path_point.pose.position.x = search_point[0];
            new_path_point.pose.position.y = search_point[1];
            min_potential = potential;
        }

        lat += interface_.search_density;
    }

    Eigen::Vector3d new_direction = Eigen::Vector3d(new_path_point.pose.position.x,
                                                    new_path_point.pose.position.y,
                                                    0.0) - path_end;
    double new_theta_ = atan2(cross2d(Eigen::Vector3d::UnitX(), new_direction),
                              new_direction.dot(Eigen::Vector3d::UnitX()));
    theta_ = theta_ * interface_.momentum + new_theta_ * (1 - interface_.momentum);
    path_.poses.emplace_back(new_path_point);

    ROS_INFO_STREAM("new_path_point, x: "<< new_path_point.pose.position.x <<" y: "<< new_path_point.pose.position.y << std::endl << std::endl);
}


double PathPlanning::getPotential(const double distance) {
    if (distance > interface_.effect_range) {
        return 0.0;
    }

    return (5 - distance) * (5 - distance);
}

double PathPlanning::cross2d(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    return a[0] * b[1] - a[1] * b[0];
}

/**
  * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window
*/
void PathPlanning::reconfigureRequest(const Interface::Config& config, uint32_t level) {
    interface_.fromConfig(config);
}


} // namespace path_publisher_ros_tool
