#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

class MapManager {
public:
  MapManager(ros::NodeHandle& nh);

  /// Switches active map by name (calls map_server unload/load)
  void switchMap(const std::string& next_map);

private:
  void publishActive();

  ros::Publisher pub_active_;
  ros::ServiceClient srv_load_, srv_unload_;
  std::string active_map_;
};
