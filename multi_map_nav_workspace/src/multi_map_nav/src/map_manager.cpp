#include "multi_map_nav/map_manager.h"

MapManager::MapManager(ros::NodeHandle& nh) {
  nh.param("map_manager/default_map", active_map_, std::string("room_a"));
  pub_active_ = nh.advertise<std_msgs::String>("map_manager/active_map", 1, true);
  srv_unload_ = nh.serviceClient<std_srvs::Empty>("map_server/unload_map");
  srv_load_   = nh.serviceClient<std_srvs::Empty>("map_server/load_map");

  // publish initial map
  publishActive();
}

void MapManager::switchMap(const std::string& next_map) {
  if (next_map == active_map_) return;

  std_srvs::Empty srv;
  srv_unload_.call(srv);

  // set param for map_server to load
  ros::param::set("/map_server/map", next_map);
  srv_load_.call(srv);

  active_map_ = next_map;
  publishActive();
}

void MapManager::publishActive() {
  std_msgs::String msg;
  msg.data = active_map_;
  pub_active_.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "map_manager_node");
  ros::NodeHandle nh("~");
  MapManager mgr(nh);
  ros::spin();
  return 0;
}
