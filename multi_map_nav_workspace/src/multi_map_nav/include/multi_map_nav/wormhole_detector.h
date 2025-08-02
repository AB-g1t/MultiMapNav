#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <vector>
#include <memory>
#include "multi_map_nav/wormhole_database.h"

/**
 * @brief Structure to represent a wormhole
 */
struct Wormhole {
  std::string map_a, map_b, wkt_a, wkt_b;
  double entry_x_a, entry_y_a, entry_x_b, entry_y_b;
  double exit_x_a, exit_y_a, exit_x_b, exit_y_b;
  int id;
};

/**
 * @brief Wormhole Detector for multi-map navigation
 */
class WormholeDetector {
public:
  WormholeDetector(ros::NodeHandle& nh);

private:
  void loadWormholes();
  void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void mapCb(const std_msgs::String::ConstPtr& msg);
  bool pointInWKT(double x, double y, const std::string& wkt);
  bool isRobotInWormholeRegion(const geometry_msgs::PoseStamped& pose, const Wormhole& wormhole);

  ros::NodeHandle nh_;
  ros::Subscriber sub_pose_, sub_map_;
  ros::Publisher pub_cross_;
  ros::Publisher wormhole_crossed_pub_;
  
  std::string current_map_, target_map_;
  std::vector<Wormhole> wormholes_;
  std::unique_ptr<multi_map_nav::WormholeDatabase> database_;
  
  bool robot_in_wormhole_;
  geometry_msgs::PoseStamped current_robot_pose_;
};
