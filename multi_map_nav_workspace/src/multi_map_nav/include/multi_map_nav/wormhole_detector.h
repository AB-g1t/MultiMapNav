#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
 * 
 * This class monitors the robot's position and detects when it enters
 * wormhole regions, triggering map switches for seamless navigation
 * between different mapped areas.
 */
class WormholeDetector {
public:
  /**
   * @brief Constructor
   * @param nh ROS node handle
   */
  WormholeDetector(ros::NodeHandle& nh);

private:
  /**
   * @brief Load wormholes from database for current map
   */
  void loadWormholes();
  
  /**
   * @brief Callback for pose updates
   * @param msg Pose message
   */
  void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  
  /**
   * @brief Callback for map updates
   * @param msg Map name message
   */
  void mapCb(const std_msgs::String::ConstPtr& msg);
  
  /**
   * @brief Check if a point is inside a WKT polygon
   * @param x X coordinate
   * @param y Y coordinate
   * @param wkt Well-Known Text polygon representation
   * @return true if point is inside polygon
   */
  bool pointInWKT(double x, double y, const std::string& wkt);

  ros::Subscriber sub_pose_, sub_map_;
  ros::Publisher pub_cross_;
  std::string current_map_, target_map_;
  std::vector<Wormhole> wormholes_;
  std::unique_ptr<multi_map_nav::WormholeDatabase> database_;
};
