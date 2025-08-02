#pragma once
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_map_nav/MultiMapNavAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>
#include "multi_map_nav/wormhole_database.h"

class MultiMapNavServer {
public:
  MultiMapNavServer(ros::NodeHandle& nh);

private:
  void executeCB(const multi_map_nav::MultiMapNavGoalConstPtr& goal);
  void crossCb(const std_msgs::String::ConstPtr& msg);
  void mapCb(const std_msgs::String::ConstPtr& msg);
  void publishInitial(const geometry_msgs::PoseStamped& p);
  void publishFeedback(const std::string& s);
  
  // Enhanced wormhole passing functionality
  bool waitForActiveMap();
  bool performWormholeNavigation(const std::string& target_map);
  bool findWormholeToTargetMap(const std::string& target_map);
  void setupWormholePoses(const multi_map_nav::WormholeEntry& wormhole, bool reverse);
  bool waitForWormholeCrossing();
  bool performMapSwitch(const std::string& new_map);
  bool sendMoveWithRetry(const geometry_msgs::PoseStamped& target);
  bool sendMove(const geometry_msgs::PoseStamped& p);  // Changed to return bool
  void publishInitialPoseWithCovariance(const geometry_msgs::PoseStamped& p);
  bool parseExitPoseFromString(const std::string& pose_str, geometry_msgs::PoseStamped& exit_pose);

  actionlib::SimpleActionServer<multi_map_nav::MultiMapNavAction> as_;
  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> mb_client_;
  ros::Subscriber sub_cross_, sub_map_;
  ros::Publisher pub_map_switch_, pub_initialpose_;
  std::string active_map_, pending_map_;
  geometry_msgs::PoseStamped entry_pose_, exit_pose_, final_pose_;
  bool crossed_;
  std::unique_ptr<multi_map_nav::WormholeDatabase> database_;
};
