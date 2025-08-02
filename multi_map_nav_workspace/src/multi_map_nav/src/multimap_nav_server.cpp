#include "multi_map_nav/multimap_nav_server.h"
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

MultiMapNavServer::MultiMapNavServer(ros::NodeHandle& nh)
: as_(nh, "multi_map_nav", boost::bind(&MultiMapNavServer::executeCB, this, _1), false),
  mb_client_(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true)),
  crossed_(false)
{
  pub_map_switch_   = nh.advertise<std_msgs::String>("map_manager/active_map", 1);
  pub_initialpose_  = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  sub_cross_        = nh.subscribe("wormhole_crossed", 1, &MultiMapNavServer::crossCb, this);
  sub_map_          = nh.subscribe("map_manager/active_map", 1, &MultiMapNavServer::mapCb, this);

  // Initialize database for wormhole lookups
  std::string db_path = nh.param<std::string>("database_path", "wormholes.db");
  database_ = std::make_unique<multi_map_nav::WormholeDatabase>(db_path);
  if (!database_->initialize()) {
    ROS_ERROR("Failed to initialize wormhole database");
  }

  as_.start();
  ROS_INFO("MultiMapNavServer initialized and ready for goals");
}

void MultiMapNavServer::mapCb(const std_msgs::String::ConstPtr& msg) {
  active_map_ = msg->data;
  ROS_DEBUG("Active map updated to: %s", active_map_.c_str());
}

void MultiMapNavServer::crossCb(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Received wormhole crossing signal: %s", msg->data.c_str());

  // Parse format: "target_map;exit_x,exit_y,exit_yaw" or just "target_map"
  auto sep = msg->data.find(';');
  pending_map_ = msg->data.substr(0, sep);

  exit_pose_ = entry_pose_;  // fallback if specific pose info is not given
  exit_pose_.header.stamp = ros::Time::now();

  if (sep != std::string::npos && sep + 1 < msg->data.size()) {
    std::string pose_info = msg->data.substr(sep + 1);
    if (parseExitPoseFromString(pose_info, exit_pose_)) {
      ROS_INFO("Parsed exit pose: (%.2f, %.2f, %.2f)",
               exit_pose_.pose.position.x, exit_pose_.pose.position.y,
               tf2::getYaw(exit_pose_.pose.orientation));
    } else {
      ROS_WARN("Failed to parse exit pose, using default.");
    }
  }

  crossed_ = true;
}

bool MultiMapNavServer::parseExitPoseFromString(const std::string& pose_str,
                                               geometry_msgs::PoseStamped& exit_pose) {
  std::stringstream ss(pose_str);
  std::string token;
  std::vector<double> values;

  while (std::getline(ss, token, ',')) {
    try {
      values.push_back(std::stod(token));
    } catch (...) {
      return false;
    }
  }

  if (values.size() >= 2) {
    exit_pose.pose.position.x = values[0];
    exit_pose.pose.position.y = values[1];
    exit_pose.pose.position.z = 0.0;

    if (values.size() >= 3) {
      tf2::Quaternion q;
      q.setRPY(0, 0, values[2]);
      exit_pose.pose.orientation = tf2::toMsg(q);
    } else {
      exit_pose.pose.orientation.x = 0;
      exit_pose.pose.orientation.y = 0;
      exit_pose.pose.orientation.z = 0;
      exit_pose.pose.orientation.w = 1;
    }

    return true;
  }

  return false;
}


void MultiMapNavServer::executeCB(const multi_map_nav::MultiMapNavGoalConstPtr& goal) {
  publishFeedback("Goal received");
  final_pose_ = goal->target_pose;

  // Wait for active map info before proceeding
  if (!waitForActiveMap()) {
    ROS_ERROR("Active map unknown and timeout reached");
    as_.setAborted();
    return;
  }

  ROS_INFO("=== Multi-Map Navigation Goal ===");
  ROS_INFO("Current map: %s", active_map_.c_str());
  ROS_INFO("Target map: %s", goal->target_map.c_str());
  ROS_INFO("Target pose: (%.2f, %.2f)", goal->target_pose.pose.position.x, 
           goal->target_pose.pose.position.y);

  // Same map navigation
  if (goal->target_map == active_map_) {
    ROS_INFO("Executing intra-map navigation within '%s'", active_map_.c_str());
    publishFeedback("Navigating within current map");
    if (sendMoveWithRetry(final_pose_)) {
      publishFeedback("Navigation succeeded");
      as_.setSucceeded();
    } else {
      ROS_ERROR("Navigation failed");
      as_.setAborted();
    }
    return;
  }

  // Inter-map navigation requires wormhole
  ROS_INFO("Executing inter-map navigation: %s -> %s", active_map_.c_str(), goal->target_map.c_str());
  
  // Check database connection
  if (!database_ || !database_->isConnected()) {
    ROS_ERROR("Database not connected, cannot find wormhole");
    as_.setAborted();
    return;
  }
  
  // Find wormhole to target map and set up poses
  if (!findWormholeToTargetMap(goal->target_map)) {
    ROS_ERROR("No wormhole found from %s to %s", active_map_.c_str(), goal->target_map.c_str());
    as_.setAborted();
    return;
  }

  // Execute wormhole-based navigation
  if (!performWormholeNavigation(goal->target_map)) {
    ROS_ERROR("Inter-map navigation failed");
    as_.setAborted();
    return;
  }

  publishFeedback("Multi-map navigation succeeded");
  as_.setSucceeded();
}

bool MultiMapNavServer::waitForActiveMap() {
  if (!active_map_.empty()) return true;

  ros::Rate rate(10);
  int count = 0, timeout = 50;  // 5 sec timeout

  while (active_map_.empty() && ros::ok() && count < timeout) {
    ros::spinOnce();
    rate.sleep();
    ++count;
  }

  if (active_map_.empty()) {
    ROS_WARN("Active map not received, defaulting to 'room_a'");
    active_map_ = "room_a";
  }

  return true;
}

bool MultiMapNavServer::performWormholeNavigation(const std::string& target_map) {
  if (!findWormholeToTargetMap(target_map)) return false;

  publishFeedback("Navigating to wormhole entry point");
  if (!sendMoveWithRetry(entry_pose_)) return false;

  publishFeedback("Waiting for wormhole crossing detection");
  if (!waitForWormholeCrossing()) return false;

  publishFeedback("Switching map");
  if (!performMapSwitch(pending_map_)) return false;

  publishFeedback("Navigating to final target");
  if (!sendMoveWithRetry(final_pose_)) return false;

  return true;
}

bool MultiMapNavServer::findWormholeToTargetMap(const std::string& target_map) {
  if (!database_ || !database_->isConnected()) {
    ROS_ERROR("Wormhole database not connected");
    return false;
  }

  auto wormholes = database_->getWormholesForMap(active_map_);
  for (const auto& wormhole : wormholes) {
    if (wormhole.map_b == target_map) {
      setupWormholePoses(wormhole, false);
      return true;
    }
    // Add bidirectional check if supported by your database
  }

  ROS_ERROR("No wormhole from %s to %s found", active_map_.c_str(), target_map.c_str());
  return false;
}

// In MultiMapNavServer::setupWormholePoses()
void MultiMapNavServer::setupWormholePoses(const multi_map_nav::WormholeEntry& wormhole, bool reverse) {
  entry_pose_.header.frame_id = "map";
  entry_pose_.header.stamp = ros::Time::now();
  exit_pose_.header.frame_id = "map";
  exit_pose_.header.stamp = ros::Time::now();

  if (!reverse) {
    // Use approach point instead of exact entry coordinates
    entry_pose_.pose.position.x = wormhole.entry_x_a - 2.0; // 2m before entry
    entry_pose_.pose.position.y = wormhole.entry_y_a;
    exit_pose_.pose.position.x = wormhole.exit_x_b;
    exit_pose_.pose.position.y = wormhole.exit_y_b;
  } else {
    entry_pose_.pose.position.x = wormhole.entry_x_b - 2.0; // 2m before entry
    entry_pose_.pose.position.y = wormhole.entry_y_b;
    exit_pose_.pose.position.x = wormhole.exit_x_a;
    exit_pose_.pose.position.y = wormhole.exit_y_a;
  }

  entry_pose_.pose.position.z = 0.0;
  exit_pose_.pose.position.z = 0.0;
  entry_pose_.pose.orientation.w = 1.0;
  exit_pose_.pose.orientation.w = 1.0;
  
  ROS_WARN("=== WORMHOLE POSES SET ===");
  ROS_WARN("Entry pose: (%.2f, %.2f)", entry_pose_.pose.position.x, entry_pose_.pose.position.y);
  ROS_WARN("Exit pose: (%.2f, %.2f)", exit_pose_.pose.position.x, exit_pose_.pose.position.y);
  ROS_WARN("========================");
}


bool MultiMapNavServer::waitForWormholeCrossing() {
  ros::Rate rate(10);
  int count = 0, limit = 300;  // ~30 seconds timeout
  crossed_ = false;

  while (ros::ok() && !crossed_ && count < limit) {
    ros::spinOnce();
    rate.sleep();
    ++count;

    // Provide periodic feedback
    if (count % 50 == 0) {
      ROS_INFO("Still waiting for wormhole crossing... (%d/%d)", count, limit);
    }
  }

  if (!crossed_) {
    ROS_ERROR("Wormhole crossing timeout");
    return false;
  }

  crossed_ = false;
  return true;
}

bool MultiMapNavServer::performMapSwitch(const std::string& new_map) {
  std_msgs::String msg;
  msg.data = new_map;
  pub_map_switch_.publish(msg);
  ros::Duration(2.0).sleep();

  active_map_ = new_map;
  publishInitial(exit_pose_);
  ros::Duration(1.0).sleep();

  ROS_INFO("Switched to map: %s", new_map.c_str());
  return true;
}

bool MultiMapNavServer::sendMoveWithRetry(const geometry_msgs::PoseStamped& target) {
  const int max_retries = 3;

  for (int attempt = 1; attempt <= max_retries; ++attempt) {
    if (sendMove(target)) return true;

    if (attempt < max_retries) {
      ROS_WARN("Retrying navigation (%d/%d)", attempt, max_retries);
      ros::Duration(2.0).sleep();
    }
  }

  return false;
}

bool MultiMapNavServer::sendMove(const geometry_msgs::PoseStamped& p) {
  if (!mb_client_->waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("move_base action server not available");
    return false;
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = p;
  goal.target_pose.header.stamp = ros::Time::now();

  mb_client_->sendGoal(goal);
  
  bool finished = mb_client_->waitForResult(ros::Duration(120.0));
  if (!finished) {
    mb_client_->cancelGoal();
    return false;
  }

  return mb_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void MultiMapNavServer::publishInitial(const geometry_msgs::PoseStamped& p) {
  geometry_msgs::PoseWithCovarianceStamped ip;
  ip.header = p.header;
  ip.header.stamp = ros::Time::now();
  ip.pose.pose = p.pose;

  // Reasonable covariance for AMCL
  for (int i = 0; i < 36; i++) ip.pose.covariance[i] = 0.0;
  ip.pose.covariance[0] = 0.25;   // x variance
  ip.pose.covariance[7] = 0.25;   // y variance  
  ip.pose.covariance[35] = 0.25;  // yaw variance

  pub_initialpose_.publish(ip);
}

void MultiMapNavServer::publishFeedback(const std::string& s) {
  ROS_INFO("MultiMapNav feedback: %s", s.c_str());
  // If your action supports detailed feedback, publish here
}

// ADD THE MISSING MAIN FUNCTION HERE:
int main(int argc, char** argv) {
  ros::init(argc, argv, "multimap_nav_server");
  ros::NodeHandle nh("~");
  
  try {
    MultiMapNavServer server(nh);
    ROS_INFO("Multi-map navigation server started successfully");
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("Failed to start multi-map navigation server: %s", e.what());
    return -1;
  }
  
  return 0;
}
