#include "multi_map_nav/multimap_nav_server.h"
#include <sstream>

MultiMapNavServer::MultiMapNavServer(ros::NodeHandle& nh)
: as_(nh, "multi_map_nav", boost::bind(&MultiMapNavServer::executeCB, this, _1), false),
  mb_client_(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base",true)),
  crossed_(false)
{
  pub_map_switch_   = nh.advertise<std_msgs::String>("map_manager/active_map",1);
  pub_initialpose_  = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);
  sub_cross_        = nh.subscribe("wormhole_crossed",1,&MultiMapNavServer::crossCb,this);
  sub_map_          = nh.subscribe("map_manager/active_map",1,&MultiMapNavServer::mapCb,this);
  
  // Initialize database for wormhole lookups
  std::string db_path = nh.param<std::string>("database_path", "wormholes.db");
  database_ = std::make_unique<multi_map_nav::WormholeDatabase>(db_path);
  if (!database_->initialize()) {
    ROS_ERROR("Failed to initialize wormhole database");
  }
  
  as_.start();
}

void MultiMapNavServer::mapCb(const std_msgs::String::ConstPtr& m) {
  active_map_ = m->data;
}

void MultiMapNavServer::crossCb(const std_msgs::String::ConstPtr& m) {
  // payload: "room_b;POLYGON((...))"
  auto sep = m->data.find(';');
  pending_map_ = m->data.substr(0,sep);
  // parse exit polygon same as detector
  // here for simplicity set exit_pose_ = entry_pose_
  crossed_ = true;
}

void MultiMapNavServer::executeCB(const multi_map_nav::MultiMapNavGoalConstPtr& goal) {
  publishFeedback("Goal received");
  final_pose_ = goal->target_pose;

  // Wait for active map if it's empty
  if (active_map_.empty()) {
    publishFeedback("Waiting for active map...");
    ros::Rate r(10);
    int timeout = 0;
    while (active_map_.empty() && timeout < 50) {  // 5 second timeout
      r.sleep();
      timeout++;
    }
    if (active_map_.empty()) {
      ROS_ERROR("Timeout waiting for active map, using default 'room_a'");
      active_map_ = "room_a";
    }
  }

  // same map?
  if (goal->target_map == active_map_) {
    sendMove(final_pose_);
    as_.setSucceeded();
    return;
  }

  // find wormhole entry from detector's loaded list
  // Get wormhole entry pose from database
  if (!database_ || !database_->isConnected()) {
    ROS_ERROR("Database not connected, cannot find wormhole");
    as_.setAborted();
    return;
  }
  
  auto wormholes = database_->getWormholesForMap(active_map_);
  bool found_wormhole = false;
  
  for (const auto& wormhole : wormholes) {
    if (wormhole.map_b == goal->target_map) {
      // Found the wormhole to target map
      entry_pose_.header.frame_id = "map";
      entry_pose_.header.stamp = ros::Time::now();
      entry_pose_.pose.position.x = wormhole.entry_x_a;
      entry_pose_.pose.position.y = wormhole.entry_y_a;
      entry_pose_.pose.position.z = 0.0;
      entry_pose_.pose.orientation.x = 0.0;
      entry_pose_.pose.orientation.y = 0.0;
      entry_pose_.pose.orientation.z = 0.0;
      entry_pose_.pose.orientation.w = 1.0;
      
      // Set exit pose for the target map
      exit_pose_.header.frame_id = "map";
      exit_pose_.header.stamp = ros::Time::now();
      exit_pose_.pose.position.x = wormhole.exit_x_b;
      exit_pose_.pose.position.y = wormhole.exit_y_b;
      exit_pose_.pose.position.z = 0.0;
      exit_pose_.pose.orientation.x = 0.0;
      exit_pose_.pose.orientation.y = 0.0;
      exit_pose_.pose.orientation.z = 0.0;
      exit_pose_.pose.orientation.w = 1.0;
      
      found_wormhole = true;
      break;
    }
  }
  
  if (!found_wormhole) {
    ROS_ERROR("No wormhole found from %s to %s", active_map_.c_str(), goal->target_map.c_str());
    as_.setAborted();
    return;
  }

  publishFeedback("Heading to wormhole entry");
  sendMove(entry_pose_);

  // wait crossing
  ros::Rate r(10);
  while (ros::ok() && !crossed_) r.sleep();
  crossed_ = false;

  publishFeedback("Switching map");
  std_msgs::String cmd; cmd.data = pending_map_;
  pub_map_switch_.publish(cmd);
  ros::Duration(1.0).sleep();  // allow map load

  publishInitial(exit_pose_);
  publishFeedback("Heading to final target");
  sendMove(final_pose_);

  as_.setSucceeded();
}

void MultiMapNavServer::sendMove(const geometry_msgs::PoseStamped& p) {
  move_base_msgs::MoveBaseGoal m;
  m.target_pose = p;
  mb_client_->waitForServer();
  mb_client_->sendGoal(m);
  mb_client_->waitForResult();
}

void MultiMapNavServer::publishInitial(const geometry_msgs::PoseStamped& p) {
  geometry_msgs::PoseWithCovarianceStamped ip;
  ip.header = p.header;
  ip.pose.pose = p.pose;
  // zero covariance for simplicity
  pub_initialpose_.publish(ip);
}

void MultiMapNavServer::publishFeedback(const std::string& s) {
  // Note: The generated feedback message only has a 'success' field, not 'status'
  // We'll use a different approach to provide feedback
  ROS_INFO("MultiMapNav Feedback: %s", s.c_str());
  // TODO: Regenerate action messages to include status field in feedback
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "multimap_nav_server");
  ros::NodeHandle nh("~");
  MultiMapNavServer server(nh);
  ros::spin();
  return 0;
}
