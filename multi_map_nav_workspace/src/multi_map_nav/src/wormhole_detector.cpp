#include "multi_map_nav/wormhole_detector.h"
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

WormholeDetector::WormholeDetector(ros::NodeHandle& nh) : nh_(nh), robot_in_wormhole_(false) {
  // Initialize database
  std::string db_path = nh_.param<std::string>("database_path", "wormholes.db");
  database_ = std::make_unique<multi_map_nav::WormholeDatabase>(db_path);
  
  if (!database_->initialize()) {
    ROS_ERROR("Failed to initialize wormhole database");
  } else {
    ROS_INFO("Wormhole database initialized successfully");
  }
  
  // Set up subscribers and publishers
  sub_map_  = nh_.subscribe("/multi_map_nav/map_manager/active_map", 1, &WormholeDetector::mapCb, this);
  sub_pose_ = nh_.subscribe("/amcl_pose", 1, &WormholeDetector::poseCb, this);
  pub_cross_= nh_.advertise<std_msgs::String>("wormhole_crossed", 1);
  wormhole_crossed_pub_ = nh_.advertise<std_msgs::String>("/wormhole_crossed", 1);
  
  // Set default map if not received
  current_map_ = "room_a";
  loadWormholes();
  
  ROS_INFO("WormholeDetector initialized successfully");
  ROS_INFO("Subscribing to: /amcl_pose and /multi_map_nav/map_manager/active_map");
  ROS_INFO("Publishing to: /wormhole_crossed and wormhole_crossed");
}

void WormholeDetector::loadWormholes() {
  wormholes_.clear();
  
  if (!database_ || !database_->isConnected()) {
    ROS_ERROR("Database not connected, cannot load wormholes");
    return;
  }
  
  // Load wormholes from database for current map
  auto db_wormholes = database_->getWormholesForMap(current_map_);
  ROS_WARN("=== LOADING WORMHOLES FOR MAP: %s ===", current_map_.c_str());
  ROS_INFO("Database returned %zu wormholes for map: %s", db_wormholes.size(), current_map_.c_str());
  
  for (const auto& db_wormhole : db_wormholes) {
    Wormhole w;
    w.id = db_wormhole.id;
    w.map_a = db_wormhole.map_a;
    w.map_b = db_wormhole.map_b;
    w.wkt_a = db_wormhole.wkt_a;
    w.wkt_b = db_wormhole.wkt_b;
    w.entry_x_a = db_wormhole.entry_x_a;
    w.entry_y_a = db_wormhole.entry_y_a;
    w.entry_x_b = db_wormhole.entry_x_b;
    w.entry_y_b = db_wormhole.entry_y_b;
    w.exit_x_a = db_wormhole.exit_x_a;
    w.exit_y_a = db_wormhole.exit_y_a;
    w.exit_x_b = db_wormhole.exit_x_b;
    w.exit_y_b = db_wormhole.exit_y_b;
    
    wormholes_.push_back(w);
    
    ROS_WARN("Wormhole %d: %s -> %s", w.id, w.map_a.c_str(), w.map_b.c_str());
    ROS_WARN("  Entry A: (%.2f, %.2f)", w.entry_x_a, w.entry_y_a);
    ROS_WARN("  Exit B: (%.2f, %.2f)", w.exit_x_b, w.exit_y_b);
    ROS_WARN("  WKT A: %s", w.wkt_a.c_str());
  }
  
  ROS_WARN("=== END WORMHOLE LOADING ===");
  ROS_INFO("Total loaded %zu wormholes for map: %s", wormholes_.size(), current_map_.c_str());
}

void WormholeDetector::mapCb(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Map callback received: %s", msg->data.c_str());
  current_map_ = msg->data;
  loadWormholes();
}

void WormholeDetector::poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  // Update current robot pose for crossing detection
  current_robot_pose_.header = msg->header;
  current_robot_pose_.pose = msg->pose.pose;
  
  // Debug output with coordinate analysis
  static int debug_counter = 0;
  static int pose_counter = 0;
  pose_counter++;
  
  if (debug_counter++ % 100 == 0) {
    ROS_INFO("Robot at: (%.2f, %.2f) in map: %s [msg #%d]", 
             x, y, current_map_.c_str(), pose_counter);
    ROS_INFO("Checking %zu wormholes, robot_in_wormhole: %s", 
             wormholes_.size(), robot_in_wormhole_ ? "TRUE" : "FALSE");
    
    // Show distances to all wormhole entries
    for (const auto& w : wormholes_) {
      double dist_to_entry = sqrt(pow(x - w.entry_x_a, 2) + pow(y - w.entry_y_a, 2));
      ROS_INFO("  Distance to wormhole %d entry (%.2f,%.2f): %.2f meters", 
               w.id, w.entry_x_a, w.entry_y_a, dist_to_entry);
    }
  }
  
  // Check for wormhole detection and crossing
  bool currently_in_wormhole = false;
  
  for (const auto& w : wormholes_) {
    bool in_this_wormhole = pointInWKT(x, y, w.wkt_a);
    
    if (debug_counter % 100 == 0) {
      ROS_INFO("  Wormhole %d (%s->%s): %s", w.id, w.map_a.c_str(), w.map_b.c_str(), 
               in_this_wormhole ? "INSIDE" : "outside");
    }
    
    if (in_this_wormhole) {
      currently_in_wormhole = true;
      
      // Check distance to entry point for more precise triggering
      double entry_distance = sqrt(pow(x - w.entry_x_a, 2) + pow(y - w.entry_y_a, 2));
      
      // Robot just entered wormhole region
      if (!robot_in_wormhole_) {
        ROS_WARN("=== WORMHOLE CROSSING DETECTED ===");
        ROS_WARN("Robot at (%.2f, %.2f) entering wormhole %d to map: %s", 
                 x, y, w.id, w.map_b.c_str());
        ROS_WARN("Distance to entry point: %.2f meters", entry_distance);
        
        // Publish crossing message with target map and exit pose
        std_msgs::String crossing_msg;
        crossing_msg.data = w.map_b + ";" + std::to_string(w.exit_x_b) + "," + 
                           std::to_string(w.exit_y_b) + ",0.0";
        
        ROS_WARN("Publishing crossing message: '%s'", crossing_msg.data.c_str());
        
        wormhole_crossed_pub_.publish(crossing_msg);
        pub_cross_.publish(crossing_msg);
        
        ROS_WARN("=== CROSSING MESSAGE PUBLISHED ===");
      } else {
        if (debug_counter % 50 == 0) {
          ROS_INFO("Robot still in wormhole region (already detected), distance to entry: %.2f", entry_distance);
        }
      }
      break;
    }
  }
  
  // Update state
  if (robot_in_wormhole_ != currently_in_wormhole) {
    ROS_INFO("Wormhole state changed: %s -> %s", 
             robot_in_wormhole_ ? "IN_WORMHOLE" : "OUTSIDE", 
             currently_in_wormhole ? "IN_WORMHOLE" : "OUTSIDE");
  }
  
  robot_in_wormhole_ = currently_in_wormhole;
}

bool WormholeDetector::isRobotInWormholeRegion(const geometry_msgs::PoseStamped& pose, const Wormhole& wormhole) {
  return pointInWKT(pose.pose.position.x, pose.pose.position.y, wormhole.wkt_a);
}

// Enhanced WKT parser with better error reporting
bool WormholeDetector::pointInWKT(double x, double y, const std::string& wkt) {
  try {
    if (wkt.empty()) {
      ROS_WARN("Empty WKT string");
      return false;
    }
    
    // Find the coordinate string inside the polygon
    auto start = wkt.find("((");
    auto end = wkt.find("))");
    
    if (start == std::string::npos || end == std::string::npos) {
      ROS_WARN("Invalid WKT format (missing parentheses): %s", wkt.c_str());
      return false;
    }
    
    std::string coords = wkt.substr(start + 2, end - start - 2);
    std::stringstream ss(coords);
    std::string coordinate_pair;
    
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    int point_count = 0;
    
    // Parse coordinate pairs
    while (std::getline(ss, coordinate_pair, ',')) {
      std::stringstream coord_stream(coordinate_pair);
      double px, py;
      
      if (coord_stream >> px >> py) {
        xmin = std::min(xmin, px);
        xmax = std::max(xmax, px);
        ymin = std::min(ymin, py);
        ymax = std::max(ymax, py);
        point_count++;
      } else {
        ROS_WARN("Failed to parse coordinate pair: '%s'", coordinate_pair.c_str());
      }
    }
    
    if (point_count == 0) {
      ROS_WARN("No valid coordinate pairs found in WKT: %s", wkt.c_str());
      return false;
    }
    
    // Check if point is within bounding box
    bool inside = (x >= xmin && x <= xmax && y >= ymin && y <= ymax);
    
    static int wkt_debug_counter = 0;
    if (wkt_debug_counter++ % 1000 == 0) {  // Very infrequent debug
      ROS_DEBUG("WKT check: point(%.2f,%.2f) vs box(%.2f-%.2f, %.2f-%.2f) = %s", 
                x, y, xmin, xmax, ymin, ymax, inside ? "INSIDE" : "OUTSIDE");
    }
    
    return inside;
    
  } catch (const std::exception& e) {
    ROS_ERROR("Error parsing WKT '%s': %s", wkt.c_str(), e.what());
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wormhole_detector_node");
  ros::NodeHandle nh("~");
  
  try {
    ROS_INFO("Starting wormhole detector node...");
    WormholeDetector detector(nh);
    ROS_INFO("Wormhole detector node started successfully");
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("Failed to start wormhole detector: %s", e.what());
    return -1;
  }
  
  return 0;
}
