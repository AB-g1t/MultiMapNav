#include "multi_map_nav/wormhole_detector.h"
#include <sstream>

WormholeDetector::WormholeDetector(ros::NodeHandle& nh) {
  // Initialize database
  std::string db_path = nh.param<std::string>("database_path", "wormholes.db");
  database_ = std::make_unique<multi_map_nav::WormholeDatabase>(db_path);
  
  if (!database_->initialize()) {
    ROS_ERROR("Failed to initialize wormhole database");
  }
  
  sub_map_  = nh.subscribe("/multi_map_nav/map_manager/active_map", 1, &WormholeDetector::mapCb, this);
  sub_pose_ = nh.subscribe("/amcl_pose", 1, &WormholeDetector::poseCb, this);
  pub_cross_= nh.advertise<std_msgs::String>("wormhole_crossed", 1);
  
  // Set default map if not received
  current_map_ = "room_a";
  loadWormholes();
}

void WormholeDetector::loadWormholes() {
  wormholes_.clear();
  
  if (!database_ || !database_->isConnected()) {
    ROS_WARN("Database not connected, cannot load wormholes");
    return;
  }
  
  // Load wormholes from database for current map
  auto db_wormholes = database_->getWormholesForMap(current_map_);
  
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
  }
  
  ROS_INFO("Loaded %zu wormholes for map: %s", wormholes_.size(), current_map_.c_str());
}

void WormholeDetector::mapCb(const std_msgs::String::ConstPtr& msg) {
  current_map_ = msg->data;
  loadWormholes();
}

void WormholeDetector::poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  
  // Debug output
  static int debug_counter = 0;
  if (debug_counter++ % 10 == 0) {  // Print every 10th message
    ROS_INFO("Robot at (%.2f, %.2f) in map: %s", x, y, current_map_.c_str());
  }
  
  for (const auto& w : wormholes_) {
    if (pointInWKT(x,y,w.wkt_a)) {
      ROS_INFO("WORMHOLE DETECTED! Robot at (%.2f, %.2f) in wormhole region", x, y);
      std_msgs::String e;
      e.data = w.map_b + ";" + w.wkt_b;
      pub_cross_.publish(e);
      break;
    }
  }
}

// VERY simplified WKT parser just for rectangles:
// e.g. "POLYGON((x1 y1,x2 y2,...))"
bool WormholeDetector::pointInWKT(double x, double y, const std::string& wkt) {
  // parse min/max quickly
  std::vector<std::pair<double,double>> pts;
  auto inner = wkt.substr(wkt.find("((")+2, wkt.find("))") - wkt.find("((")-2);
  std::stringstream ss(inner);
  std::string tok;
  double xmin=1e9, xmax=-1e9, ymin=1e9, ymax=-1e9;
  while (std::getline(ss,tok,',')) {
    std::stringstream pp(tok);
    double px,py; pp>>px>>py;
    xmin = std::min(xmin,px); xmax = std::max(xmax,px);
    ymin = std::min(ymin,py); ymax = std::max(ymax,py);
  }
  return x>=xmin && x<=xmax && y>=ymin && y<=ymax;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "wormhole_detector_node");
  ros::NodeHandle nh("~");
  WormholeDetector det(nh);
  ros::spin();
  return 0;
}
