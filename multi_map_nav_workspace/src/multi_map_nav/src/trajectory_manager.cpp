#include "multi_map_nav/trajectory_manager.h"
#include <ros/console.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sstream>
#include <cmath>

namespace multi_map_nav {

TrajectoryManager::TrajectoryManager(ros::NodeHandle& nh, const std::string& db_path)
    : nh_(nh), db_(nullptr), db_path_(db_path), connected_(false), recording_(false) {
    
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize publishers and subscribers
    pose_sub_ = nh_.subscribe("/amcl_pose", 1, &TrajectoryManager::poseCallback, this);
    map_sub_ = nh_.subscribe("/map_manager/active_map", 1, &TrajectoryManager::mapCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);
    
    ROS_INFO("TrajectoryManager initialized with database path: %s", db_path_.c_str());
}

TrajectoryManager::~TrajectoryManager() {
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool TrajectoryManager::initialize() {
    if (!initializeDatabase()) {
        ROS_ERROR("Failed to initialize trajectory database");
        return false;
    }
    
    ROS_INFO("TrajectoryManager initialized successfully");
    return true;
}

int TrajectoryManager::startTrajectory(const std::string& name, const std::string& start_map) {
    if (recording_) {
        ROS_WARN("Already recording a trajectory. Stop current trajectory first.");
        return -1;
    }
    
    // Create new trajectory entry in database
    std::stringstream ss;
    ss << "INSERT INTO trajectories (name, start_map, start_time) VALUES ('"
       << name << "', '" << start_map << "', datetime('now'))";
    
    if (!executeQuery(ss.str())) {
        ROS_ERROR("Failed to create trajectory entry in database");
        return -1;
    }
    
    // Get the ID of the newly created trajectory
    sqlite3_stmt* stmt;
    const std::string query = "SELECT last_insert_rowid()";
    int rc = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to get trajectory ID");
        return -1;
    }
    
    int trajectory_id = -1;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        trajectory_id = sqlite3_column_int(stmt, 0);
    }
    sqlite3_finalize(stmt);
    
    if (trajectory_id == -1) {
        ROS_ERROR("Failed to get trajectory ID from database");
        return -1;
    }
    
    // Initialize current trajectory
    current_trajectory_.id = trajectory_id;
    current_trajectory_.name = name;
    current_trajectory_.start_map = start_map;
    current_trajectory_.start_time = ros::Time::now();
    current_trajectory_.points.clear();
    current_trajectory_.completed = false;
    
    recording_ = true;
    ROS_INFO("Started recording trajectory '%s' with ID %d", name.c_str(), trajectory_id);
    
    return trajectory_id;
}

bool TrajectoryManager::stopTrajectory(const std::string& end_map) {
    if (!recording_) {
        ROS_WARN("No trajectory is currently being recorded");
        return false;
    }
    
    current_trajectory_.end_map = end_map;
    current_trajectory_.end_time = ros::Time::now();
    current_trajectory_.completed = true;
    
    // Save trajectory points to database
    if (!saveTrajectoryPoints(current_trajectory_.id, current_trajectory_.points)) {
        ROS_ERROR("Failed to save trajectory points to database");
        return false;
    }
    
    // Update trajectory metadata
    std::stringstream ss;
    ss << "UPDATE trajectories SET end_map = '" << end_map 
       << "', end_time = datetime('now'), completed = 1 WHERE id = " 
       << current_trajectory_.id;
    
    if (!executeQuery(ss.str())) {
        ROS_ERROR("Failed to update trajectory metadata");
        return false;
    }
    
    recording_ = false;
    ROS_INFO("Stopped recording trajectory '%s' with ID %d", 
             current_trajectory_.name.c_str(), current_trajectory_.id);
    
    return true;
}

void TrajectoryManager::addPosePoint(const geometry_msgs::PoseWithCovarianceStamped& pose, 
                                    const std::string& map_name) {
    if (!recording_) {
        return;
    }
    
    TrajectoryPoint point = poseToTrajectoryPoint(pose, map_name);
    current_trajectory_.points.push_back(point);
    
    // Publish visualization
    publishTrajectoryVisualization();
}

void TrajectoryManager::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    addPosePoint(*msg, current_map_);
}

void TrajectoryManager::mapCallback(const std_msgs::String::ConstPtr& msg) {
    current_map_ = msg->data;
    
    if (recording_) {
        // Record map transition
        ROS_INFO("Map transition detected: %s", current_map_.c_str());
    }
}

void TrajectoryManager::publishTrajectoryVisualization(int trajectory_id) {
    // Publish path message
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    
    std::vector<TrajectoryPoint> points;
    if (trajectory_id == -1) {
        // Use current trajectory
        points = current_trajectory_.points;
    } else {
        // Load trajectory from database
        points = loadTrajectoryPoints(trajectory_id);
    }
    
    for (const auto& point : points) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = point.timestamp;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = point.z;
        
        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, point.yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        path_msg.poses.push_back(pose);
    }
    
    path_pub_.publish(path_msg);
    
    // Publish markers for trajectory points
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    
    for (const auto& point : points) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory_points";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = point.z;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        
        marker_array.markers.push_back(marker);
    }
    
    marker_pub_.publish(marker_array);
}

std::map<std::string, double> TrajectoryManager::calculateTrajectoryStatistics(int trajectory_id) {
    std::map<std::string, double> stats;
    
    std::vector<TrajectoryPoint> points = loadTrajectoryPoints(trajectory_id);
    if (points.empty()) {
        return stats;
    }
    
    // Calculate total distance
    double total_distance = calculateTotalDistance(points);
    stats["distance"] = total_distance;
    
    // Calculate duration
    double duration = (points.back().timestamp - points.front().timestamp).toSec();
    stats["duration"] = duration;
    
    // Calculate average speed
    if (duration > 0) {
        stats["average_speed"] = total_distance / duration;
    } else {
        stats["average_speed"] = 0.0;
    }
    
    // Count map transitions
    int map_transitions = 0;
    std::string prev_map = points.front().map_name;
    for (const auto& point : points) {
        if (point.map_name != prev_map) {
            map_transitions++;
            prev_map = point.map_name;
        }
    }
    stats["map_transitions"] = map_transitions;
    
    return stats;
}

bool TrajectoryManager::initializeDatabase() {
    int rc = sqlite3_open(db_path_.c_str(), &db_);
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to open trajectory database: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    connected_ = true;
    return createTables();
}

bool TrajectoryManager::createTables() {
    const std::string create_trajectories_sql = R"(
        CREATE TABLE IF NOT EXISTS trajectories (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            start_map TEXT NOT NULL,
            end_map TEXT,
            start_time DATETIME NOT NULL,
            end_time DATETIME,
            completed BOOLEAN DEFAULT FALSE,
            created_at DATETIME DEFAULT CURRENT_TIMESTAMP
        )
    )";
    
    const std::string create_points_sql = R"(
        CREATE TABLE IF NOT EXISTS trajectory_points (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            trajectory_id INTEGER NOT NULL,
            x REAL NOT NULL,
            y REAL NOT NULL,
            z REAL NOT NULL,
            yaw REAL NOT NULL,
            map_name TEXT NOT NULL,
            timestamp DATETIME NOT NULL,
            FOREIGN KEY (trajectory_id) REFERENCES trajectories(id)
        )
    )";
    
    if (!executeQuery(create_trajectories_sql) || !executeQuery(create_points_sql)) {
        ROS_ERROR("Failed to create trajectory tables");
        return false;
    }
    
    ROS_INFO("Trajectory tables created successfully");
    return true;
}

bool TrajectoryManager::executeQuery(const std::string& sql) {
    char* err_msg = nullptr;
    int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err_msg);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    return true;
}

bool TrajectoryManager::saveTrajectoryPoints(int trajectory_id, const std::vector<TrajectoryPoint>& points) {
    for (const auto& point : points) {
        std::stringstream ss;
        ss << "INSERT INTO trajectory_points (trajectory_id, x, y, z, yaw, map_name, timestamp) "
           << "VALUES (" << trajectory_id << ", " << point.x << ", " << point.y << ", " << point.z 
           << ", " << point.yaw << ", '" << point.map_name << "', datetime('now'))";
        
        if (!executeQuery(ss.str())) {
            return false;
        }
    }
    
    return true;
}

std::vector<TrajectoryPoint> TrajectoryManager::loadTrajectoryPoints(int trajectory_id) {
    std::vector<TrajectoryPoint> points;
    
    std::string query = "SELECT x, y, z, yaw, map_name, timestamp FROM trajectory_points "
                       "WHERE trajectory_id = " + std::to_string(trajectory_id) + 
                       " ORDER BY timestamp";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to prepare query: %s", sqlite3_errmsg(db_));
        return points;
    }
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        TrajectoryPoint point;
        point.x = sqlite3_column_double(stmt, 0);
        point.y = sqlite3_column_double(stmt, 1);
        point.z = sqlite3_column_double(stmt, 2);
        point.yaw = sqlite3_column_double(stmt, 3);
        point.map_name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
        // Note: timestamp conversion would need more complex handling
        point.timestamp = ros::Time::now(); // Simplified for now
        
        points.push_back(point);
    }
    
    sqlite3_finalize(stmt);
    return points;
}

TrajectoryPoint TrajectoryManager::poseToTrajectoryPoint(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                                        const std::string& map_name) {
    TrajectoryPoint point;
    point.x = pose.pose.pose.position.x;
    point.y = pose.pose.pose.position.y;
    point.z = pose.pose.pose.position.z;
    point.map_name = map_name;
    point.timestamp = pose.header.stamp;
    
    // Convert quaternion to yaw
    tf2::Quaternion q(pose.pose.pose.orientation.x,
                     pose.pose.pose.orientation.y,
                     pose.pose.pose.orientation.z,
                     pose.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    point.yaw = yaw;
    
    return point;
}

double TrajectoryManager::calculateDistance(const TrajectoryPoint& p1, const TrajectoryPoint& p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double TrajectoryManager::calculateTotalDistance(const std::vector<TrajectoryPoint>& points) {
    if (points.size() < 2) {
        return 0.0;
    }
    
    double total_distance = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        total_distance += calculateDistance(points[i-1], points[i]);
    }
    
    return total_distance;
}

// Additional methods for getting trajectories (simplified implementations)
std::unique_ptr<Trajectory> TrajectoryManager::getTrajectory(int id) {
    // Simplified implementation - would need full database query
    return nullptr;
}

std::vector<Trajectory> TrajectoryManager::getAllTrajectories() {
    // Simplified implementation - would need full database query
    return std::vector<Trajectory>();
}

std::vector<Trajectory> TrajectoryManager::getTrajectoriesForMap(const std::string& map_name) {
    // Simplified implementation - would need full database query
    return std::vector<Trajectory>();
}

bool TrajectoryManager::deleteTrajectory(int id) {
    // Simplified implementation - would need full database query
    return false;
}

} // namespace multi_map_nav

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_manager_node");
    ros::NodeHandle nh("~");
    
    std::string db_path;
    nh.param<std::string>("database_path", db_path, "trajectories.db");
    
    multi_map_nav::TrajectoryManager trajectory_manager(nh, db_path);
    
    if (!trajectory_manager.initialize()) {
        ROS_ERROR("Failed to initialize trajectory manager");
        return 1;
    }
    
    // Start recording a trajectory
    int trajectory_id = trajectory_manager.startTrajectory("test_trajectory", "room_a");
    if (trajectory_id != -1) {
        ROS_INFO("Started recording trajectory with ID: %d", trajectory_id);
    }
    
    ros::spin();
    
    // Stop recording when node is shutting down
    trajectory_manager.stopTrajectory("room_a");
    
    return 0;
} 