#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sqlite3.h>
#include <vector>
#include <string>
#include <memory>

namespace multi_map_nav {

/**
 * @brief Structure to represent a trajectory point
 */
struct TrajectoryPoint {
    double x, y, z;
    double yaw;
    std::string map_name;
    ros::Time timestamp;
};

/**
 * @brief Structure to represent a complete trajectory
 */
struct Trajectory {
    int id;
    std::string name;
    std::string start_map;
    std::string end_map;
    std::vector<TrajectoryPoint> points;
    ros::Time start_time;
    ros::Time end_time;
    bool completed;
};

/**
 * @brief Trajectory Manager for collecting, storing, and visualizing robot trajectories
 * 
 * This class provides functionality to:
 * - Collect trajectory data from robot pose updates
 * - Store trajectories in an SQLite database
 * - Visualize trajectories in RViz
 * - Analyze trajectory performance and statistics
 */
class TrajectoryManager {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param db_path Path to the SQLite database file
     */
    TrajectoryManager(ros::NodeHandle& nh, const std::string& db_path);
    
    /**
     * @brief Destructor
     */
    ~TrajectoryManager();
    
    /**
     * @brief Initialize the trajectory manager
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start recording a new trajectory
     * @param name Name of the trajectory
     * @param start_map Starting map name
     * @return Trajectory ID if successful, -1 otherwise
     */
    int startTrajectory(const std::string& name, const std::string& start_map);
    
    /**
     * @brief Stop recording the current trajectory
     * @param end_map Ending map name
     * @return true if successful, false otherwise
     */
    bool stopTrajectory(const std::string& end_map);
    
    /**
     * @brief Add a pose point to the current trajectory
     * @param pose Current robot pose
     * @param map_name Current map name
     */
    void addPosePoint(const geometry_msgs::PoseWithCovarianceStamped& pose, 
                     const std::string& map_name);
    
    /**
     * @brief Get a trajectory by ID
     * @param id Trajectory ID
     * @return Trajectory object if found, nullptr otherwise
     */
    std::unique_ptr<Trajectory> getTrajectory(int id);
    
    /**
     * @brief Get all trajectories
     * @return Vector of all trajectories
     */
    std::vector<Trajectory> getAllTrajectories();
    
    /**
     * @brief Get trajectories for a specific map
     * @param map_name Map name
     * @return Vector of trajectories
     */
    std::vector<Trajectory> getTrajectoriesForMap(const std::string& map_name);
    
    /**
     * @brief Delete a trajectory
     * @param id Trajectory ID
     * @return true if successful, false otherwise
     */
    bool deleteTrajectory(int id);
    
    /**
     * @brief Publish trajectory visualization
     * @param trajectory_id ID of trajectory to visualize (-1 for current)
     */
    void publishTrajectoryVisualization(int trajectory_id = -1);
    
    /**
     * @brief Calculate trajectory statistics
     * @param trajectory_id ID of trajectory to analyze
     * @return Map of statistics (distance, duration, etc.)
     */
    std::map<std::string, double> calculateTrajectoryStatistics(int trajectory_id);

private:
    ros::NodeHandle& nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    sqlite3* db_;
    std::string db_path_;
    bool connected_;
    
    Trajectory current_trajectory_;
    bool recording_;
    std::string current_map_;
    
    /**
     * @brief Callback for pose updates
     * @param msg Pose message
     */
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    
    /**
     * @brief Callback for map updates
     * @param msg Map name message
     */
    void mapCallback(const std_msgs::String::ConstPtr& msg);
    
    /**
     * @brief Initialize the database and create tables
     * @return true if successful, false otherwise
     */
    bool initializeDatabase();
    
    /**
     * @brief Create the trajectories table
     * @return true if successful, false otherwise
     */
    bool createTables();
    
    /**
     * @brief Execute a SQL query
     * @param sql SQL query string
     * @return true if successful, false otherwise
     */
    bool executeQuery(const std::string& sql);
    
    /**
     * @brief Save trajectory points to database
     * @param trajectory_id Trajectory ID
     * @param points Vector of trajectory points
     * @return true if successful, false otherwise
     */
    bool saveTrajectoryPoints(int trajectory_id, const std::vector<TrajectoryPoint>& points);
    
    /**
     * @brief Load trajectory points from database
     * @param trajectory_id Trajectory ID
     * @return Vector of trajectory points
     */
    std::vector<TrajectoryPoint> loadTrajectoryPoints(int trajectory_id);
    
    /**
     * @brief Convert pose to trajectory point
     * @param pose Robot pose
     * @param map_name Map name
     * @return Trajectory point
     */
    TrajectoryPoint poseToTrajectoryPoint(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                        const std::string& map_name);
    
    /**
     * @brief Calculate distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Distance in meters
     */
    double calculateDistance(const TrajectoryPoint& p1, const TrajectoryPoint& p2);
    
    /**
     * @brief Calculate total trajectory distance
     * @param points Vector of trajectory points
     * @return Total distance in meters
     */
    double calculateTotalDistance(const std::vector<TrajectoryPoint>& points);
};

} // namespace multi_map_nav 