# Multi-Map Navigation and Wormhole Implementation

## Overview

This ROS package implements a sophisticated multi-map navigation system that enables robots to seamlessly navigate between different mapped rooms using a "wormhole" mechanism. The system includes SQL database integration for persistent storage, trajectory collection and visualization, and a modular architecture designed for easy integration and future enhancements.

## Features

- **Multi-Map Navigation**: Navigate between different rooms with automatic map switching
- **Wormhole Detection**: Automatic detection of transition regions between maps
- **SQL Database Integration**: Persistent storage of wormhole positions and trajectory data
- **Trajectory Collection**: Record and analyze robot movement patterns
- **Visualization**: Real-time trajectory visualization in RViz
- **Modular Architecture**: Object-oriented design with separate header and source files
- **Action Server**: ROS action server for handling navigation goals
- **Configuration Management**: Parameter-based configuration without hardcoded paths

## System Architecture

### Core Components

1. **WormholeDetector**: Monitors robot position and detects wormhole entry
2. **MultiMapNavServer**: Action server for handling navigation goals
3. **MapManager**: Manages map switching and loading
4. **WormholeDatabase**: SQL database interface for wormhole storage
5. **TrajectoryManager**: Collects, stores, and visualizes robot trajectories

### Data Flow

```
Navigation Goal → MultiMapNavServer → MapManager → WormholeDetector → Database
                     ↓
              TrajectoryManager → Visualization
```

## Algorithms and Pseudocode

### 1. Wormhole Detection Algorithm

```pseudocode
Algorithm: Wormhole Detection
Input: Robot pose (x, y), current map name
Output: Wormhole activation event

1. Load wormholes for current map from database
2. For each wormhole in current map:
   a. Parse WKT polygon for region A
   b. Check if robot position (x, y) is inside polygon
   c. If inside:
      - Publish wormhole_crossed message
      - Include target map and exit region information
      - Trigger map switching process

Function: pointInWKT(x, y, wkt)
1. Extract polygon coordinates from WKT string
2. Calculate bounding box (min/max x, y)
3. Quick rejection: if point outside bounding box, return false
4. Ray casting algorithm for point-in-polygon test
5. Return true if point is inside polygon
```

### 2. Multi-Map Navigation Algorithm

```pseudocode
Algorithm: Multi-Map Navigation
Input: Navigation goal (target_map, target_pose)
Output: Navigation completion status

1. Check if target map equals current map
   a. If same map: Send direct move_base goal
   b. If different map: Proceed to step 2

2. Find wormhole entry point for target map
   a. Query database for wormholes from current to target map
   b. Select appropriate wormhole based on proximity
   c. Navigate to wormhole entry point

3. Wait for wormhole crossing detection
   a. Monitor wormhole_crossed topic
   b. Extract target map and exit region information

4. Switch maps
   a. Publish map switch command
   b. Wait for map loading completion
   c. Initialize robot pose in new map

5. Navigate to final target
   a. Send move_base goal to target pose
   b. Monitor navigation completion

6. Return success/failure status
```

### 3. Trajectory Collection Algorithm

```pseudocode
Algorithm: Trajectory Collection
Input: Robot pose updates, map changes
Output: Stored trajectory data

1. Start trajectory recording
   a. Create new trajectory entry in database
   b. Initialize trajectory points array
   c. Set recording flag to true

2. For each pose update:
   a. Convert pose to trajectory point
   b. Add timestamp and map information
   c. Store point in memory buffer
   d. Publish visualization markers

3. On map change:
   a. Record map transition point
   b. Update trajectory metadata

4. Stop trajectory recording
   a. Save all points to database
   b. Calculate trajectory statistics
   c. Mark trajectory as completed

Function: calculateTrajectoryStatistics(trajectory_id)
1. Load trajectory points from database
2. Calculate total distance (sum of point-to-point distances)
3. Calculate duration (end_time - start_time)
4. Calculate average speed (distance / duration)
5. Identify map transitions
6. Return statistics map
```

## Database Schema

### Wormholes Table
```sql
CREATE TABLE wormholes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    map_a TEXT NOT NULL,
    map_b TEXT NOT NULL,
    wkt_a TEXT NOT NULL,
    wkt_b TEXT NOT NULL,
    entry_x_a REAL NOT NULL,
    entry_y_a REAL NOT NULL,
    entry_x_b REAL NOT NULL,
    entry_y_b REAL NOT NULL,
    exit_x_a REAL NOT NULL,
    exit_y_a REAL NOT NULL,
    exit_x_b REAL NOT NULL,
    exit_y_b REAL NOT NULL,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### Trajectories Table
```sql
CREATE TABLE trajectories (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    start_map TEXT NOT NULL,
    end_map TEXT NOT NULL,
    start_time DATETIME NOT NULL,
    end_time DATETIME,
    completed BOOLEAN DEFAULT FALSE,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);
```

### Trajectory Points Table
```sql
CREATE TABLE trajectory_points (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    trajectory_id INTEGER NOT NULL,
    x REAL NOT NULL,
    y REAL NOT NULL,
    z REAL NOT NULL,
    yaw REAL NOT NULL,
    map_name TEXT NOT NULL,
    timestamp DATETIME NOT NULL,
    FOREIGN KEY (trajectory_id) REFERENCES trajectories(id)
);
```

## Installation and Setup

### Prerequisites
- ROS Noetic
- SQLite3 development libraries
- YAML-CPP
- TF2

### Installation
```bash
# Install dependencies
sudo apt-get install libsqlite3-dev libyaml-cpp-dev

# Clone the repository
cd ~/catkin_ws/src
git clone <repository-url>

# Build the package
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Configuration
1. Create wormhole regions in your maps
2. Add wormhole entries to the database
3. Configure map paths in the launch file
4. Set default map parameter

## Usage

### Starting the System
```bash
# Launch the complete multi-map navigation system
roslaunch multi_map_nav multimap_demo.launch
```

### Sending Navigation Goals

#### Using Python Script
```bash
# Send goal to room_a at position (5, 3)
python3 send_goal.py
```

#### Using Action Client
```python
import actionlib
from multi_map_nav.msg import MultiMapNavAction, MultiMapNavGoal

client = actionlib.SimpleActionClient('multi_map_nav', MultiMapNavAction)
goal = MultiMapNavGoal()
goal.target_map = "room_b"
goal.target_pose.pose.position.x = 2.0
goal.target_pose.pose.position.y = 1.0
client.send_goal(goal)
```

#### Using RViz
1. Add "2D Nav Goal" tool in RViz
2. Click and drag to set goal pose
3. Robot will navigate automatically

### Database Management

#### Adding Wormholes
```python
from multi_map_nav.wormhole_database import WormholeDatabase

db = WormholeDatabase("/path/to/database.db")
db.initialize()

entry = WormholeEntry()
entry.map_a = "room_a"
entry.map_b = "room_b"
entry.wkt_a = "POLYGON((0 0, 1 0, 1 1, 0 1))"
entry.wkt_b = "POLYGON((0 0, 1 0, 1 1, 0 1))"
entry.entry_x_a = 0.5
entry.entry_y_a = 0.5
# ... set other coordinates

db.addWormhole(entry)
```

#### Viewing Trajectories
```python
from multi_map_nav.trajectory_manager import TrajectoryManager

tm = TrajectoryManager(nh, "/path/to/database.db")
trajectories = tm.getAllTrajectories()
for traj in trajectories:
    print(f"Trajectory {traj.id}: {traj.name}")
    stats = tm.calculateTrajectoryStatistics(traj.id)
    print(f"Distance: {stats['distance']:.2f}m")
    print(f"Duration: {stats['duration']:.2f}s")
```

## Configuration Parameters

### Node Parameters
- `map_manager/default_map`: Default map name (default: "room_a")
- `config/wormholes`: Database path for wormholes
- `config/trajectories`: Database path for trajectories

### Topics
- `/amcl_pose`: Robot pose (input)
- `/map_manager/active_map`: Current map name (input/output)
- `/wormhole_crossed`: Wormhole detection events (output)
- `/multi_map_nav/goal`: Navigation goals (input)
- `/multi_map_nav/result`: Navigation results (output)
- `/trajectory_path`: Trajectory visualization (output)

## Testing and Validation

### Unit Tests
```bash
# Run unit tests
catkin_make run_tests_multi_map_nav
```

### Integration Tests
1. Start the system with test maps
2. Send navigation goals between different maps
3. Verify wormhole detection and map switching
4. Check trajectory recording and visualization

### Performance Metrics
- Navigation success rate
- Map switching time
- Trajectory accuracy
- Database query performance

## Troubleshooting

### Common Issues

1. **Wormhole not detected**
   - Check WKT polygon format
   - Verify robot is within wormhole region
   - Check database connectivity

2. **Map switching fails**
   - Ensure map files exist
   - Check map_server configuration
   - Verify topic connections

3. **Database errors**
   - Check file permissions
   - Verify SQLite installation
   - Check database path

### Debug Information
```bash
# Enable debug logging
export ROSCONSOLE_MIN_SEVERITY=DEBUG

# View database contents
sqlite3 /path/to/database.db "SELECT * FROM wormholes;"
```

## Future Enhancements

1. **Dynamic Wormhole Creation**: GUI for creating wormholes interactively
2. **Advanced Path Planning**: Multi-map path planning algorithms
3. **Machine Learning**: Predictive wormhole usage patterns
4. **Cloud Integration**: Remote database and trajectory sharing
5. **3D Navigation**: Support for multi-floor environments

## Contributing

1. Fork the repository
2. Create a feature branch
3. Follow coding guidelines
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS community for the excellent framework
- Anscer Robotics for the AR100 platform
- Contributors and testers

## Contact

For questions and support, please contact the development team or create an issue in the repository. 