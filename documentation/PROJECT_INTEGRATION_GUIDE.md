# Multi-Map Navigation System Integration Guide

## 🚀 Quick Start

### Option 1: Run with Web Dashboard (Recommended)
```bash
# Start the complete system with web interface
./run_simulation_with_dashboard.sh
```

### Option 2: Run 6-Room Simulation
```bash
# Start the 6-room navigation demonstration
./run_6_room_simulation.sh
```

### Option 3: Basic Demonstration
```bash
# Run the basic multi-map navigation demo
./run_demonstration.sh
```

## 📋 System Components

### Core Navigation System
- **Multi-Map Navigation**: Navigate between 6 different rooms
- **Wormhole Detection**: Automatic transition between maps
- **SQL Database**: Persistent storage of wormholes and trajectories
- **Action Server**: Handle navigation goals with target map and position

### Web Dashboard Features
- **Real-time Monitoring**: Live position and status updates
- **Room Visualization**: Interactive room selection and navigation
- **Trajectory Tracking**: Visual path history
- **Wormhole Events**: Monitor map transitions
- **Navigation History**: Complete movement log

### 6-Room Environment
- **Living Room** (room_a): Central hub
- **Kitchen** (room_b): Connected to living room
- **Bedroom** (room_c): Upper level
- **Bathroom** (room_d): Upper level
- **Office** (room_e): Central hub connecting all rooms
- **Garage** (room_f): Remote location

## 🎯 How to Use in Your Project

### 1. Basic Navigation Commands

#### Send Navigation Goal via Python
```python
#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from multi_map_nav.msg import MultiMapNavAction, MultiMapNavGoal

def send_navigation_goal(target_map, x, y, yaw=0.0):
    """Send a navigation goal to the multi-map system"""
    client = actionlib.SimpleActionClient('multi_map_nav', MultiMapNavAction)
    client.wait_for_server()
    
    goal = MultiMapNavGoal()
    goal.target_map = target_map
    
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    
    # Convert yaw to quaternion
    import math
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
    
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

# Example usage
rospy.init_node('my_navigation_client')
send_navigation_goal("room_b", 8.0, 1.0, 1.57)  # Go to kitchen
```

#### Send Navigation Goal via Command Line
```bash
# Navigate to kitchen
python3 send_goal.py room_b 8.0 1.0 90

# Navigate to bedroom
python3 send_goal.py room_c 1.0 8.0 -90

# Navigate to office
python3 send_goal.py room_e 4.0 4.0 0
```

### 2. Web Dashboard Integration

#### Access the Dashboard
1. Start the system: `./run_simulation_with_dashboard.sh`
2. Open browser: `http://localhost:5000`
3. Use the interactive interface to:
   - Select target rooms
   - Monitor robot position
   - View navigation history
   - Track wormhole events

#### Dashboard API Endpoints
```python
# Get current system status
GET /api/data

# Get room information
GET /api/rooms

# Get navigation history
GET /api/history
```

### 3. Database Integration

#### View Wormholes
```bash
# View all wormholes in database
python3 view_database.py

# Or use SQLite directly
sqlite3 wormholes.db "SELECT * FROM wormholes;"
```

#### Add Custom Wormholes
```python
from multi_map_nav.wormhole_database import WormholeDatabase

db = WormholeDatabase("wormholes.db")
db.initialize()

# Add a new wormhole
wormhole = WormholeEntry(
    map_a="room_a",
    map_b="room_b", 
    wkt_a="POLYGON((5 0, 6 0, 6 1, 5 1, 5 0))",
    wkt_b="POLYGON((0 0, 1 0, 1 1, 0 1, 0 0))",
    entry_x_a=5.5, entry_y_a=0.5,
    entry_x_b=0.5, entry_y_b=0.5,
    exit_x_a=5.5, exit_y_a=0.5,
    exit_x_b=0.5, exit_y_b=0.5
)
db.addWormhole(wormhole)
```

### 4. Custom Room Configuration

#### Modify Room Layout
Edit `setup_6_rooms.py` to change:
- Room positions and connections
- Wormhole locations
- Room names and descriptions

#### Add New Rooms
1. Add room to database in `setup_6_rooms.py`
2. Create corresponding map files
3. Update wormhole connections
4. Rebuild system: `python3 setup_6_rooms.py`

### 5. Trajectory Analysis

#### View Trajectories
```bash
# View trajectory data
sqlite3 trajectories.db "SELECT * FROM trajectories;"

# Analyze movement patterns
python3 analyze_trajectories.py
```

#### Custom Trajectory Analysis
```python
from multi_map_nav.trajectory_manager import TrajectoryManager

tm = TrajectoryManager("trajectories.db")
trajectories = tm.getAllTrajectories()

for traj in trajectories:
    print(f"Trajectory {traj.id}: {traj.start_map} -> {traj.end_map}")
    print(f"Distance: {traj.total_distance:.2f}m")
    print(f"Duration: {traj.duration:.2f}s")
```

## 🔧 Customization Options

### 1. Modify Navigation Behavior
Edit `src/multi_map_nav/src/multimap_nav_server.cpp`:
- Change navigation parameters
- Add custom navigation logic
- Modify wormhole transition behavior

### 2. Custom Wormhole Detection
Edit `src/multi_map_nav/src/wormhole_detector.cpp`:
- Modify detection algorithms
- Add custom transition conditions
- Implement different wormhole types

### 3. Enhanced Visualization
Edit `src/multi_map_nav/rviz/multimap_demo.rviz`:
- Add custom markers
- Modify map display
- Add trajectory visualization

### 4. Web Dashboard Customization
Edit `web_dashboard.py`:
- Add new dashboard features
- Customize room layouts
- Add new API endpoints

## 📊 Monitoring and Debugging

### System Status Commands
```bash
# Check if all nodes are running
rosnode list

# Monitor navigation topics
rostopic echo /multi_map_nav/feedback
rostopic echo /wormhole_crossed
rostopic echo /map_manager/active_map

# View robot position
rostopic echo /amcl_pose

# Check navigation status
rostopic echo /move_base/status
```

### Debug Tools
```bash
# Test system components
python3 test_components.py

# Verify system readiness
python3 verify_ready.py

# View database contents
python3 view_database.py
```

## 🎯 Integration Examples

### Example 1: Automated Room Patrol
```python
#!/usr/bin/env python3
import rospy
import time
from multi_map_nav.msg import MultiMapNavAction, MultiMapNavGoal
import actionlib

def patrol_rooms():
    """Automated patrol through all rooms"""
    client = actionlib.SimpleActionClient('multi_map_nav', MultiMapNavAction)
    client.wait_for_server()
    
    rooms = [
        ("room_a", 0, 0),      # Living Room
        ("room_b", 8, 1),      # Kitchen  
        ("room_c", 1, 8),      # Bedroom
        ("room_d", 8, 8),      # Bathroom
        ("room_e", 4, 4),      # Office
        ("room_f", 14, 4),     # Garage
    ]
    
    for room, x, y in rooms:
        goal = MultiMapNavGoal()
        goal.target_map = room
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        print(f"Navigating to {room}...")
        client.send_goal(goal)
        client.wait_for_result()
        time.sleep(2)  # Wait at each room

if __name__ == "__main__":
    rospy.init_node('room_patrol')
    patrol_rooms()
```

### Example 2: Smart Room Selection
```python
def smart_navigation():
    """Navigate based on room availability and priority"""
    # Define room priorities and availability
    room_priorities = {
        "room_b": 1,  # Kitchen - highest priority
        "room_d": 2,  # Bathroom
        "room_c": 3,  # Bedroom
        "room_e": 4,  # Office
        "room_f": 5,  # Garage
        "room_a": 6   # Living Room - lowest priority
    }
    
    # Check which rooms need attention
    rooms_to_visit = ["room_b", "room_d"]  # Example
    
    for room in sorted(rooms_to_visit, key=lambda x: room_priorities[x]):
        send_navigation_goal(room, 0, 0)
```

## 🚀 Advanced Features

### 1. Multi-Robot Support
- Extend the system to handle multiple robots
- Implement robot coordination
- Add collision avoidance between robots

### 2. Dynamic Obstacle Avoidance
- Integrate with dynamic obstacle detection
- Implement real-time path replanning
- Add obstacle prediction

### 3. Machine Learning Integration
- Use trajectory data for path optimization
- Implement predictive navigation
- Add learning-based room selection

### 4. IoT Integration
- Connect with smart home devices
- Implement sensor-based navigation
- Add environmental awareness

## 📝 Troubleshooting

### Common Issues
1. **Action server not available**: Wait for system initialization
2. **Database errors**: Run `python3 setup_demo.py`
3. **Map loading issues**: Check map file paths
4. **Navigation failures**: Verify robot localization

### Performance Optimization
1. **Reduce map switching time**: Optimize map loading
2. **Improve navigation speed**: Adjust move_base parameters
3. **Enhance visualization**: Use GPU-accelerated rendering

---

**Your multi-map navigation system is now ready for integration!** 🎉 