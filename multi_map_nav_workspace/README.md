# Multi-Map Navigation System - ROS Workspace

This is a complete ROS workspace containing the Multi-Map Navigation System.

## 🚀 Quick Start

1. **Build the system:**
   ```bash
   cd multi_map_nav_workspace
   catkin_make
   source devel/setup.bash
   ```

2. **Run the action server:**
   ```bash
   roslaunch multi_map_nav 6_room_simulation.launch
   ```

3. **Send navigation goals:**
   ```bash
   # Example: Navigate from room A to room B
   rostopic pub /multimap_nav/goal multi_map_nav/MultiMapNavActionGoal
   ```

## 📁 Workspace Structure

```
multi_map_nav_workspace/
├── src/
│   └── multi_map_nav/          # Main package
│       ├── src/                 # C++ source files
│       ├── include/             # Header files
│       ├── action/              # Action definitions
│       ├── launch/              # Launch files
│       ├── maps/                # Room maps
│       ├── config/              # Configuration files
│       ├── rviz/                # RViz configurations
│       ├── CMakeLists.txt       # Build configuration
│       ├── package.xml          # Package manifest
│       ├── wormholes.db         # Wormhole database
│       └── trajectories.db      # Trajectory database
└── documentation/               # Project documentation
```

## 🔧 System Requirements

- ROS Noetic
- SQLite3
- Catkin build system
- Gazebo simulator (for simulation mode)

## 📝 License

This project is part of the AR100 drone navigation system.

---
Generated on: 2025-08-01 22:06:11
