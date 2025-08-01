# Multi-Map Navigation System - Deliverables

This package contains all deliverables for the Multi-Map Navigation System implementation.

## 📁 Directory Structure

### 🚀 Ready-to-Build ROS Workspace (`multi_map_nav_workspace/`)
- **Complete ROS workspace** that can be built with `catkin_make`
- **Proper package structure** with all source files, headers, and configurations
- **Ready to run** - just build and launch!

### 📚 Documentation (`documentation/`)
- `README.md`: Main project documentation
- `6_ROOM_GUIDE.md`: Guide for 6-room simulation
- `6_ROOM_SIMULATION_GUIDE.md`: Simulation setup and running
- `DASHBOARD_GUIDE.md`: Web dashboard documentation
- `DEMONSTRATION_GUIDE.md`: Demo instructions
- `DEMONSTRATION_SUMMARY.md`: Demo summary
- `PROJECT_INTEGRATION_GUIDE.md`: Integration guide

## 🚀 Quick Start (READY TO BUILD!)

1. **Extract the zip file**
2. **Build the system:**
   ```bash
   export PYTHONNOUSERSITE=1
   cd multi_map_nav_workspace
   catkin_make
   source devel/setup.bash
   ```

3. **Run the simulation:**

   **Option A: 6-Room House Simulation**
   ```bash
   roslaunch multi_map_nav 6_room_simulation.launch
   ```

   **Option B: Distributed Warehouse Automation**
   ```bash
   roslaunch multi_map_nav warehouse_distributed.launch
   ```

4. **Send navigation goals:**

   **For 6-Room House:**
   ```bash
   # Navigate to room_b at position (1, 1)
   rostopic pub /multi_map_nav/multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal "header: {frame_id: ''} goal_id: {id: ''} goal: {target_map: 'room_b', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   ```

   **For Distributed Warehouse:**
   ```bash
   # Navigate from Receiving to Storage 1
   rostopic pub /multi_map_nav/multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal "header: {frame_id: ''} goal_id: {id: ''} goal: {target_map: 'room_b', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   
   # Navigate from Receiving to Processing
   rostopic pub /multi_map_nav/multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal "header: {frame_id: ''} goal_id: {id: ''} goal: {target_map: 'room_c', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   
   # Navigate from Receiving to Shipping (long distance via wormhole)
   rostopic pub /multi_map_nav/multi_map_nav/goal multi_map_nav/MultiMapNavActionGoal "header: {frame_id: ''} goal_id: {id: ''} goal: {target_map: 'room_e', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
   ```

   **Available rooms:** room_a, room_b, room_c, room_d, room_e, room_f
   
   **Warehouse rooms:**
   - **room_a**: Receiving Area (green dock)
   - **room_b**: Storage Area 1 (brown racks)
   - **room_c**: Processing Area (blue stations)
   - **room_d**: Storage Area 2 (cyan cold storage)
   - **room_e**: Shipping Area (red dock + conveyor)
   - **room_f**: Quality Control (yellow stations)
   
   **Monitor progress:**
   ```bash
   rostopic echo /multi_map_nav/multi_map_nav/feedback
   ```

## 📊 What's Included

### ✅ Mapped Rooms
- Separate maps for each room (room_a through room_f)
- Each room has a `.pgm` image file and `.yaml` configuration
- Wormhole regions are defined in the map configurations

### ✅ SQL Database
- `wormholes.db`: Database containing wormhole positions and connections
- `trajectories.db`: Database storing navigation trajectories
- Database schema and entries for wormhole positions

### ✅ C++ Code
- `multimap_nav_server.cpp`: Main action server implementation
- `map_manager.cpp`: Map management and switching logic
- `trajectory_manager.cpp`: Trajectory planning and execution
- `wormhole_database.cpp`: Database operations for wormholes
- `wormhole_detector.cpp`: Wormhole detection algorithms
- Header files (.h) for all components
- `CMakeLists.txt` and `package.xml` for building

### ✅ Action Server
- `MultiMapNav.action`: Action definition file
- Launch files for running the system
- Fully functional action server handling multi-map navigation goals

### ✅ Documentation
- Complete documentation of all functionality
- Setup and usage guides
- Integration instructions

## 🔧 System Requirements

- ROS Noetic
- SQLite3
- Catkin build system
- Gazebo simulator (for simulation mode)

## 📝 License

This project is part of the AR100 drone navigation system.

---
Generated on: 2025-08-01 22:06:11
