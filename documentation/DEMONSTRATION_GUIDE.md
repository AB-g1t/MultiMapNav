# Multi-Map Navigation Demonstration Guide

## Overview

This guide will walk you through running a complete demonstration of the multi-map navigation system with wormhole functionality. The demonstration showcases all the features required by the problem statement.

## Prerequisites

- ROS Noetic installed and configured
- Catkin workspace set up
- Python 3 with required packages
- SQLite3
- RViz for visualization

## Quick Start

### Option 1: Automated Demonstration (Recommended)

1. **Install dependencies (if needed):**
   ```bash
   chmod +x install_dependencies.sh
   ./install_dependencies.sh
   ```

2. **Run the automated demonstration script:**
   ```bash
   chmod +x run_demonstration.sh
   ./run_demonstration.sh
   ```

2. **Follow the on-screen instructions** - the script will:
   - Build the package
   - Set up the database
   - Launch the system
   - Run the demonstration
   - Show results

### Option 2: Manual Step-by-Step

1. **Build the package:**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

2. **Set up the database:**
   ```bash
   python3 setup_demo.py
   ```

3. **Test components:**
   ```bash
   python3 test_components.py
   ```

4. **Launch the system:**
   ```bash
   roslaunch multi_map_nav multimap_demo_fixed.launch
   ```

5. **Run the demonstration:**
   ```bash
   python3 demo_navigation.py
   ```

## What the Demonstration Shows

### 1. System Initialization
- **Database Setup**: Creates wormholes.db with sample wormhole entries
- **Map Loading**: Loads room_a and room_b maps
- **Node Startup**: Initializes all required ROS nodes
- **RViz Visualization**: Opens RViz with proper configuration

### 2. Navigation Demonstrations

#### Demo 1: Same-Map Navigation
- **Goal**: Navigate within room_a (3.0, 2.0)
- **Expected Behavior**: Direct navigation using move_base
- **Visualization**: Robot moves directly to target

#### Demo 2: Cross-Map Navigation (room_a → room_b)
- **Goal**: Navigate to room_b (-2.0, -1.0)
- **Expected Behavior**: 
  1. Robot navigates to wormhole entry point
  2. Wormhole detection triggers
  3. Map switches from room_a to room_b
  4. Robot continues to final target
- **Visualization**: Map changes in RViz, trajectory shows wormhole crossing

#### Demo 3: Cross-Map Navigation (room_b → room_a)
- **Goal**: Navigate back to room_a (1.0, 1.0)
- **Expected Behavior**: Reverse wormhole crossing
- **Visualization**: Map switches back to room_a

#### Demo 4: Final Navigation
- **Goal**: Navigate to final position in room_a (4.0, 4.0)
- **Expected Behavior**: Direct navigation within room_a
- **Visualization**: Robot completes final movement

### 3. Features Demonstrated

#### Core Requirements ✅
- **Multi-Map Navigation**: Seamless switching between room_a and room_b
- **Wormhole Mechanism**: Automatic detection and activation
- **SQL Database**: Persistent storage of wormhole positions
- **C++ Implementation**: All core functionality in C++
- **Action Server**: Handles navigation goals with target map and position

#### Bonus Features ✅
- **Trajectory Collection**: Records and visualizes robot paths
- **Modular Architecture**: Separate nodes for different functions
- **Comprehensive Documentation**: Detailed README and code comments
- **OOP Principles**: Classes with header/source separation
- **No Hardcoded Paths**: Parameter-based configuration
- **Coding Guidelines**: Clean, maintainable code

## Expected Output

### Console Output
```
🚀 MULTI-MAP NAVIGATION DEMONSTRATION
======================================

📋 Demo 1: Same-map navigation
🎯 Navigate within room_a
   Target Map: room_a
   Position: (3.0, 2.0)
   Yaw: 0.0°
✅ Navigation completed successfully!

📋 Demo 2: Cross-map navigation (room_a → room_b)
🎯 Navigate from room_a to room_b
   Target Map: room_b
   Position: (-2.0, -1.0)
   Yaw: 90.0°
✅ Navigation completed successfully!

📋 Demo 3: Cross-map navigation (room_b → room_a)
🎯 Navigate from room_b back to room_a
   Target Map: room_a
   Position: (1.0, 1.0)
   Yaw: -90.0°
✅ Navigation completed successfully!

📋 Demo 4: Final navigation within room_a
🎯 Navigate to final position in room_a
   Target Map: room_a
   Position: (4.0, 4.0)
   Yaw: 180.0°
✅ Navigation completed successfully!

🎉 DEMONSTRATION COMPLETED!
```

### RViz Visualization
- **Map Display**: Shows current active map
- **Robot Model**: 3D robot representation
- **Navigation Path**: Green path showing planned route
- **Trajectory**: Red line showing actual robot path
- **Wormhole Markers**: Visual indicators of wormhole regions
- **Particle Cloud**: AMCL localization particles

## Troubleshooting

### Common Issues

1. **"Package not found" error**
   - Solution: Run `catkin_make` and `source devel/setup.bash`

2. **"Action server not available"**
   - Solution: Ensure the launch file is running and wait for initialization

3. **"Database not found"**
   - Solution: Run `python3 setup_demo.py` to create the database

4. **"Map files empty"**
   - Solution: This is normal for demonstration - the system uses placeholder maps

5. **"RViz not opening"**
   - Solution: Check if RViz is installed and the configuration file exists

### Debug Commands

```bash
# Check ROS topics
rostopic list

# Check node status
rosnode list

# View database contents (Python-based, no sqlite3 required)
python3 view_database.py

# Alternative: Use sqlite3 if available
sqlite3 wormholes.db "SELECT * FROM wormholes;"
sqlite3 trajectories.db "SELECT * FROM trajectories;"

# Monitor wormhole events
rostopic echo /wormhole_crossed

# Monitor map changes
rostopic echo /map_manager/active_map
```

## Recording the Demonstration

### Screen Recording Setup
1. **Start screen recording software** (e.g., OBS, SimpleScreenRecorder)
2. **Arrange windows**: RViz on one side, terminal on the other
3. **Run the demonstration**: Execute `./run_demonstration.sh`
4. **Record the entire process**: From startup to completion

### What to Capture
- System startup and initialization
- Database setup confirmation
- RViz opening with proper configuration
- Each navigation demonstration
- Map switching events
- Trajectory visualization
- Final completion message

### Recording Tips
- **Duration**: Expect 3-5 minutes for complete demonstration
- **Resolution**: Use 1080p or higher for clear visualization
- **Audio**: Optional - can include voice narration
- **Focus**: Keep RViz window prominent for visualization

## Submission Checklist

Before submitting, ensure you have:

- [ ] Complete ROS package with all source files
- [ ] Working demonstration that runs without errors
- [ ] Screen recording showing all functionality
- [ ] README.md with comprehensive documentation
- [ ] Database files (wormholes.db, trajectories.db)
- [ ] All bonus features implemented and demonstrated
- [ ] Code follows OOP principles and coding guidelines
- [ ] No hardcoded paths in source code

## Success Criteria

The demonstration is successful when:

1. **All navigation goals complete successfully**
2. **Map switching works seamlessly**
3. **Wormhole detection triggers properly**
4. **Trajectory recording functions correctly**
5. **RViz visualization shows all components**
6. **Database stores and retrieves data properly**
7. **Action server handles all goal types**

## Next Steps

After running the demonstration:

1. **Review the trajectory data** in trajectories.db
2. **Check wormhole configuration** in wormholes.db
3. **Verify all features work as expected**
4. **Record a video of the demonstration**
5. **Prepare submission package**
6. **Submit with confidence!**

---

**Good luck with your demonstration!** 🚀 