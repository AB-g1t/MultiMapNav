# 6-Room Multi-Map Navigation Simulation Guide

## Overview

This guide covers the **6-room simulation environment** in Gazebo, which provides a **visual representation** of the multi-map navigation system. The simulation includes colored rooms, wormhole markers, and a realistic house layout that demonstrates the navigation capabilities in a visually appealing way.

## 🏠 Visual Room Layout

### Room Configuration in Simulation
```
                    Garage (Red)
                       |
    Bedroom (Purple) ----- Office (Orange) ----- Kitchen (Green)
         |                |                |
         |                |                |
    Living Room (Blue) ----- | -------- Bathroom (Yellow)
```

### Room Colors and Features
| Room | Color | Position | Description |
|------|-------|----------|-------------|
| **Living Room** | Blue | (0, 0) | Starting point, main living area |
| **Kitchen** | Green | (10, 0) | Cooking area, connected to living room |
| **Bedroom** | Purple | (0, 10) | Sleeping area, connected to living room |
| **Bathroom** | Yellow | (10, 10) | Bathroom facilities |
| **Office** | Orange | (5, 5) | **Central Hub** - connects to all rooms |
| **Garage** | Red | (15, 5) | Vehicle storage, connected to kitchen/bathroom |

### Visual Elements
- **Colored Floor Areas**: Each room has a distinct color for easy identification
- **Gray Walls**: Separate rooms and create realistic boundaries
- **Wormhole Markers**: Magenta circles at room connection points
- **Central Hub Marker**: Cyan circle in the Office (central routing point)
- **Room Labels**: Floating labels above each room

## 🚀 Quick Start

### Option 1: Automated Simulation
```bash
chmod +x run_6_room_simulation.sh
./run_6_room_simulation.sh
```

### Option 2: Manual Launch
```bash
# Set up 6-room database
python3 setup_6_rooms.py

# Launch simulation
roslaunch multi_map_nav 6_room_simulation_standalone.launch

# Run demonstration
python3 demo_6_rooms.py
```

## 🎯 What You'll See

### Gazebo Simulation Window
1. **6 Colored Rooms**: Each room has a distinct colored floor
2. **Gray Walls**: Separate rooms and create realistic boundaries
3. **Wormhole Markers**: Magenta circles at room connection points
4. **Robot**: Spawned in the Living Room (blue area)
5. **Central Hub**: Cyan circle in the Office (orange area)

### RViz Visualization
1. **Map Display**: Shows current active map
2. **Robot Model**: 3D robot representation
3. **Navigation Path**: Green path showing planned route
4. **Trajectory**: Red line showing actual robot path
5. **Wormhole Markers**: Visual indicators of wormhole regions
6. **Particle Cloud**: AMCL localization particles

## 🎮 Simulation Controls

### Gazebo Controls
- **Mouse**: Rotate and zoom camera
- **Arrow Keys**: Move camera around the environment
- **Scroll Wheel**: Zoom in/out
- **Right Click**: Pan camera

### RViz Controls
- **Mouse**: Rotate and zoom view
- **Shift + Mouse**: Pan view
- **Scroll Wheel**: Zoom in/out

## 🔍 Visual Features Explained

### Room Colors
- **Blue (Living Room)**: Starting point, main living area
- **Green (Kitchen)**: Cooking area, connected to living room
- **Purple (Bedroom)**: Sleeping area, connected to living room
- **Yellow (Bathroom)**: Bathroom facilities
- **Orange (Office)**: Central hub, connects to all rooms
- **Red (Garage)**: Vehicle storage, connected to kitchen/bathroom

### Wormhole Markers
- **Magenta Circles**: Direct room-to-room connections
- **Cyan Circle**: Central hub in Office (larger radius)
- **Location**: Positioned at room boundaries where wormholes are defined
- **Function**: Visual indicators of where map switching occurs

### Walls and Boundaries
- **Gray Walls**: Separate rooms and create realistic boundaries
- **Height**: 2 meters tall
- **Thickness**: 0.2 meters
- **Function**: Create visual separation between rooms

## 🎬 Demonstration Scenarios

### Visual Navigation Paths
1. **Living Room → Kitchen**: Robot moves from blue to green area
2. **Living Room → Bedroom**: Robot moves from blue to purple area
3. **Bedroom → Office**: Robot moves from purple to orange area
4. **Office → Garage**: Robot moves from orange to red area
5. **Garage → Bathroom**: Robot moves from red to yellow area
6. **Bathroom → Living Room**: Robot returns to blue area

### Map Switching Visualization
- **Wormhole Detection**: Robot enters magenta/cyan circles
- **Map Change**: RViz shows map switching
- **Trajectory Recording**: Red line shows robot's path
- **Navigation Path**: Green line shows planned route

## 📊 Simulation Features

### Core Capabilities
- ✅ **Visual Room Representation**: 6 colored rooms in Gazebo
- ✅ **Wormhole Visualization**: Markers at connection points
- ✅ **Realistic Layout**: Walls and room separation
- ✅ **Robot Navigation**: Visual robot movement between rooms
- ✅ **Map Switching**: Observable in both Gazebo and RViz
- ✅ **Trajectory Recording**: Visual path tracking
- ✅ **Central Hub**: Special marker in Office

### Advanced Features
- ✅ **Multi-Camera Views**: Gazebo and RViz simultaneously
- ✅ **Real-time Visualization**: Live navigation feedback
- ✅ **Interactive Environment**: Camera controls and exploration
- ✅ **Professional Quality**: Production-ready simulation

## 🛠️ Technical Details

### Gazebo World File
- **File**: `src/multi_map_nav/worlds/6_room_house.world`
- **Format**: SDF (Simulation Description Format)
- **Features**: 6 rooms, walls, wormhole markers, room labels

### Launch File
- **File**: `src/multi_map_nav/launch/6_room_simulation.launch`
- **Features**: Gazebo world loading, robot spawning, navigation setup

### Simulation Components
1. **Gazebo World**: 6-room house with visual elements
2. **Robot Model**: Anscer robot spawned in Living Room
3. **Navigation Stack**: move_base, AMCL, path planning
4. **Multi-Map System**: Map switching and wormhole detection
5. **RViz Visualization**: Real-time navigation feedback

## 🎉 Benefits of Simulation

### Enhanced Understanding
- **Visual Clarity**: See exactly where rooms are located
- **Wormhole Visualization**: Understand connection points
- **Real-time Feedback**: Watch navigation in action
- **Professional Presentation**: Impressive visual demonstration

### Educational Value
- **Spatial Understanding**: See room relationships
- **Navigation Logic**: Understand path planning
- **System Architecture**: Visualize multi-map concept
- **Wormhole Mechanism**: See how map switching works

### Demonstration Quality
- **Professional Appearance**: High-quality visual simulation
- **Comprehensive Coverage**: All 6 rooms and connections
- **Interactive Experience**: Explore the environment
- **Recording Ready**: Perfect for video demonstrations

## 🚀 Next Steps

### For Demonstration
1. **Run the simulation**: `./run_6_room_simulation.sh`
2. **Explore the environment**: Use camera controls in Gazebo
3. **Watch navigation**: Observe robot movement between rooms
4. **Record the demonstration**: Capture both Gazebo and RViz
5. **Document the results**: Show visual navigation capabilities

### For Development
1. **Modify room layouts**: Change room positions or sizes
2. **Add more rooms**: Extend the simulation further
3. **Customize wormholes**: Adjust connection points
4. **Add obstacles**: Create more complex environments
5. **Implement custom behaviors**: Room-specific robot actions

## 🎯 Success Criteria

The simulation is successful when:
1. **All 6 rooms are visible** with distinct colors
2. **Wormhole markers are present** at connection points
3. **Robot navigates smoothly** between rooms
4. **Map switching occurs** when robot enters wormhole areas
5. **Trajectory is recorded** and visible in RViz
6. **Both Gazebo and RViz** show synchronized information

## 📁 Files Created

### Simulation Files
- `6_room_house.world` - Gazebo world with 6 rooms
- `6_room_simulation.launch` - Simulation launch file
- `run_6_room_simulation.sh` - Automated simulation script

### Documentation
- `6_ROOM_SIMULATION_GUIDE.md` - This comprehensive guide
- Visual room layout diagrams
- Simulation feature descriptions

---

**The 6-room simulation provides a visually stunning and educationally valuable demonstration of the multi-map navigation system, making complex concepts easy to understand and impressive to present.** 🏠🎮🚀 