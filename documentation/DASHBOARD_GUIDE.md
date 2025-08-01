# Real-Time Navigation Dashboard Guide

## Overview

This guide covers the **real-time navigation dashboard system** that provides comprehensive monitoring of the 6-room multi-map navigation system. The dashboard includes both a **terminal-based interface** and a **web-based interface** for professional monitoring and demonstration.

## 🎯 Dashboard Features

### Real-Time Monitoring
- **Current Position**: Live coordinate updates
- **Current Room**: Automatic room detection
- **Target Information**: Goal room and position
- **Navigation Status**: Real-time status updates
- **Distance to Target**: Live distance calculation
- **Estimated Time**: Arrival time estimation
- **Progress Tracking**: Visual progress indicators

### Statistics Tracking
- **Rooms Visited**: Count of unique rooms visited
- **Total Distance**: Cumulative navigation distance
- **Total Time**: Session duration
- **Navigation History**: Recent navigation events
- **Wormhole Crossings**: Map switching events

### Visual Elements
- **Progress Bars**: Visual progress indicators
- **Room Layout**: Interactive room visualization
- **Charts**: Real-time navigation graphs
- **Alerts**: Special event notifications
- **Color Coding**: Room-specific colors

## 🚀 Quick Start

### Option 1: Complete Simulation with Dashboard
```bash
chmod +x run_simulation_with_dashboard.sh
./run_simulation_with_dashboard.sh
```

### Option 2: Terminal Dashboard Only
```bash
# Set up 6-room database
python3 setup_6_rooms.py

# Launch simulation
roslaunch multi_map_nav 6_room_simulation_standalone.launch

# In another terminal, run dashboard
python3 navigation_dashboard.py
```

### Option 3: Web Dashboard Only
```bash
# Set up 6-room database
python3 setup_6_rooms.py

# Launch simulation
roslaunch multi_map_nav 6_room_simulation_standalone.launch

# In another terminal, run web dashboard
python3 web_dashboard.py

# Open browser to http://localhost:5000
```

## 📊 Terminal Dashboard

### Features
- **Real-time Updates**: 500ms refresh rate
- **Clear Display**: Formatted console output
- **Comprehensive Stats**: All navigation metrics
- **Progress Visualization**: ASCII progress bars
- **Room Status**: Current and target room display

### Display Sections
1. **Current Status**: Position, orientation, room
2. **Target Information**: Goal room and position
3. **Navigation Statistics**: Visited rooms, time, distance
4. **Room Layout**: Visual room arrangement
5. **Progress Bar**: Navigation completion percentage
6. **System Info**: Timestamp and ROS rate

### Sample Output
```
================================================================================
🚀 REAL-TIME NAVIGATION DASHBOARD
================================================================================

📍 CURRENT STATUS
----------------------------------------
🏠 Current Room:     Living Room
📍 Position:         (2.34, 1.67)
🧭 Orientation:      45.2°
🎯 Target Room:      Kitchen
🎯 Target Position:  (8.00, 1.00)
📊 Status:           Navigating
📏 Distance to Target: 5.78 m
⏱️  Estimated Time:   11.6 s

📈 NAVIGATION STATISTICS
----------------------------------------
🏠 Rooms Visited:    2
📋 Visited Rooms:    Living Room, Bedroom
⏱️  Total Time:       2.3 min
📏 Total Distance:   15.67 m

🏠 ROOM LAYOUT
----------------------------------------
📍 Living Room    (Center: 0, 0)
🎯 Kitchen        (Center: 10, 0)
  Bedroom         (Center: 0, 10)
  Bathroom        (Center: 10, 10)
  Office          (Center: 5, 5)
  Garage          (Center: 15, 5)

📊 NAVIGATION PROGRESS
----------------------------------------
[████████████████████████████░░] 85.2%

🔧 SYSTEM INFO
----------------------------------------
🕐 Timestamp:        14:32:15
📡 ROS Rate:         10 Hz
🗺️  Active Maps:      6 rooms available
```

## 🌐 Web Dashboard

### Features
- **Professional Interface**: Modern web design
- **Real-time Updates**: Live data refresh
- **Interactive Charts**: Chart.js integration
- **Responsive Design**: Mobile-friendly layout
- **Visual Alerts**: Event notifications
- **Room Visualization**: Color-coded rooms

### Dashboard Sections

#### 1. Current Status Card
- Current room and position
- Robot orientation
- Navigation status
- Real-time updates

#### 2. Target Information Card
- Target room and position
- Distance to target
- Estimated arrival time
- Progress bar visualization

#### 3. Navigation Statistics Card
- Rooms visited count
- Total navigation time
- Total distance traveled
- List of visited rooms

#### 4. Room Layout Card
- Color-coded room display
- Current room highlighting
- Target room highlighting
- Interactive room grid

#### 5. Navigation Progress Chart
- Real-time distance tracking
- Time-series visualization
- Chart.js integration
- Live data updates

#### 6. Statistics Cards
- Rooms visited counter
- Total distance display
- Total time tracking
- Progress percentage

### Visual Elements

#### Room Colors
- **Living Room**: Blue (#3498db)
- **Kitchen**: Green (#2ecc71)
- **Bedroom**: Purple (#9b59b6)
- **Bathroom**: Yellow (#f1c40f)
- **Office**: Orange (#e67e22)
- **Garage**: Red (#e74c3c)

#### Status Indicators
- **Current Room**: Pulsing blue highlight
- **Target Room**: Red highlight
- **Progress Bar**: Gradient fill
- **Alerts**: Color-coded notifications

## 🎮 Dashboard Controls

### Terminal Dashboard
- **Auto-refresh**: Updates every 500ms
- **Clear display**: Screen clearing for readability
- **Keyboard interrupt**: Ctrl+C to stop
- **Real-time data**: Live ROS topic monitoring

### Web Dashboard
- **Browser access**: http://localhost:5000
- **Auto-refresh**: Updates every 500ms
- **Interactive charts**: Click and hover
- **Responsive design**: Works on all devices

## 📈 Monitoring Capabilities

### Real-Time Data
- **Position Tracking**: Live coordinate updates
- **Room Detection**: Automatic room switching
- **Path Planning**: Navigation path visualization
- **Wormhole Events**: Map switching detection
- **Progress Monitoring**: Distance and time tracking

### Statistics Collection
- **Navigation History**: Complete session tracking
- **Performance Metrics**: Time and distance stats
- **Room Visits**: Unique room tracking
- **Event Logging**: Special event recording

### Alert System
- **Wormhole Crossing**: Map switching notifications
- **Target Approach**: Near-target alerts
- **Navigation Complete**: Goal achievement alerts
- **Error Detection**: System issue notifications

## 🛠️ Technical Implementation

### Terminal Dashboard
- **Language**: Python 3
- **Dependencies**: rospy, threading, time
- **Topics**: /amcl_pose, /move_base/NavfnROS/plan, /map_manager/active_map
- **Display**: Console-based with ANSI colors
- **Update Rate**: 500ms

### Web Dashboard
- **Backend**: Flask (Python)
- **Frontend**: HTML5, CSS3, JavaScript
- **Charts**: Chart.js
- **Real-time**: AJAX polling
- **Port**: 5000

### Data Sources
- **Robot Position**: /amcl_pose topic
- **Navigation Path**: /move_base/NavfnROS/plan topic
- **Active Map**: /map_manager/active_map topic
- **Wormhole Events**: /wormhole_crossed topic

## 🎯 Use Cases

### Demonstration
- **Professional Presentation**: Web dashboard for audiences
- **Real-time Monitoring**: Live navigation tracking
- **Statistics Display**: Comprehensive metrics
- **Visual Feedback**: Progress and status indicators

### Development
- **Debugging**: Real-time system monitoring
- **Performance Analysis**: Navigation statistics
- **Testing**: Comprehensive system validation
- **Documentation**: Visual system behavior

### Education
- **Learning Tool**: Visual navigation concepts
- **Interactive Display**: Real-time system behavior
- **Statistics Learning**: Navigation metrics
- **System Understanding**: Multi-map concepts

## 🚀 Advanced Features

### Customization
- **Update Rate**: Configurable refresh intervals
- **Display Options**: Customizable layouts
- **Alert Thresholds**: Adjustable notification levels
- **Chart Types**: Multiple visualization options

### Integration
- **ROS Integration**: Native ROS topic support
- **Database Logging**: Persistent data storage
- **Export Capabilities**: Data export functions
- **API Access**: RESTful API endpoints

### Extensibility
- **Plugin System**: Modular dashboard components
- **Custom Metrics**: User-defined statistics
- **Additional Views**: New visualization types
- **Multi-robot Support**: Multiple robot monitoring

## 📁 Files Created

### Dashboard Files
- `navigation_dashboard.py` - Terminal dashboard
- `web_dashboard.py` - Web dashboard server
- `templates/dashboard.html` - Web interface
- `run_simulation_with_dashboard.sh` - Complete setup script

### Documentation
- `DASHBOARD_GUIDE.md` - This comprehensive guide
- Dashboard feature descriptions
- Technical implementation details

## 🎉 Benefits

### Professional Quality
- **Modern Interface**: Professional web design
- **Real-time Updates**: Live data monitoring
- **Comprehensive Stats**: Complete navigation metrics
- **Visual Feedback**: Progress and status indicators

### Educational Value
- **System Understanding**: Visual navigation concepts
- **Real-time Learning**: Live system behavior
- **Statistics Education**: Navigation metrics
- **Interactive Experience**: Hands-on monitoring

### Demonstration Quality
- **Audience Appeal**: Professional web interface
- **Comprehensive Coverage**: All system aspects
- **Real-time Feedback**: Live navigation tracking
- **Visual Impact**: Impressive monitoring system

---

**The real-time navigation dashboard system provides comprehensive monitoring capabilities for the 6-room multi-map navigation system, offering both terminal and web interfaces for professional demonstration and development.** 📊🌐🚀 