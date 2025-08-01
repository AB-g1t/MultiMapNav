#include "multi_map_nav/wormhole_database.h"
#include <ros/ros.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "populate_warehouse_wormholes");
    
    std::string db_path = "wormholes.db";
    if (argc > 1) {
        db_path = argv[1];
    }
    
    multi_map_nav::WormholeDatabase db(db_path);
    
    if (!db.initialize()) {
        ROS_ERROR("Failed to initialize database");
        return 1;
    }
    
    // Clear existing wormholes
    ROS_INFO("Clearing existing wormholes...");
    auto existing_wormholes = db.getAllWormholes();
    for (const auto& wormhole : existing_wormholes) {
        db.deleteWormhole(wormhole.id);
    }
    ROS_INFO("Deleted %zu existing wormholes", existing_wormholes.size());
    
    // Warehouse wormhole entries
    std::vector<multi_map_nav::WormholeEntry> wormholes = {
        // Room A (Receiving) to Room B (Storage 1)
        {1, "room_a", "room_b", 
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         8.0, 8.0, 8.0, 8.0,  // entry points
         8.0, 8.0, 8.0, 8.0}, // exit points
        
        // Room A (Receiving) to Room C (Processing)
        {2, "room_a", "room_c",
         "POLYGON((6 -10, 10 -10, 10 -6, 6 -6, 6 -10))",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         8.0, -8.0, 8.0, 8.0,  // entry points
         8.0, -8.0, 8.0, 8.0}, // exit points
        
        // Room B (Storage 1) to Room D (Storage 2)
        {3, "room_b", "room_d",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         8.0, 8.0, 8.0, 8.0,  // entry points
         8.0, 8.0, 8.0, 8.0}, // exit points
        
        // Room C (Processing) to Room D (Storage 2)
        {4, "room_c", "room_d",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         8.0, 8.0, 8.0, 8.0,  // entry points
         8.0, 8.0, 8.0, 8.0}, // exit points
        
        // Room A (Receiving) to Room E (Shipping)
        {5, "room_a", "room_e",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         8.0, 8.0, 8.0, 8.0,  // entry points
         8.0, 8.0, 8.0, 8.0}, // exit points
        
        // Room D (Storage 2) to Room F (Quality Control)
        {6, "room_d", "room_f",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         8.0, 8.0, 8.0, 8.0,  // entry points
         8.0, 8.0, 8.0, 8.0}, // exit points
        
        // Room E (Shipping) to Room F (Quality Control)
        {7, "room_e", "room_f",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         "POLYGON((6 6, 10 6, 10 10, 6 10, 6 6))",
         8.0, 8.0, 8.0, 8.0,  // entry points
         8.0, 8.0, 8.0, 8.0}  // exit points
    };
    
    // Add all wormholes to database
    for (const auto& wormhole : wormholes) {
        if (!db.addWormhole(wormhole)) {
            ROS_ERROR("Failed to add wormhole %d", wormhole.id);
            return 1;
        }
    }
    
    ROS_INFO("Successfully populated database with %zu warehouse wormholes", wormholes.size());
    
    // Verify by reading back
    auto all_wormholes = db.getAllWormholes();
    ROS_INFO("Database now contains %zu wormholes", all_wormholes.size());
    
    return 0;
} 