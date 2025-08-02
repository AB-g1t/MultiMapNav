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
    
    // CORRECTED warehouse wormhole entries
    // Based on Gazebo world layout: Room A(0,0), Room B(50,0), Room C(0,50), Room D(50,50), Room E(100,0), Room F(100,50)
    std::vector<multi_map_nav::WormholeEntry> wormholes = {
        
        // Wormhole 1: Room A (0,0) to Room B (50,0) - East connection
        {1, "room_a", "room_b", 
         "POLYGON((6 -3, 14 -3, 14 3, 6 3, 6 -3))",      // Larger detection region in room_a
         "POLYGON((36 -3, 44 -3, 44 3, 36 3, 36 -3))",   // Detection region in room_b  
         10.0, 0.0,   // Entry point in room_a (east wall)
         40.0, 0.0,   // Exit point in room_b (west wall) - CORRECTED!
         40.0, 0.0,   // Entry point in room_b (for return journey)
         10.0, 0.0},  // Exit point in room_a (for return journey)
         
        // Wormhole 2: Room A (0,0) to Room C (0,50) - North connection  
        {2, "room_a", "room_c",
         "POLYGON((-3 6, 3 6, 3 14, -3 14, -3 6))",      // Detection region in room_a (north wall)
         "POLYGON((-3 36, 3 36, 3 44, -3 44, -3 36))",   // Detection region in room_c (south wall)
         0.0, 10.0,   // Entry point in room_a (north wall)
         0.0, 40.0,   // Exit point in room_c (south wall) - CORRECTED!
         0.0, 40.0,   // Entry point in room_c (for return)
         0.0, 10.0},  // Exit point in room_a (for return)
         
        // Wormhole 3: Room B (50,0) to Room D (50,50) - North connection
        {3, "room_b", "room_d",
         "POLYGON((46 6, 54 6, 54 14, 46 14, 46 6))",     // Detection region in room_b (north wall)
         "POLYGON((46 36, 54 36, 54 44, 46 44, 46 36))",  // Detection region in room_d (south wall)
         50.0, 10.0,  // Entry point in room_b (north wall)
         50.0, 40.0,  // Exit point in room_d (south wall) - CORRECTED!
         50.0, 40.0,  // Entry point in room_d (for return)
         50.0, 10.0}, // Exit point in room_b (for return)
         
        // Wormhole 4: Room C (0,50) to Room D (50,50) - East connection
        {4, "room_c", "room_d",
         "POLYGON((6 46, 14 46, 14 54, 6 54, 6 46))",     // Detection region in room_c (east wall)
         "POLYGON((36 46, 44 46, 44 54, 36 54, 36 46))",  // Detection region in room_d (west wall)
         10.0, 50.0,  // Entry point in room_c (east wall)
         40.0, 50.0,  // Exit point in room_d (west wall) - CORRECTED!
         40.0, 50.0,  // Entry point in room_d (for return)
         10.0, 50.0}, // Exit point in room_c (for return)
         
        // Wormhole 5: Room A (0,0) to Room E (100,0) - Long distance connection
        {5, "room_a", "room_e",
         "POLYGON((6 -8, 14 -8, 14 0, 6 0, 6 -8))",      // Detection region in room_a (southeast)
         "POLYGON((86 -3, 94 -3, 94 3, 86 3, 86 -3))",   // Detection region in room_e (west wall)
         10.0, -4.0,  // Entry point in room_a (southeast area)
         90.0, 0.0,   // Exit point in room_e (west wall) - CORRECTED!
         90.0, 0.0,   // Entry point in room_e (for return)
         10.0, -4.0}, // Exit point in room_a (for return)
         
        // Wormhole 6: Room D (50,50) to Room F (100,50) - East connection
        {6, "room_d", "room_f",
         "POLYGON((56 46, 64 46, 64 54, 56 54, 56 46))",  // Detection region in room_d (east wall)
         "POLYGON((86 46, 94 46, 94 54, 86 54, 86 46))",  // Detection region in room_f (west wall)
         60.0, 50.0,  // Entry point in room_d (east wall)
         90.0, 50.0,  // Exit point in room_f (west wall) - CORRECTED!
         90.0, 50.0,  // Entry point in room_f (for return)
         60.0, 50.0}, // Exit point in room_d (for return)
         
        // Wormhole 7: Room E (100,0) to Room F (100,50) - North connection
        {7, "room_e", "room_f",
         "POLYGON((96 6, 104 6, 104 14, 96 14, 96 6))",   // Detection region in room_e (north wall)
         "POLYGON((96 36, 104 36, 104 44, 96 44, 96 36))", // Detection region in room_f (south wall)
         100.0, 10.0, // Entry point in room_e (north wall)
         100.0, 40.0, // Exit point in room_f (south wall) - CORRECTED!
         100.0, 40.0, // Entry point in room_f (for return)
         100.0, 10.0} // Exit point in room_e (for return)
    };
    
    // Add all wormholes to database
    for (const auto& wormhole : wormholes) {
        if (!db.addWormhole(wormhole)) {
            ROS_ERROR("Failed to add wormhole %d", wormhole.id);
            return 1;
        }
        ROS_INFO("Added wormhole %d: %s -> %s", wormhole.id, wormhole.map_a.c_str(), wormhole.map_b.c_str());
        ROS_INFO("  Entry: (%.1f, %.1f) -> Exit: (%.1f, %.1f)", 
                 wormhole.entry_x_a, wormhole.entry_y_a, 
                 wormhole.exit_x_b, wormhole.exit_y_b);
    }
    
    ROS_INFO("Successfully populated database with %zu warehouse wormholes", wormholes.size());
    
    // Verify by reading back
    auto all_wormholes = db.getAllWormholes();
    ROS_INFO("Database now contains %zu wormholes", all_wormholes.size());
    
    return 0;
}
