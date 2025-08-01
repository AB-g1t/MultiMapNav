#pragma once

#include <ros/ros.h>
#include <sqlite3.h>
#include <string>
#include <vector>
#include <memory>

namespace multi_map_nav {

/**
 * @brief Structure to represent a wormhole entry in the database
 */
struct WormholeEntry {
    int id;
    std::string map_a;
    std::string map_b;
    std::string wkt_a;  // Well-Known Text representation of region A
    std::string wkt_b;  // Well-Known Text representation of region B
    double entry_x_a, entry_y_a;  // Entry point in map A
    double entry_x_b, entry_y_b;  // Entry point in map B
    double exit_x_a, exit_y_a;    // Exit point in map A
    double exit_x_b, exit_y_b;    // Exit point in map B
    std::string created_at;
};

/**
 * @brief SQL Database manager for wormhole positions
 * 
 * This class provides an interface to store and retrieve wormhole information
 * from an SQLite database. It handles the creation, querying, and management
 * of wormhole entries between different maps.
 */
class WormholeDatabase {
public:
    /**
     * @brief Constructor
     * @param db_path Path to the SQLite database file
     */
    explicit WormholeDatabase(const std::string& db_path);
    
    /**
     * @brief Destructor
     */
    ~WormholeDatabase();
    
    /**
     * @brief Initialize the database and create tables if they don't exist
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Add a new wormhole entry to the database
     * @param entry The wormhole entry to add
     * @return true if successful, false otherwise
     */
    bool addWormhole(const WormholeEntry& entry);
    
    /**
     * @brief Get all wormholes for a specific map
     * @param map_name Name of the map
     * @return Vector of wormhole entries
     */
    std::vector<WormholeEntry> getWormholesForMap(const std::string& map_name);
    
    /**
     * @brief Get a specific wormhole by ID
     * @param id Wormhole ID
     * @return Wormhole entry if found, nullptr otherwise
     */
    std::unique_ptr<WormholeEntry> getWormholeById(int id);
    
    /**
     * @brief Update an existing wormhole entry
     * @param entry The updated wormhole entry
     * @return true if successful, false otherwise
     */
    bool updateWormhole(const WormholeEntry& entry);
    
    /**
     * @brief Delete a wormhole entry by ID
     * @param id Wormhole ID to delete
     * @return true if successful, false otherwise
     */
    bool deleteWormhole(int id);
    
    /**
     * @brief Get all wormholes in the database
     * @return Vector of all wormhole entries
     */
    std::vector<WormholeEntry> getAllWormholes();
    
    /**
     * @brief Check if the database is connected
     * @return true if connected, false otherwise
     */
    bool isConnected() const;

private:
    sqlite3* db_;
    std::string db_path_;
    bool connected_;
    
    /**
     * @brief Create the wormholes table if it doesn't exist
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
     * @brief Convert a database row to a WormholeEntry
     * @param row Database row data
     * @return WormholeEntry object
     */
    WormholeEntry rowToWormholeEntry(const std::vector<std::string>& row);
};

} // namespace multi_map_nav 