#include "multi_map_nav/wormhole_database.h"
#include <ros/console.h>
#include <sstream>
#include <iomanip>
#include <ctime>

namespace multi_map_nav {

WormholeDatabase::WormholeDatabase(const std::string& db_path)
    : db_(nullptr), db_path_(db_path), connected_(false) {
    ROS_INFO("Initializing WormholeDatabase with path: %s", db_path_.c_str());
}

WormholeDatabase::~WormholeDatabase() {
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool WormholeDatabase::initialize() {
    int rc = sqlite3_open(db_path_.c_str(), &db_);
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to open database: %s", sqlite3_errmsg(db_));
        return false;
    }
    
    connected_ = true;
    ROS_INFO("Successfully connected to database: %s", db_path_.c_str());
    
    return createTables();
}

bool WormholeDatabase::createTables() {
    const std::string create_table_sql = R"(
        CREATE TABLE IF NOT EXISTS wormholes (
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
        )
    )";
    
    if (!executeQuery(create_table_sql)) {
        ROS_ERROR("Failed to create wormholes table");
        return false;
    }
    
    ROS_INFO("Wormholes table created successfully");
    return true;
}

bool WormholeDatabase::addWormhole(const WormholeEntry& entry) {
    std::stringstream ss;
    ss << "INSERT INTO wormholes (map_a, map_b, wkt_a, wkt_b, "
       << "entry_x_a, entry_y_a, entry_x_b, entry_y_b, "
       << "exit_x_a, exit_y_a, exit_x_b, exit_y_b) "
       << "VALUES ('" << entry.map_a << "', '" << entry.map_b << "', "
       << "'" << entry.wkt_a << "', '" << entry.wkt_b << "', "
       << entry.entry_x_a << ", " << entry.entry_y_a << ", "
       << entry.entry_x_b << ", " << entry.entry_y_b << ", "
       << entry.exit_x_a << ", " << entry.exit_y_a << ", "
       << entry.exit_x_b << ", " << entry.exit_y_b << ")";
    
    if (!executeQuery(ss.str())) {
        ROS_ERROR("Failed to add wormhole to database");
        return false;
    }
    
    ROS_INFO("Successfully added wormhole from %s to %s", 
             entry.map_a.c_str(), entry.map_b.c_str());
    return true;
}

std::vector<WormholeEntry> WormholeDatabase::getWormholesForMap(const std::string& map_name) {
    std::vector<WormholeEntry> wormholes;
    
    std::string query = "SELECT * FROM wormholes WHERE map_a = '" + map_name + 
                       "' OR map_b = '" + map_name + "'";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to prepare query: %s", sqlite3_errmsg(db_));
        return wormholes;
    }
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        WormholeEntry entry;
        entry.id = sqlite3_column_int(stmt, 0);
        entry.map_a = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        entry.map_b = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        entry.wkt_a = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        entry.wkt_b = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
        entry.entry_x_a = sqlite3_column_double(stmt, 5);
        entry.entry_y_a = sqlite3_column_double(stmt, 6);
        entry.entry_x_b = sqlite3_column_double(stmt, 7);
        entry.entry_y_b = sqlite3_column_double(stmt, 8);
        entry.exit_x_a = sqlite3_column_double(stmt, 9);
        entry.exit_y_a = sqlite3_column_double(stmt, 10);
        entry.exit_x_b = sqlite3_column_double(stmt, 11);
        entry.exit_y_b = sqlite3_column_double(stmt, 12);
        entry.created_at = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 13));
        
        wormholes.push_back(entry);
    }
    
    sqlite3_finalize(stmt);
    return wormholes;
}

std::unique_ptr<WormholeEntry> WormholeDatabase::getWormholeById(int id) {
    std::string query = "SELECT * FROM wormholes WHERE id = " + std::to_string(id);
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to prepare query: %s", sqlite3_errmsg(db_));
        return nullptr;
    }
    
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        auto entry = std::make_unique<WormholeEntry>();
        entry->id = sqlite3_column_int(stmt, 0);
        entry->map_a = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        entry->map_b = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        entry->wkt_a = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        entry->wkt_b = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
        entry->entry_x_a = sqlite3_column_double(stmt, 5);
        entry->entry_y_a = sqlite3_column_double(stmt, 6);
        entry->entry_x_b = sqlite3_column_double(stmt, 7);
        entry->entry_y_b = sqlite3_column_double(stmt, 8);
        entry->exit_x_a = sqlite3_column_double(stmt, 9);
        entry->exit_y_a = sqlite3_column_double(stmt, 10);
        entry->exit_x_b = sqlite3_column_double(stmt, 11);
        entry->exit_y_b = sqlite3_column_double(stmt, 12);
        entry->created_at = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 13));
        
        sqlite3_finalize(stmt);
        return entry;
    }
    
    sqlite3_finalize(stmt);
    return nullptr;
}

bool WormholeDatabase::updateWormhole(const WormholeEntry& entry) {
    std::stringstream ss;
    ss << "UPDATE wormholes SET "
       << "map_a = '" << entry.map_a << "', "
       << "map_b = '" << entry.map_b << "', "
       << "wkt_a = '" << entry.wkt_a << "', "
       << "wkt_b = '" << entry.wkt_b << "', "
       << "entry_x_a = " << entry.entry_x_a << ", "
       << "entry_y_a = " << entry.entry_y_a << ", "
       << "entry_x_b = " << entry.entry_x_b << ", "
       << "entry_y_b = " << entry.entry_y_b << ", "
       << "exit_x_a = " << entry.exit_x_a << ", "
       << "exit_y_a = " << entry.exit_y_a << ", "
       << "exit_x_b = " << entry.exit_x_b << ", "
       << "exit_y_b = " << entry.exit_y_b << " "
       << "WHERE id = " << entry.id;
    
    if (!executeQuery(ss.str())) {
        ROS_ERROR("Failed to update wormhole in database");
        return false;
    }
    
    ROS_INFO("Successfully updated wormhole with ID %d", entry.id);
    return true;
}

bool WormholeDatabase::deleteWormhole(int id) {
    std::string query = "DELETE FROM wormholes WHERE id = " + std::to_string(id);
    
    if (!executeQuery(query)) {
        ROS_ERROR("Failed to delete wormhole from database");
        return false;
    }
    
    ROS_INFO("Successfully deleted wormhole with ID %d", id);
    return true;
}

std::vector<WormholeEntry> WormholeDatabase::getAllWormholes() {
    std::vector<WormholeEntry> wormholes;
    
    const std::string query = "SELECT * FROM wormholes ORDER BY id";
    
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to prepare query: %s", sqlite3_errmsg(db_));
        return wormholes;
    }
    
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        WormholeEntry entry;
        entry.id = sqlite3_column_int(stmt, 0);
        entry.map_a = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        entry.map_b = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        entry.wkt_a = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
        entry.wkt_b = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
        entry.entry_x_a = sqlite3_column_double(stmt, 5);
        entry.entry_y_a = sqlite3_column_double(stmt, 6);
        entry.entry_x_b = sqlite3_column_double(stmt, 7);
        entry.entry_y_b = sqlite3_column_double(stmt, 8);
        entry.exit_x_a = sqlite3_column_double(stmt, 9);
        entry.exit_y_a = sqlite3_column_double(stmt, 10);
        entry.exit_x_b = sqlite3_column_double(stmt, 11);
        entry.exit_y_b = sqlite3_column_double(stmt, 12);
        entry.created_at = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 13));
        
        wormholes.push_back(entry);
    }
    
    sqlite3_finalize(stmt);
    return wormholes;
}

bool WormholeDatabase::executeQuery(const std::string& sql) {
    char* err_msg = nullptr;
    int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err_msg);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    return true;
}

bool WormholeDatabase::isConnected() const {
    return connected_;
}

} // namespace multi_map_nav 