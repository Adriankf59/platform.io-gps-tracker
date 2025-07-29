// Config.h - FIXED Configuration dengan Enhanced Real GPS Offline Storage
#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// ENHANCED FIXED REAL GPS OFFLINE DATA STORAGE CONFIGURATION
// ========================================
#define ENABLE_OFFLINE_STORAGE true              // Enable FIXED real GPS offline data storage
#define OFFLINE_MAX_RECORDS 100                  // Maximum offline records
#define OFFLINE_RECORD_SIZE 220                  // Increased size per record (bytes) untuk real GPS data
#define OFFLINE_AUTO_SYNC true                   // Auto sync when network available
#define OFFLINE_SYNC_BATCH_SIZE 5                // Records per sync batch
#define OFFLINE_SYNC_INTERVAL 2000               // Interval between batches (ms)
#define OFFLINE_MAINTENANCE_INTERVAL 300000      // Cleanup interval (5 minutes)
#define OFFLINE_MAX_AGE_HOURS 24                 // Auto-delete records older than 24 hours
#define OFFLINE_STORAGE_WARNING 15               // Warning when storage near full
#define OFFLINE_COMPRESSION_ENABLED false        // Enable data compression
#define OFFLINE_DEBUG_MODE true                  // Enable verbose offline logging (for testing)
#define NETWORK_CHECK_INTERVAL 1000              // Check network every 1 second
#define OFFLINE_SYNC_PRIORITY true               // Prioritize sync offline data
#define OFFLINE_SYNC_RETRY_INTERVAL 3000         // Retry sync every 3 seconds (FIXED: reduced from 5000)
#define OFFLINE_DATA_MAX_AGE 86400000            // Max age of data offline (24 hours in ms)

// FIXED: Real GPS Data Priority Configuration
#define GPS_DATA_PRIORITY_REAL 1                 // Highest priority - current GPS data
#define GPS_DATA_PRIORITY_LAST_KNOWN 2           // Medium priority - last known position
#define GPS_DATA_PRIORITY_SIMULATED 3            // Lowest priority - simulated data (minimal)
#define MAX_SIMULATED_RECORDS_PER_SESSION 3      // Limit simulated data per outage session
#define LAST_KNOWN_POSITION_MAX_AGE 3600000      // Max age for last known position (1 hour)

// Offline file paths
#define OFFLINE_DATA_FILE "/offline_gps.json"    // Main data file
#define OFFLINE_INDEX_FILE "/offline_index.txt"  // Index tracking file
#define OFFLINE_CONFIG_FILE "/offline_config.json" // Offline configuration

// ========================================
// KONFIGURASI PIN HARDWARE
// ========================================
// Pin komunikasi dengan modem A7670C
#define RXD2 27              // Pin RX ESP32 terhubung ke TX modem A7670C
#define TXD2 26              // Pin TX ESP32 terhubung ke RX modem A7670C
#define POWER_PIN 4          // Pin untuk power on/off modem A7670C

// Pin komunikasi dengan modul GPS
#define GPS_RX_PIN 16        // Pin RX ESP32 terhubung ke TX modul GPS
#define GPS_TX_PIN 17        // Pin TX ESP32 terhubung ke RX modul GPS

// Pin kontrol relay
#define RELAY_PIN 23         // Pin untuk mengontrol relay
#define RELAY_ON HIGH        // Logika untuk relay ON (sesuaikan dengan tipe relay)
#define RELAY_OFF LOW        // Logika untuk relay OFF (sesuaikan dengan tipe relay)

// ========================================
// GPS CONFIGURATION (UPDATED UNTUK REAL DATA PRIORITY)
// ========================================
// GPS Quality Thresholds
#define GPS_UPDATE_RATE 5              // GPS update rate in Hz (1, 5, or 10)
#define GPS_MIN_SATELLITES 4           // Minimum satellites untuk valid fix
#define GPS_MAX_HDOP 2.5              // Maximum HDOP untuk valid fix
#define GPS_BUFFER_CLEAR_INTERVAL 30000  // Interval pembersihan buffer GPS (30 detik)

// GPS High Accuracy Mode untuk Real Data
#define GPS_HIGH_ACCURACY_HDOP 1.0     // HDOP threshold for high accuracy
#define GPS_HIGH_ACCURACY_SATS 8       // Minimum satellites for high accuracy

// GPS Timing Configuration (FIXED untuk better real data collection)
#define GPS_WAIT_TIMEOUT 120000        // Increased to 120 seconds wait for GPS fix
#define GPS_LOG_INTERVAL 5000          // Log GPS status every 5 seconds
#define GPS_POSITION_UPDATE_INTERVAL 1000  // Update last known position every 1 second

// ========================================
// MOVEMENT DETECTION CONFIGURATION (UPDATED)
// ========================================
// Movement State Intervals (UPDATED for better efficiency)
#define GPS_INTERVAL_MOVING 3000          // 3 seconds when moving
#define GPS_INTERVAL_PARKED 30000         // 30 seconds when parked (CHANGED from 15000)
#define GPS_INTERVAL_STATIC 3600000       // 1 hour when static (CHANGED from 60000)

// Movement Detection Thresholds (UPDATED for noise reduction)
#define MOVEMENT_SPEED_THRESHOLD 4.0      // 4 km/h threshold for movement (CHANGED from 1.0)
#define PARKED_TO_STATIC_TIMEOUT 300000   // 5 minutes (300 seconds) to transition from PARKED to STATIC
#define MOVEMENT_DETECTION_SAMPLES 2      // Number of samples for movement detection
#define GPS_ACCURACY_THRESHOLD 5.0        // Minimum GPS accuracy required (meters)

// ========================================
// SYSTEM HEALTH MONITORING (ENHANCED)
// ========================================
#define ENABLE_AUTO_RESTART true                    // Enable auto-restart
#define AUTO_RESTART_INTERVAL 259200000             // 72 hours = 3 days (in milliseconds)
#define MEMORY_CRITICAL_THRESHOLD 15000             // 15KB minimum memory
#define MEMORY_WARNING_THRESHOLD 20000              // 20KB memory warning
#define SUCCESS_RATE_THRESHOLD 10                   // 10% minimum success rate
#define MAX_HEALTH_FAILURES 3                       // Max consecutive health failures
#define HEALTH_CHECK_INTERVAL 300000                // Check system health every 5 minutes
#define MEMORY_CHECK_INTERVAL 60000                 // Check memory every 1 minute
#define NO_TRANSMISSION_TIMEOUT 1800000             // 30 minutes without successful transmission
#define STUCK_STATE_TIMEOUT 600000                  // 10 minutes stuck in error state
#define NO_SUCCESS_TIMEOUT NO_TRANSMISSION_TIMEOUT  // Alias untuk consistency

// Recovery Thresholds
#define MAX_CONSECUTIVE_FAILURES 10                 // Max consecutive transmission failures
#define MODEM_ERROR_TIMEOUT 300000                  // 5 minutes in modem error state
#define WEBSOCKET_DISCONNECT_TIMEOUT 600000         // 10 minutes WebSocket disconnected

// ========================================
// IDENTIFIKASI PERANGKAT
// ========================================
#define GPS_ID "2d7a9833-872f-4523-b0e4-c36734940a6f"  // UUID unik untuk perangkat ini

// ========================================
// KONFIGURASI JARINGAN (OPTIMIZED FOR TESTING)
// ========================================
// Konfigurasi APN untuk koneksi GPRS
#define APN ""               // Kosongkan untuk auto-detect APN

// WebSocket Configuration (Optimized for testing realtime)
#define WS_URL "ws://70.153.193.19/websocket"  // URL server WebSocket
#define WS_PING_INTERVAL 20000                   // 20 seconds keepalive
#define WS_RECONNECT_DELAY 1000                  // 1 second for faster recovery
#define WS_CONNECT_TIMEOUT 8000                  // FIXED: Increased connection timeout (was 5000)
#define WS_RESPONSE_TIMEOUT 5000                 // FIXED: Increased response timeout (was 3000)
#define WS_KEEP_ALIVE_INTERVAL 15000             // Keep connection warm (15 detik)
#define WS_SUBSCRIPTION_TIMEOUT 8000             // FIXED: Increased wait time (was 5000)

// ========================================
// PERFORMANCE MONITORING (FIXED)
// ========================================
#define ENABLE_LATENCY_MONITORING true           // Track transmission latency
#define LATENCY_SAMPLE_SIZE 10                   // Number of latency samples to keep
#define MAX_ACCEPTABLE_LATENCY 3000              // FIXED: More realistic untuk real GPS data (was 2000)
#define LATENCY_WARNING_THRESHOLD 2000           // Warning threshold (ms)

// Connection Health Monitoring
#define CONNECTION_HEALTH_CHECK_INTERVAL 30000   // Check connection every 30s
#define SIGNAL_QUALITY_CHECK_INTERVAL 15000      // Check signal every 15s
#define AUTO_OPTIMIZATION_INTERVAL 300000        // Re-optimize every 5 minutes

// ========================================
// KONFIGURASI TIMING SISTEM (TESTING MODE)
// ========================================
#define WATCHDOG_TIMEOUT 120000          // Timeout watchdog timer (2 menit)

// FIXED: Adjusted timeouts untuk real GPS data processing
#define MODEM_AT_TIMEOUT 3000            // FIXED: Increased AT command timeout (was 2000)
#define NETWORK_CONNECT_TIMEOUT 12000    // FIXED: Increased network timeout (was 8000)
#define GPRS_CONNECT_TIMEOUT 10000       // FIXED: Increased GPRS timeout (was 6000)
#define SIM_CHECK_TIMEOUT 5000           // FIXED: Increased SIM check timeout (was 3000)

// ========================================
// KONFIGURASI KOMUNIKASI SERIAL
// ========================================
#define MODEM_BAUD_RATE 115200          // Baud rate untuk komunikasi dengan modem
#define SERIAL_RX_BUFFER_SIZE 2048      // Increased RX buffer for better performance
#define SERIAL_TX_BUFFER_SIZE 1024      // Increased TX buffer for better performance

// FIXED: GPS serial configuration untuk real data
#define GPS_BAUD_RATE 9600              // Standard GPS baud rate
#define GPS_SERIAL_RX_BUFFER_SIZE 2048  // FIXED: Increased GPS RX buffer
#define GPS_SERIAL_TX_BUFFER_SIZE 512   // GPS TX buffer

// ========================================
// PENGATURAN RETRY/PERCOBAAN ULANG (FIXED)
// ========================================
#define MAX_RESET_RETRIES 3              // FIXED: Increased for better recovery (was 2)
#define MAX_CONNECTION_FAILURES 7        // FIXED: Increased for testing real GPS (was 5)
#define RETRY_DELAY_BASE 1500            // FIXED: Increased base delay (was 1000)
#define RETRY_DELAY_MULTIPLIER 1.5       // Exponential backoff multiplier

// ========================================
// NAMA MODUL UNTUK LOGGING
// ========================================
#define MODULE_MAIN "MAIN"               // Modul utama
#define MODULE_GPS "GPS"                 // Modul GPS
#define MODULE_MODEM "MODEM"             // Modul GSM/GPRS
#define MODULE_RELAY "RELAY"             // Modul relay
#define MODULE_SYS "SYS"                 // Modul sistem
#define MODULE_WS "WEBSOCKET"            // Modul WebSocket
#define MODULE_WEBSOCKET "WEBSOCKET"     // FIXED: Alias untuk consistency
#define MODULE_PERF "PERF"               // Modul performance monitoring
#define MODULE_HEALTH "HEALTH"           // Modul system health monitoring
#define MODULE_OFFLINE "OFFLINE"         // Modul FIXED offline data management dengan real GPS

// ========================================
// PAYLOAD STRUCTURE OPTIMIZATION (FIXED UNTUK REAL GPS)
// ========================================
// Based on server endpoint structure - match exact field names for compatibility
#define SERVER_FIELD_LATITUDE "latitude"                // Server expects: latitude (string)
#define SERVER_FIELD_LONGITUDE "longitude"              // Server expects: longitude (string)
#define SERVER_FIELD_SPEED "speed"                      // Server expects: speed (number)
#define SERVER_FIELD_BATTERY_LEVEL "battery_level"      // Server expects: battery_level (number)
#define SERVER_FIELD_SATELLITES "satellites_used"       // Server expects: satellites_used (number)
#define SERVER_FIELD_TIMESTAMP "timestamp"              // Server expects: timestamp (ISO string)
#define SERVER_FIELD_GPS_ID "gps_id"                    // Server expects: gps_id (string)

// Optional fields (can be null for optimization)
#define SERVER_FIELD_RPM "rpm"                          // Server allows: null
#define SERVER_FIELD_FUEL_LEVEL "fuel_level"            // Server allows: null
#define SERVER_FIELD_IGNITION_STATUS "ignition_status"  // Server allows: null

// FIXED: Enhanced payload fields untuk real GPS data
#define SERVER_FIELD_HDOP "hdop"                        // GPS HDOP value
#define SERVER_FIELD_FIX_QUALITY "fix_quality"          // GPS fix quality
#define SERVER_FIELD_ALTITUDE "altitude"                // GPS altitude
#define SERVER_FIELD_HEADING "heading"                  // GPS course/heading
#define SERVER_FIELD_DATA_SOURCE "data_source"          // Data source: "real_gps", "last_known", "simulated"

// Payload optimization modes
#define PAYLOAD_MODE_FULL 0                             // Include all fields
#define PAYLOAD_MODE_ESSENTIAL 1                        // Only required fields
#define PAYLOAD_MODE_MINIMAL 2                          // Absolute minimum fields
#define PAYLOAD_MODE_REAL_GPS 3                         // FIXED: Enhanced mode for real GPS data
#define DEFAULT_PAYLOAD_MODE PAYLOAD_MODE_REAL_GPS      // FIXED: Use enhanced mode

// Data type optimization for server compatibility (FIXED untuk real GPS)
#define LATITUDE_PRECISION 6                            // Increased precision for real GPS (was 5)
#define LONGITUDE_PRECISION 6                           // Increased precision for real GPS (was 5)
#define SPEED_PRECISION 1                               // Decimal places for speed
#define BATTERY_PRECISION 1                             // Decimal places for battery level
#define HDOP_PRECISION 1                                // HDOP precision
#define ALTITUDE_PRECISION 1                            // Altitude precision

// ========================================
// PAYLOAD SIZE OPTIMIZATION (FIXED UNTUK REAL GPS)
// ========================================
#define MAX_PAYLOAD_SIZE 384                     // FIXED: Increased from 256 to 384 bytes untuk real GPS data
#define ENABLE_PAYLOAD_COMPRESSION false         // Disable compression (server may not support)

// Expected payload sizes for different modes (UPDATED):
// ESSENTIAL: ~140 bytes
// REAL_GPS: ~280 bytes (FIXED: new enhanced mode)
// FULL: ~180 bytes  
// MINIMAL: ~90 bytes
// OFFLINE: ~180 bytes (FIXED: enhanced format dengan GPS metadata)

// ========================================
// ADVANCED OPTIMIZATION FEATURES (FIXED)
// ========================================
// Auto-optimization based on conditions
#define ENABLE_ADAPTIVE_OPTIMIZATION true        // Automatically adjust settings
#define ENABLE_SIGNAL_BASED_OPTIMIZATION true    // Optimize based on signal strength
#define ENABLE_LATENCY_BASED_OPTIMIZATION true   // Optimize based on measured latency
#define ENABLE_GPS_QUALITY_OPTIMIZATION true     // FIXED: Optimize based on GPS quality

// Performance thresholds for auto-optimization (FIXED)
#define SIGNAL_WEAK_THRESHOLD 12                 // FIXED: Adjusted signal strength threshold (was 10)
#define SIGNAL_STRONG_THRESHOLD 25               // FIXED: Adjusted strong signal threshold (was 20)
#define CONSECUTIVE_SLOW_THRESHOLD 5             // Increased for testing (was 3)
#define GPS_QUALITY_THRESHOLD 2.0               // FIXED: HDOP threshold for quality optimization

// Power vs Performance balance
#define OPTIMIZATION_MODE_PERFORMANCE 0          // Maximum performance (higher power usage)
#define OPTIMIZATION_MODE_BALANCED 1             // Balanced performance and power
#define OPTIMIZATION_MODE_POWER_SAVE 2           // Power saving (lower performance)
#define OPTIMIZATION_MODE_GPS_PRIORITY 3         // FIXED: GPS data quality priority
#define DEFAULT_OPTIMIZATION_MODE OPTIMIZATION_MODE_GPS_PRIORITY  // FIXED: Use GPS priority mode

// ========================================
// DEBUGGING AND DIAGNOSTICS (TESTING MODE FIXED)
// ========================================
#define ENABLE_PERFORMANCE_LOGGING true          // Log performance metrics
#define ENABLE_NETWORK_DIAGNOSTICS true          // Enable network diagnostic features
#define ENABLE_LATENCY_HISTOGRAM true            // Track latency distribution
#define DIAGNOSTIC_LOG_INTERVAL 30000            // Log diagnostics every 30s (was 60000)
#define ENABLE_GPS_DIAGNOSTICS true              // FIXED: Enable GPS-specific diagnostics

// Debug levels for optimization
#define DEBUG_OPTIMIZATION_VERBOSE true          // Verbose optimization logging
#define DEBUG_LATENCY_TRACKING true              // Track individual transmission latency
#define DEBUG_NETWORK_QUALITY true               // Log network quality metrics
#define DEBUG_PAYLOAD_SIZE true                  // Log payload size for optimization
#define DEBUG_WEBSOCKET_FRAMES true              // Log WebSocket frame details
#define DEBUG_GPS_DATA_SOURCE true               // FIXED: Log GPS data source (real/last_known/simulated)

// Debug mode configuration
#define DEBUG_MODE true                          // Enable debug mode for Logger

// ========================================
// TESTING HELPERS (ENHANCED UNTUK REAL GPS)
// ========================================
#define ENABLE_FORCE_SEND_COMMAND true           // Enable 'send' command
#define ENABLE_INTERVAL_OVERRIDE true            // Enable dynamic interval change
#define ENABLE_PERFORMANCE_COMMANDS true         // Enable performance testing commands
#define LOG_EVERY_GPS_UPDATE false               // Log every GPS update (verbose)
#define ENABLE_MANUAL_SPEED_TESTING true         // Enable manual speed input for testing
#define ENABLE_GPS_SOURCE_TESTING true           // FIXED: Enable GPS data source testing

// ENHANCED: FIXED Real GPS Offline Testing Commands
#define ENABLE_OFFLINE_SIMULATION_COMMANDS true  // Enable offline simulation commands
#define ENABLE_OFFLINE_DEBUG_COMMANDS true       // Enable offline debug commands
#define ENABLE_NETWORK_OUTAGE_SIMULATION true    // Enable network outage simulation
#define ENABLE_COMPREHENSIVE_OFFLINE_TESTING true // Enable comprehensive offline testing
#define ENABLE_REAL_GPS_DATA_TESTING true        // FIXED: Enable real GPS data testing
#define ENABLE_GPS_DATA_SOURCE_COMMANDS true     // FIXED: Enable data source testing commands

// ========================================
// BATTERY MONITORING (DISABLED FOR TESTING)
// ========================================
#define ENABLE_BATTERY_MONITORING false          // Disable for testing
#define BATTERY_READ_INTERVAL 60000              // Read battery every minute
#define BATTERY_VOLTAGE_DIVIDER_RATIO 2.0        // Voltage divider ratio for ADC
#define BATTERY_ADC_PIN 35                       // ADC pin for battery monitoring
#define BATTERY_CALIBRATION_FACTOR 1.0           // Calibration factor

// Battery thresholds (set very low for testing)
#define BATTERY_MIN_VOLTAGE 3.0                  // Minimum battery voltage (0%)
#define BATTERY_MAX_VOLTAGE 4.2                  // Maximum battery voltage (100%)
#define BATTERY_LOW_THRESHOLD 9.0                // Very low to prevent emergency (was 11.5)
#define BATTERY_RECOVERY_THRESHOLD 10.0          // Very low for testing (was 12.0)

// ========================================
// POWER MANAGEMENT (TESTING MODE)
// ========================================
#define ACTIVITY_TIMEOUT 3600000                 // 1 hour before sleep (was 60000)
#define ENABLE_SLEEP_MODE false                  // Disable sleep for testing
#define FORCE_CONTINUOUS_OPERATION true          // Always stay active

// ========================================
// TIMESTAMP OPTIMIZATION (FIXED UNTUK REAL GPS)
// ========================================
#define TIMESTAMP_FORMAT_ISO8601 true            // Use ISO8601 format for server compatibility
#define TIMESTAMP_INCLUDE_MILLISECONDS false     // Exclude milliseconds for smaller payload
#define TIMESTAMP_TIMEZONE_UTC true              // Always use UTC timezone
#define TIMESTAMP_FROM_GPS_PREFERRED true        // FIXED: Prefer GPS time when available

// Timestamp format macro for consistent formatting
#define TIMESTAMP_FORMAT "%04d-%02d-%02dT%02d:%02d:%02dZ"  // ISO8601 UTC format

// ========================================
// PAYLOAD CREATION HELPERS (FIXED UNTUK REAL GPS)
// ========================================
// Helper macros for creating optimized JSON payloads
#define CREATE_ESSENTIAL_PAYLOAD(lat_str, lng_str, speed_val, sat_val, timestamp_str) \
  sprintf(payload_buffer, ESSENTIAL_PAYLOAD_TEMPLATE, lat_str, lng_str, speed_val, sat_val, timestamp_str)

// FIXED: Enhanced real GPS payload creation
#define CREATE_REAL_GPS_PAYLOAD(lat_str, lng_str, speed_val, battery_val, sat_val, hdop_val, fix_quality, altitude_val, heading_val, data_source, timestamp_str) \
  sprintf(payload_buffer, REAL_GPS_PAYLOAD_TEMPLATE, lat_str, lng_str, speed_val, battery_val, sat_val, hdop_val, fix_quality, altitude_val, heading_val, data_source, timestamp_str)

#define CREATE_FULL_PAYLOAD(lat_str, lng_str, speed_val, battery_val, sat_val, timestamp_str) \
  sprintf(payload_buffer, FULL_PAYLOAD_TEMPLATE, lat_str, lng_str, speed_val, battery_val, sat_val, timestamp_str)

#define CREATE_MINIMAL_PAYLOAD(lat_str, lng_str) \
  sprintf(payload_buffer, MINIMAL_PAYLOAD_TEMPLATE, lat_str, lng_str)

// FIXED: Enhanced offline payload creation dengan GPS metadata
#define CREATE_OFFLINE_PAYLOAD(lat_str, lng_str, speed_val, sat_val, battery_val, hdop_val, fix_quality, altitude_val, heading_val, data_source, timestamp_val, timestamp_str) \
  sprintf(payload_buffer, OFFLINE_PAYLOAD_TEMPLATE, lat_str, lng_str, speed_val, sat_val, battery_val, hdop_val, fix_quality, altitude_val, heading_val, data_source, timestamp_val, timestamp_str)

// ========================================
// DATA SOURCE IDENTIFICATION MACROS
// ========================================
#define GPS_DATA_SOURCE_REAL "real_gps"          // Current real GPS data
#define GPS_DATA_SOURCE_LAST_KNOWN "last_known"  // Last known GPS position
#define GPS_DATA_SOURCE_SIMULATED "simulated"    // Simulated/fallback data
#define GPS_DATA_SOURCE_UNKNOWN "unknown"        // Unknown source

// ========================================
// VALIDATION MACROS (FIXED)
// ========================================
// Compile-time validation of configuration
#if WS_PING_INTERVAL < 10000
  #warning "WS_PING_INTERVAL too low, may cause connection issues"
#endif

#if MAX_ACCEPTABLE_LATENCY < 2000
  #warning "MAX_ACCEPTABLE_LATENCY very aggressive for real GPS data"
#endif

#if GPS_INTERVAL_MOVING < 1000
  #warning "GPS_INTERVAL_MOVING very aggressive, may impact performance"
#endif

#if MAX_PAYLOAD_SIZE < 256
  #warning "MAX_PAYLOAD_SIZE too small for real GPS data"
#endif

#if ENABLE_OFFLINE_STORAGE && OFFLINE_MAX_RECORDS > 1000
  #warning "OFFLINE_MAX_RECORDS very high, may cause memory issues"
#endif

// FIXED: Real GPS offline storage validation
#if ENABLE_OFFLINE_STORAGE && !defined(OFFLINE_DATA_FILE)
  #error "OFFLINE_DATA_FILE must be defined when ENABLE_OFFLINE_STORAGE is true"
#endif

#if ENABLE_OFFLINE_STORAGE && OFFLINE_MAX_RECORDS < 10
  #warning "OFFLINE_MAX_RECORDS too low for effective offline storage"
#endif

#if NETWORK_CHECK_INTERVAL > 10000
  #warning "NETWORK_CHECK_INTERVAL too high, may miss network restore events"
#endif

#if OFFLINE_RECORD_SIZE < 200
  #warning "OFFLINE_RECORD_SIZE too small for real GPS metadata"
#endif

// ========================================
// FIXED OPTIMIZED PAYLOAD TEMPLATES UNTUK REAL GPS DATA
// ========================================
// Essential payload template (minimal required fields)
#define ESSENTIAL_PAYLOAD_TEMPLATE "{\
\"" SERVER_FIELD_LATITUDE "\":\"%s\",\
\"" SERVER_FIELD_LONGITUDE "\":\"%s\",\
\"" SERVER_FIELD_SPEED "\":%d,\
\"" SERVER_FIELD_SATELLITES "\":%d,\
\"" SERVER_FIELD_TIMESTAMP "\":\"%s\",\
\"" SERVER_FIELD_GPS_ID "\":\"" GPS_ID "\"\
}"

// FIXED: Enhanced real GPS payload template
#define REAL_GPS_PAYLOAD_TEMPLATE "{\
\"" SERVER_FIELD_LATITUDE "\":\"%s\",\
\"" SERVER_FIELD_LONGITUDE "\":\"%s\",\
\"" SERVER_FIELD_SPEED "\":%.1f,\
\"" SERVER_FIELD_BATTERY_LEVEL "\":%.1f,\
\"" SERVER_FIELD_SATELLITES "\":%d,\
\"" SERVER_FIELD_HDOP "\":%.1f,\
\"" SERVER_FIELD_FIX_QUALITY "\":%d,\
\"" SERVER_FIELD_ALTITUDE "\":%.1f,\
\"" SERVER_FIELD_HEADING "\":%.1f,\
\"" SERVER_FIELD_DATA_SOURCE "\":\"%s\",\
\"" SERVER_FIELD_TIMESTAMP "\":\"%s\",\
\"" SERVER_FIELD_GPS_ID "\":\"" GPS_ID "\"\
}"

// Full payload template (all fields including nullables)
#define FULL_PAYLOAD_TEMPLATE "{\
\"" SERVER_FIELD_LATITUDE "\":\"%s\",\
\"" SERVER_FIELD_LONGITUDE "\":\"%s\",\
\"" SERVER_FIELD_SPEED "\":%d,\
\"" SERVER_FIELD_RPM "\":null,\
\"" SERVER_FIELD_FUEL_LEVEL "\":null,\
\"" SERVER_FIELD_IGNITION_STATUS "\":null,\
\"" SERVER_FIELD_BATTERY_LEVEL "\":%.1f,\
\"" SERVER_FIELD_SATELLITES "\":%d,\
\"" SERVER_FIELD_TIMESTAMP "\":\"%s\",\
\"" SERVER_FIELD_GPS_ID "\":\"" GPS_ID "\"\
}"

// Minimal payload template (absolute minimum for testing)
#define MINIMAL_PAYLOAD_TEMPLATE "{\
\"" SERVER_FIELD_LATITUDE "\":\"%s\",\
\"" SERVER_FIELD_LONGITUDE "\":\"%s\",\
\"" SERVER_FIELD_GPS_ID "\":\"" GPS_ID "\"\
}"

// FIXED: Enhanced offline payload template dengan real GPS metadata
#define OFFLINE_PAYLOAD_TEMPLATE "{\
\"lat\":\"%s\",\
\"lng\":\"%s\",\
\"speed\":%.1f,\
\"sats\":%d,\
\"battery\":%.1f,\
\"hdop\":%.1f,\
\"fixQuality\":%d,\
\"altitude\":%.1f,\
\"heading\":%.1f,\
\"dataSource\":\"%s\",\
\"timestamp\":%lu,\
\"timestampStr\":\"%s\",\
\"gpsId\":\"" GPS_ID "\",\
\"sent\":false\
}"

// ========================================
// CATATAN PENGEMBANGAN (UPDATED v7.4 FIXED)
// ========================================
// VERSI 7.4 - COMPLETE FIXED REAL GPS OFFLINE STORAGE IMPLEMENTATION
// 
// 1. FIXED Health monitoring: Auto-restart after 72 hours, memory monitoring
// 2. FIXED Movement detection: 4 km/h threshold, 30s parked, 1h static intervals
// 3. FIXED Auto-recovery: System health checks, stuck state detection
// 4. FIXED Performance optimization: Better error handling, connection recovery
// 5. FIXED Memory management: Critical threshold monitoring, leak prevention
// 6. COMPLETE FIXED REAL GPS OFFLINE STORAGE: 
//    - PRIORITY real GPS data > last known position > minimal simulated
//    - Proper unsent record tracking dengan getUnsentRecordCount()
//    - Fixed sync logic dengan findNextUnsentRecord() dan sendBatchFromIndex()
//    - Enhanced priority sync dengan immediate network restore detection
//    - Complete simulation commands dengan REAL GPS data collection
//    - Fixed network availability detection dengan responsive intervals
//    - Robust error handling dengan retry mechanisms
//    - Complete integration dengan WebSocket sending
//    - Enhanced GPS metadata tracking (HDOP, fix quality, altitude, heading)
//    - Data source identification (real_gps/last_known/simulated)
// 
// FIXED ISSUES:
// ✅ Real GPS data priority: real > last_known > minimal_simulated
// ✅ Enhanced GPS metadata collection (HDOP, fix quality, altitude, heading)
// ✅ Data source identification dan tracking
// ✅ getOfflineRecordCount() now returns only unsent records
// ✅ Proper sync continuation dengan findNextUnsentRecord()
// ✅ Fixed batch sending dengan sendBatchFromIndex()
// ✅ Enhanced network detection dengan immediate restore triggering
// ✅ Complete state management untuk offline/online transitions
// ✅ Robust error handling dengan proper retry mechanisms
// ✅ Integration dengan WebSocket sending function
// ✅ Comprehensive simulation commands dengan REAL GPS data collection
// ✅ Complete logging dan monitoring untuk debugging
// ✅ Memory-efficient storage dengan proper cleanup
// ✅ GPS quality optimization dan adaptive behavior
// ✅ Enhanced payload templates untuk real GPS metadata
// ✅ Increased timeouts dan buffer sizes untuk stable real GPS operations

#endif // CONFIG_H