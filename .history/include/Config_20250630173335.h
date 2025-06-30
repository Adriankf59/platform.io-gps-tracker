// Config.h - Optimized Configuration for Testing Realtime GPS
// 
// TESTING MODE CHANGES:
// 1. GPS interval: 2 seconds (FULL mode) for realtime testing
// 2. Dynamic GPS: 1s moving, 3s static (was 1s/10s)
// 3. Sleep mode: DISABLED (activity timeout 1 hour)
// 4. Emergency mode: DISABLED (battery threshold 9V)
// 5. WebSocket reconnect: 1 second (was 2s)
// 6. Connection failures: 5 attempts (was 2)
// 7. Latency target: 2000ms (was 1500ms)
// 8. New commands: 'send', 'interval', 'nowait'
// 9. Enhanced movement detection: MOVING/PARKED/STATIC states
//
#ifndef CONFIG_H
#define CONFIG_H

// ========================================
// TESTING MODE CONFIGURATION
// ========================================
#define TESTING_MODE true                        // Enable testing mode features
#define TESTING_LOG_VERBOSE true                 // Verbose logging for testing

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
// GPS CONFIGURATION (UPDATED)
// ========================================
// GPS Quality Thresholds
#define GPS_UPDATE_RATE 5              // GPS update rate in Hz (1, 5, or 10)
#define GPS_MIN_SATELLITES 4           // Minimum satellites untuk valid fix
#define GPS_MAX_HDOP 2.5              // Maximum HDOP untuk valid fix
#define GPS_BUFFER_CLEAR_INTERVAL 30000  // Interval pembersihan buffer GPS (30 detik)

// GPS High Accuracy Mode
#define GPS_HIGH_ACCURACY_HDOP 1.0     // HDOP threshold for high accuracy
#define GPS_HIGH_ACCURACY_SATS 8       // Minimum satellites for high accuracy

// GPS Timing Configuration
#define GPS_WAIT_TIMEOUT 90000         // Max 90 seconds wait for GPS fix
#define GPS_LOG_INTERVAL 5000          // Log GPS status every 5 seconds

// ========================================
// MOVEMENT DETECTION CONFIGURATION (NEW)
// ========================================
// Movement State Intervals
#define GPS_INTERVAL_MOVING 3000          // 3 seconds when moving
#define GPS_INTERVAL_PARKED 15000         // 15 seconds when parked
#define GPS_INTERVAL_STATIC 60000         // 60 seconds when static

// Movement Detection Thresholds
#define MOVEMENT_SPEED_THRESHOLD 1.0      // 1 km/h threshold for movement
#define PARKED_TO_STATIC_TIMEOUT 300000   // 5 minutes (300 seconds) to transition from PARKED to STATIC
#define MOVEMENT_DETECTION_SAMPLES 2      // Number of samples for movement detection
#define GPS_ACCURACY_THRESHOLD 5.0        // Minimum GPS accuracy required (meters)

// ========================================
// IDENTIFIKASI PERANGKAT
// ========================================
#define GPS_ID "2d7a9833-872f-4523-b0e4-c36734940a6f"  // UUID unik untuk perangkat ini

// ========================================
// PAYLOAD STRUCTURE OPTIMIZATION
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

// Payload optimization modes
#define PAYLOAD_MODE_FULL 0                             // Include all fields
#define PAYLOAD_MODE_ESSENTIAL 1                        // Only required fields
#define PAYLOAD_MODE_MINIMAL 2                          // Absolute minimum fields
#define DEFAULT_PAYLOAD_MODE PAYLOAD_MODE_ESSENTIAL     // Balanced approach

// Data type optimization for server compatibility
#define LATITUDE_PRECISION 5                            // Decimal places for latitude (string)
#define LONGITUDE_PRECISION 5                           // Decimal places for longitude (string)
#define SPEED_PRECISION 1                               // Decimal places for speed
#define BATTERY_PRECISION 1                             // Decimal places for battery level

// ========================================
// KONFIGURASI JARINGAN (OPTIMIZED FOR TESTING)
// ========================================
// Konfigurasi APN untuk koneksi GPRS
#define APN ""               // Kosongkan untuk auto-detect APN

// WebSocket Configuration (Optimized for testing realtime)
#define WS_URL "ws://vehitrack.my.id/websocket"  // URL server WebSocket
#define WS_PING_INTERVAL 20000                   // 20 seconds keepalive
#define WS_RECONNECT_DELAY 1000                  // 1 second for faster recovery
#define WS_CONNECT_TIMEOUT 5000                  // Connection timeout (5 detik)
#define WS_RESPONSE_TIMEOUT 3000                 // Response timeout (3 detik)
#define WS_KEEP_ALIVE_INTERVAL 15000             // Keep connection warm (15 detik)
#define WS_SUBSCRIPTION_TIMEOUT 5000             // Wait max 5s for subscription

// ========================================
// NETWORK OPTIMIZATION SETTINGS
// ========================================
// LTE Optimization
#define ENABLE_LTE_OPTIMIZATION true             // Enable LTE-only optimizations
#define LTE_ONLY_MODE true                       // Force LTE-only mode
#define ENABLE_ALL_LTE_BANDS true                // Enable all available LTE bands
#define DISABLE_FAST_DORMANCY true               // Disable fast dormancy for always-on

// TCP/IP Optimization
#define TCP_NODELAY true                         // Disable Nagle algorithm
#define TCP_QUICKACK true                        // Enable TCP quick ACK
#define TCP_KEEPALIVE_ENABLE true                // Enable TCP keep-alive
#define TCP_KEEPALIVE_IDLE 60                    // Keep-alive idle time (seconds)
#define TCP_KEEPALIVE_INTERVAL 30                // Keep-alive probe interval
#define TCP_KEEPALIVE_PROBES 3                   // Number of keep-alive probes

// Data Compression
#define ENABLE_HEADER_COMPRESSION true           // Enable header compression
#define ENABLE_PAYLOAD_OPTIMIZATION true         // Optimize JSON payload size

// ========================================
// OPTIMIZED PAYLOAD TEMPLATES
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

// ========================================
// PERFORMANCE MONITORING
// ========================================
#define ENABLE_LATENCY_MONITORING true           // Track transmission latency
#define LATENCY_SAMPLE_SIZE 10                   // Number of latency samples to keep
#define MAX_ACCEPTABLE_LATENCY 2000              // Relaxed for testing (was 1500)
#define LATENCY_WARNING_THRESHOLD 1500           // Warning threshold (ms)

// Connection Health Monitoring
#define CONNECTION_HEALTH_CHECK_INTERVAL 30000   // Check connection every 30s
#define SIGNAL_QUALITY_CHECK_INTERVAL 15000      // Check signal every 15s
#define AUTO_OPTIMIZATION_INTERVAL 300000        // Re-optimize every 5 minutes

// ========================================
// KONFIGURASI TIMING SISTEM (TESTING MODE)
// ========================================
// GPS_BUFFER_CLEAR_INTERVAL already defined in GPS CONFIGURATION section
#define WATCHDOG_TIMEOUT 120000          // Timeout watchdog timer (2 menit)

// Faster Response Timeouts for testing
#define MODEM_AT_TIMEOUT 2000            // AT command timeout
#define NETWORK_CONNECT_TIMEOUT 8000     // network timeout
#define GPRS_CONNECT_TIMEOUT 6000        // GPRS timeout
#define SIM_CHECK_TIMEOUT 3000           // SIM check timeout

// ========================================
// KONFIGURASI KOMUNIKASI SERIAL
// ========================================
#define MODEM_BAUD_RATE 115200          // Baud rate untuk komunikasi dengan modem
#define SERIAL_RX_BUFFER_SIZE 2048      // Increased RX buffer for better performance
#define SERIAL_TX_BUFFER_SIZE 1024      // Increased TX buffer for better performance

// ========================================
// PENGATURAN RETRY/PERCOBAAN ULANG
// ========================================
#define MAX_RESET_RETRIES 2              // Reduced for faster recovery
#define MAX_CONNECTION_FAILURES 5        // Increased for testing (was 2)
#define RETRY_DELAY_BASE 1000            // Base delay for retries (1 second)
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
#define MODULE_PERF "PERF"               // Modul performance monitoring

// ========================================
// FITUR OPSIONAL (TESTING MODE)
// ========================================
// Dynamic GPS interval (AKTIF untuk responsive tracking)
#define ENABLE_DYNAMIC_GPS_INTERVAL
#ifdef ENABLE_DYNAMIC_GPS_INTERVAL
  // Legacy settings (masih digunakan untuk compatibility)
  #define GPS_SEND_INTERVAL_MOVING 1000      // 1 second when moving (legacy)
  #define GPS_SEND_INTERVAL_STATIC 3000      // 3 seconds when static (legacy)
  
  // Movement detection settings digunakan di movement detection baru
  // GPS_INTERVAL_MOVING, GPS_INTERVAL_PARKED, GPS_INTERVAL_STATIC sudah didefinisikan di atas
#endif

// HTTP fallback (DISABLED for testing WebSocket)
// #define ENABLE_HTTP_FALLBACK

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
// TIMESTAMP OPTIMIZATION
// ========================================
#define TIMESTAMP_FORMAT_ISO8601 true            // Use ISO8601 format for server compatibility
#define TIMESTAMP_INCLUDE_MILLISECONDS false     // Exclude milliseconds for smaller payload
#define TIMESTAMP_TIMEZONE_UTC true              // Always use UTC timezone

// Timestamp format macro for consistent formatting
#define TIMESTAMP_FORMAT "%04d-%02d-%02dT%02d:%02d:%02dZ"  // ISO8601 UTC format

// ========================================
// PAYLOAD SIZE OPTIMIZATION
// ========================================
#define MAX_PAYLOAD_SIZE 256                     // Reduced from 512 to 256 bytes
#define ENABLE_PAYLOAD_COMPRESSION false         // Disable compression (server may not support)

// Expected payload sizes for different modes:
// ESSENTIAL: ~140 bytes
// FULL: ~180 bytes  
// MINIMAL: ~90 bytes

// ========================================
// ADVANCED OPTIMIZATION FEATURES
// ========================================
// Auto-optimization based on conditions
#define ENABLE_ADAPTIVE_OPTIMIZATION true        // Automatically adjust settings
#define ENABLE_SIGNAL_BASED_OPTIMIZATION true    // Optimize based on signal strength
#define ENABLE_LATENCY_BASED_OPTIMIZATION true   // Optimize based on measured latency

// Performance thresholds for auto-optimization
#define SIGNAL_WEAK_THRESHOLD 10                 // Signal strength below which to optimize
#define SIGNAL_STRONG_THRESHOLD 20               // Signal strength above which to relax optimization
#define CONSECUTIVE_SLOW_THRESHOLD 5             // Increased for testing (was 3)

// Power vs Performance balance
#define OPTIMIZATION_MODE_PERFORMANCE 0          // Maximum performance (higher power usage)
#define OPTIMIZATION_MODE_BALANCED 1             // Balanced performance and power
#define OPTIMIZATION_MODE_POWER_SAVE 2           // Power saving (lower performance)
#define DEFAULT_OPTIMIZATION_MODE OPTIMIZATION_MODE_PERFORMANCE

// ========================================
// DEBUGGING AND DIAGNOSTICS (TESTING MODE)
// ========================================
#define ENABLE_PERFORMANCE_LOGGING true          // Log performance metrics
#define ENABLE_NETWORK_DIAGNOSTICS true          // Enable network diagnostic features
#define ENABLE_LATENCY_HISTOGRAM true            // Track latency distribution
#define DIAGNOSTIC_LOG_INTERVAL 30000            // Log diagnostics every 30s (was 60000)

// Debug levels for optimization
#define DEBUG_OPTIMIZATION_VERBOSE true          // Verbose optimization logging
#define DEBUG_LATENCY_TRACKING true              // Track individual transmission latency
#define DEBUG_NETWORK_QUALITY true               // Log network quality metrics
#define DEBUG_PAYLOAD_SIZE true                  // Log payload size for optimization
#define DEBUG_WEBSOCKET_FRAMES true              // Log WebSocket frame details

// Debug mode configuration
#define DEBUG_MODE true                          // Enable debug mode for Logger

// ========================================
// TESTING HELPERS
// ========================================
#define ENABLE_FORCE_SEND_COMMAND true           // Enable 'send' command
#define ENABLE_INTERVAL_OVERRIDE true            // Enable dynamic interval change
#define ENABLE_PERFORMANCE_COMMANDS true         // Enable performance testing commands
#define LOG_EVERY_GPS_UPDATE false               // Log every GPS update (verbose)
#define ENABLE_MANUAL_SPEED_TESTING true         // Enable manual speed input for testing

// ========================================
// VALIDATION MACROS
// ========================================
// Compile-time validation of configuration
#if WS_PING_INTERVAL < 10000
  #warning "WS_PING_INTERVAL too low, may cause connection issues"
#endif

#if MAX_ACCEPTABLE_LATENCY < 1000
  #warning "MAX_ACCEPTABLE_LATENCY very aggressive, may cause false alarms"
#endif

#if GPS_INTERVAL_MOVING < 1000
  #warning "GPS_INTERVAL_MOVING very aggressive, may impact performance"
#endif

#if MAX_PAYLOAD_SIZE < 128
  #warning "MAX_PAYLOAD_SIZE very small, may not fit required data"
#endif

// ========================================
// PAYLOAD CREATION HELPERS
// ========================================
// Helper macros for creating optimized JSON payloads
#define CREATE_ESSENTIAL_PAYLOAD(lat_str, lng_str, speed_val, sat_val, timestamp_str) \
  sprintf(payload_buffer, ESSENTIAL_PAYLOAD_TEMPLATE, lat_str, lng_str, speed_val, sat_val, timestamp_str)

#define CREATE_FULL_PAYLOAD(lat_str, lng_str, speed_val, battery_val, sat_val, timestamp_str) \
  sprintf(payload_buffer, FULL_PAYLOAD_TEMPLATE, lat_str, lng_str, speed_val, battery_val, sat_val, timestamp_str)

#define CREATE_MINIMAL_PAYLOAD(lat_str, lng_str) \
  sprintf(payload_buffer, MINIMAL_PAYLOAD_TEMPLATE, lat_str, lng_str)

// ========================================
// CATATAN PENGEMBANGAN (TESTING MODE)
// ========================================
// 1. Sleep mode DISABLED untuk testing realtime
// 2. Battery thresholds set sangat rendah (9V) untuk prevent emergency mode
// 3. GPS interval dikurangi untuk testing (2 detik di FULL mode)
// 4. Movement detection: MOVING (3s), PARKED (15s), STATIC (60s)
// 5. WebSocket reconnect delay dikurangi ke 1 detik
// 6. Activity timeout dinaikkan ke 1 jam
// 7. Connection failure threshold dinaikkan untuk testing
// 8. Latency target relaxed ke 2000ms untuk testing
// 9. Manual speed testing enabled untuk simulasi movement
// 
// Target untuk testing:
// - Realtime tracking dengan interval 1-3 detik (moving)
// - Smart interval adjustment based on movement state
// - No sleep interruptions
// - Fast recovery dari disconnections
// - Verbose logging untuk debugging
// - Manual speed input untuk testing tanpa perlu bergerak

#endif // CONFIG_H