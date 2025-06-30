// Config.h - Payload Structure Optimization Based on Server Endpoint
#ifndef CONFIG_H
#define CONFIG_H

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
// KONFIGURASI JARINGAN (OPTIMIZED FOR LOW LATENCY)
// ========================================
// Konfigurasi APN untuk koneksi GPRS
#define APN ""               // Kosongkan untuk auto-detect APN

// WebSocket Configuration (Optimized for server endpoint)
#define WS_URL "ws://vehitrack.my.id/websocket"  // URL server WebSocket
#define WS_PING_INTERVAL 20000                   // Reduced from 45000 to 20000 (20 detik)
#define WS_RECONNECT_DELAY 2000                  // Reduced from 5000 to 2000 (2 detik)
#define WS_CONNECT_TIMEOUT 5000                  // Connection timeout (5 detik)
#define WS_RESPONSE_TIMEOUT 3000                 // Response timeout (3 detik)
#define WS_KEEP_ALIVE_INTERVAL 15000             // Keep connection warm (15 detik)

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
#define MAX_ACCEPTABLE_LATENCY 1500              // Target: <1.5 seconds (more aggressive)
#define LATENCY_WARNING_THRESHOLD 1200           // Warning threshold (ms)

// Connection Health Monitoring
#define CONNECTION_HEALTH_CHECK_INTERVAL 30000   // Check connection every 30s
#define SIGNAL_QUALITY_CHECK_INTERVAL 15000      // Check signal every 15s
#define AUTO_OPTIMIZATION_INTERVAL 300000        // Re-optimize every 5 minutes

// ========================================
// KONFIGURASI TIMING SISTEM (OPTIMIZED)
// ========================================
#define GPS_BUFFER_CLEAR_INTERVAL 30000  // Interval pembersihan buffer GPS (30 detik)
#define WATCHDOG_TIMEOUT 120000          // Timeout watchdog timer (2 menit)

// Faster Response Timeouts
#define MODEM_AT_TIMEOUT 2000            // Reduced from 3000 to 2000 (AT command timeout)
#define NETWORK_CONNECT_TIMEOUT 8000     // Reduced from 10000 to 8000 (network timeout)
#define GPRS_CONNECT_TIMEOUT 6000        // Reduced from default (GPRS timeout)
#define SIM_CHECK_TIMEOUT 3000           // Reduced from 5000 to 3000 (SIM check timeout)

// ========================================
// KONFIGURASI KOMUNIKASI SERIAL (OPTIMIZED)
// ========================================
#define MODEM_BAUD_RATE 115200          // Baud rate untuk komunikasi dengan modem
#define SERIAL_RX_BUFFER_SIZE 2048      // Increased RX buffer for better performance
#define SERIAL_TX_BUFFER_SIZE 1024      // Increased TX buffer for better performance

// ========================================
// PENGATURAN RETRY/PERCOBAAN ULANG (OPTIMIZED)
// ========================================
#define MAX_RESET_RETRIES 2              // Reduced from 3 to 2 (faster recovery)
#define MAX_CONNECTION_FAILURES 2        // Reduced from 3 to 2 (faster reset trigger)
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
// FITUR OPSIONAL (AKTIF UNTUK OPTIMASI)
// ========================================
// Dynamic GPS interval (AKTIF untuk responsive tracking)
#define ENABLE_DYNAMIC_GPS_INTERVAL
#ifdef ENABLE_DYNAMIC_GPS_INTERVAL
  #define GPS_SEND_INTERVAL_MOVING 1000      // Interval saat bergerak (1 detik)
  #define GPS_SEND_INTERVAL_STATIC 10000     // Interval saat diam (10 detik)
  #define MOVEMENT_SPEED_THRESHOLD 0.0       // Threshold kecepatan (km/jam) - OPTIMIZED
  #define MOVEMENT_DETECTION_SAMPLES 2       // Jumlah sample untuk deteksi
  #define GPS_ACCURACY_THRESHOLD 5.0         // Minimum GPS accuracy required (meters)
#endif

// HTTP fallback (OPSIONAL - untuk backup jika WebSocket gagal)
// #define ENABLE_HTTP_FALLBACK
#ifdef ENABLE_HTTP_FALLBACK
  #define MODULE_HTTP "HTTP"                      // Modul HTTP
  #define SERVER_HOST "vehitrack.my.id"           // Host server HTTP
  #define HTTP_TIMEOUT 8000                       // Reduced for faster response
  #define GPS_ENDPOINT "/directus/items/vehicle_datas"  // Endpoint untuk data GPS
  #define MAX_HTTP_RETRIES 2                      // Reduced from 3 to 2
  #define HTTP_KEEP_ALIVE true                    // Enable HTTP keep-alive
#endif

// ========================================
// BATTERY MONITORING OPTIMIZATION
// ========================================
#define ENABLE_BATTERY_MONITORING true           // Monitor battery level for payload
#define BATTERY_READ_INTERVAL 60000              // Read battery every minute
#define BATTERY_VOLTAGE_DIVIDER_RATIO 2.0        // Voltage divider ratio for ADC
#define BATTERY_ADC_PIN 35                       // ADC pin for battery monitoring
#define BATTERY_CALIBRATION_FACTOR 1.0           // Calibration factor

// Battery level calculation (based on typical Li-ion curve)
#define BATTERY_MIN_VOLTAGE 3.0                  // Minimum battery voltage (0%)
#define BATTERY_MAX_VOLTAGE 4.2                  // Maximum battery voltage (100%)

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
#define CONSECUTIVE_SLOW_THRESHOLD 3             // Number of slow transmissions before optimization

// Power vs Performance balance
#define OPTIMIZATION_MODE_PERFORMANCE 0          // Maximum performance (higher power usage)
#define OPTIMIZATION_MODE_BALANCED 1             // Balanced performance and power
#define OPTIMIZATION_MODE_POWER_SAVE 2           // Power saving (lower performance)
#define DEFAULT_OPTIMIZATION_MODE OPTIMIZATION_MODE_PERFORMANCE

// ========================================
// DEBUGGING AND DIAGNOSTICS
// ========================================
#define ENABLE_PERFORMANCE_LOGGING true          // Log performance metrics
#define ENABLE_NETWORK_DIAGNOSTICS true          // Enable network diagnostic features
#define ENABLE_LATENCY_HISTOGRAM true            // Track latency distribution
#define DIAGNOSTIC_LOG_INTERVAL 60000            // Log diagnostics every minute

// Debug levels for optimization
#define DEBUG_OPTIMIZATION_VERBOSE false         // Verbose optimization logging
#define DEBUG_LATENCY_TRACKING true              // Track individual transmission latency
#define DEBUG_NETWORK_QUALITY true               // Log network quality metrics
#define DEBUG_PAYLOAD_SIZE true                  // Log payload size for optimization

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

#if GPS_SEND_INTERVAL_MOVING < 500
  #warning "GPS_SEND_INTERVAL_MOVING very aggressive, may impact performance"
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
// CATATAN PENGEMBANGAN
// ========================================
// 1. Payload structure matches server endpoint exactly
// 2. Latitude/longitude sent as strings for server compatibility
// 3. Optional fields (rpm, fuel_level, ignition_status) set to null
// 4. Battery level included when available
// 5. Timestamp in ISO8601 format as expected by server
// 6. GPS_ID fixed to match device configuration
// 
// Expected payload sizes:
// - Essential mode: ~140 bytes (recommended for low latency)
// - Full mode: ~180 bytes (includes battery level)
// - Minimal mode: ~90 bytes (for testing only)
//
// Target latency with optimized payload: <1.5 seconds

#endif // CONFIG_H