// Config.h - Updated for Aggressive Data Transmission
#ifndef CONFIG_H
#define CONFIG_H

// ----- PIN DEFINITIONS -----
#define RXD2 27              // RX ESP32 connected to TX A7670C module
#define TXD2 26              // TX ESP32 connected to RX A7670C module
#define POWER_PIN 4          // Power key for A7670C module
#define GPS_RX_PIN 16        // RX ESP32 connected to TX GPS module
#define GPS_TX_PIN 17        // TX ESP32 connected to RX GPS module
#define RELAY_PIN 23         // Relay control pin
#define RELAY_ON HIGH        // Adjust for relay type (LOW for active-low)
#define RELAY_OFF LOW        // Adjust for relay type

// ----- AGGRESSIVE GPS INTERVAL CONFIGURATION -----
// Reduced intervals for smoother tracking
#define GPS_SEND_INTERVAL_MOVING 500        // 0.5 detik saat bergerak (was 1000)
#define GPS_SEND_INTERVAL_STATIC 2000       // 2 detik saat diam (was 10000)
#define GPS_SEND_INTERVAL_ACCELERATING 250  // 0.25 detik saat akselerasi/deselerasi
#define GPS_SEND_INTERVAL_TURNING 300       // 0.3 detik saat berbelok

// ----- ENHANCED MOVEMENT DETECTION -----
#define MOVEMENT_SPEED_THRESHOLD 2.0        // km/h - lowered for faster detection (was 3.0)
#define MOVEMENT_ACCELERATION_THRESHOLD 1.5 // km/h/s - untuk deteksi akselerasi
#define MOVEMENT_HEADING_CHANGE_THRESHOLD 15.0 // degrees - untuk deteksi belokan
#define MOVEMENT_DETECTION_SAMPLES 3        // Increased samples for better accuracy
#define SPEED_CHANGE_DETECTION_SAMPLES 2    // Samples untuk deteksi perubahan kecepatan

// ----- GPS QUALITY THRESHOLDS -----
#define MIN_SATELLITES_FOR_SEND 4           // Minimum satellites untuk kirim data
#define MIN_HDOP_FOR_SEND 5.0              // Maximum HDOP untuk data berkualitas
#define GPS_FIX_QUALITY_THRESHOLD 1        // Minimum fix quality (1=GPS, 2=DGPS)

// ----- INTERPOLATION & SMOOTHING -----
#define ENABLE_POSITION_INTERPOLATION true  // Enable interpolasi posisi
#define POSITION_SMOOTHING_FACTOR 0.3      // Factor untuk smoothing (0-1)
#define MAX_POSITION_JUMP_METERS 50        // Max lompatan posisi yang diterima

// ----- WEBSOCKET CONFIGURATION -----
#define WS_URL "ws://vehitrack.my.id/websocket"
#define WS_PING_INTERVAL 30000              // Reduced to 30 seconds for better connection
#define WS_RECONNECT_DELAY 3000             // Faster reconnect

// ----- BUFFER CONFIGURATION -----
#define GPS_DATA_BUFFER_SIZE 10             // Buffer untuk data GPS saat offline
#define ENABLE_OFFLINE_BUFFERING true       // Enable buffering saat koneksi putus

// ----- TIME CONSTANTS (ms) -----
#define HTTP_TIMEOUT 10000                  // Reduced HTTP timeout
#define MAX_STATE_TIME 15000                // Maximum time in a state
#define SYSTEM_STUCK_TIMEOUT 300000         // System stuck timeout (5 minutes)
#define GPS_BUFFER_CLEAR_INTERVAL 30000     // GPS buffer clearing interval
#define WATCHDOG_TIMEOUT 120000             // Watchdog timeout (2 minutes)
#define GPS_UPDATE_RATE 100                 // GPS update check rate (ms)

// ----- SERVER CONFIGURATION (BACKUP HTTP) -----
#define SERVER_HOST "vehitrack.my.id"
#define GPS_ENDPOINT "/directus/items/vehicle_datas"
#define VEHICLE_DATA_ENDPOINT "/directus/items/vehicle_datas"
#define VEHICLE_ENDPOINT "/directus/items/vehicle"

// ----- DEVICE CONFIGURATION -----
#define DEVICE_ID 1
#define RELAY_ID 1
#define GPS_ID "2d7a9833-872f-4523-b0e4-c36734940a6f"
#define APN ""              // APN automatic detection
#define UTC_OFFSET 7        // WIB (UTC+7)

// ----- GPRS CONFIGURATION -----
#define GPRS_APN ""         // APN (kosong untuk auto-detect)
#define GPRS_USER ""        // Username GPRS
#define GPRS_PASS ""        // Password GPRS

// ----- SERIAL CONFIGURATION -----
#define MODEM_BAUD_RATE 115200
#define GPS_BAUD_RATE 9600

// ----- RETRY SETTINGS -----
#define MAX_RESET_RETRIES 3
#define MAX_CONNECTION_FAILURES 3
#define MAX_HTTP_RETRIES 3

// ----- MODULE NAMES FOR LOGGING -----
#define MODULE_MAIN "MAIN"
#define MODULE_GPS "GPS"
#define MODULE_MODEM "MODEM"
#define MODULE_RELAY "RELAY"
#define MODULE_VEHICLE "VEHICLE"
#define MODULE_HTTP "HTTP"
#define MODULE_SYS "SYS"
#define MODULE_WS "WEBSOCKET"

#endif // CONFIG_H