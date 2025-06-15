// Config.h - Updated for WebSocket Support
#ifndef CONFIG_H
#define CONFIG_H

// ----- PIN DEFINITIONS -----
#define RXD2 27              // RX ESP32 connected to TX A7670C module
#define TXD2 26              // TX ESP32 connected to RX A7670C module
#define POWER_PIN 4          // Power key for A7670C module
#define GPS_RX_PIN 16        // RX ESP32 connected to TX GPS module
#define GPS_TX_PIN 17        // TX ESP32 connected to RX GPS module
#define RELAY_PIN 23         // Relay control pin
#define RELAY_ON LOW         // Adjust for relay type (LOW for active-low)
#define RELAY_OFF HIGH       // Adjust for relay type

// ----- HYBRID GPS INTERVAL CONFIGURATION -----
#define GPS_SEND_INTERVAL_MOVING 1000       // 1 detik saat bergerak
#define GPS_SEND_INTERVAL_STATIC 10000      // 10 detik saat diam
#define MOVEMENT_SPEED_THRESHOLD 3.0        // km/h - threshold untuk menentukan bergerak/diam
#define MOVEMENT_DETECTION_SAMPLES 2        // Jumlah sample untuk stabilitas deteksi

// ----- WEBSOCKET CONFIGURATION -----
#define WS_URL "ws://vehitrack.my.id/websocket"
#define WS_PING_INTERVAL 45000              // 45 seconds ping interval
#define WS_RECONNECT_DELAY 5000             // Base reconnect delay

// ----- TIME CONSTANTS (ms) -----
#define HTTP_TIMEOUT 15000              // HTTP request timeout (backup)
#define MAX_STATE_TIME 15000            // Maximum time in a state
#define SYSTEM_STUCK_TIMEOUT 300000     // System stuck timeout (5 minutes)
#define GPS_BUFFER_CLEAR_INTERVAL 30000 // GPS buffer clearing interval
#define WATCHDOG_TIMEOUT 120000         // Watchdog timeout (2 minutes)

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