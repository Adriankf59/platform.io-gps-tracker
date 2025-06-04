// Config.h

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

// ----- TIME CONSTANTS (ms) -----
#define GPS_SEND_INTERVAL 5000          // Send GPS data every 5 seconds
#define RELAY_CHECK_INTERVAL 10000      // Check relay status every 10 seconds
#define HTTP_TIMEOUT 15000              // HTTP request timeout
#define MAX_STATE_TIME 15000            // Maximum time in a state
#define SYSTEM_STUCK_TIMEOUT 300000     // System stuck timeout (5 minutes)
#define GPS_BUFFER_CLEAR_INTERVAL 30000 // GPS buffer clearing interval
#define WATCHDOG_TIMEOUT 120000         // Watchdog timeout (2 minutes)

// ----- SERVER CONFIGURATION -----
#define SERVER_HOST "ec2-13-229-83-7.ap-southeast-1.compute.amazonaws.com"
#define SERVER_PORT 8055
#define GPS_ENDPOINT "/items/vehicle_datas"
#define VEHICLE_DATA_ENDPOINT "/items/vehicle_datas"
#define RELAY_ENDPOINT "/items/commands"
#define DEVICE_ID 1
#define RELAY_ID 1
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
#define MODULE_HTTP "HTTP"
#define MODULE_SYS "SYSTEM"

// Add new
#define API_BASE_URL "http://ec2-13-229-83-7.ap-southeast-1.compute.amazonaws.com:8055"
#define GPS_ID "2d7a9833-872f-4523-b0e4-c36734940a6f"

#endif // CONFIG_H