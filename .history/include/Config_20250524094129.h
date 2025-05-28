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
const unsigned long GPS_SEND_INTERVAL = 5000;          // Send GPS data every 5 seconds
const unsigned long RELAY_CHECK_INTERVAL = 10000;      // Check relay status every 10 seconds
const unsigned long HTTP_TIMEOUT = 15000;              // HTTP request timeout
const unsigned long MAX_STATE_TIME = 15000;            // Maximum time in a state
const unsigned long SYSTEM_STUCK_TIMEOUT = 300000;     // System stuck timeout (5 minutes)
const unsigned long GPS_BUFFER_CLEAR_INTERVAL = 30000; // GPS buffer clearing interval
const unsigned long WATCHDOG_TIMEOUT = 120000;         // Watchdog timeout (2 minutes)

// ----- SERVER CONFIGURATION -----
const char* SERVER_HOST = "ec2-13-239-62-109.ap-southeast-2.compute.amazonaws.com";
const int SERVER_PORT = 80;
const char* GPS_ENDPOINT = "/items/koordinat_kendaraan";
const char* RELAY_ENDPOINT = "/items/relays";
const int DEVICE_ID = 1;
const int RELAY_ID = 1;
const char APN[] = "";      // APN automatic detection
const int UTC_OFFSET = 7;   // WIB (UTC+7)

// ----- RETRY SETTINGS -----
const int MAX_RESET_RETRIES = 3;
const int MAX_CONNECTION_FAILURES = 3;
const int MAX_HTTP_RETRIES = 3;

// ----- MODULE NAMES FOR LOGGING -----
#define MODULE_MAIN "MAIN"
#define MODULE_GPS "GPS"
#define MODULE_MODEM "MODEM"
#define MODULE_RELAY "RELAY"
#define MODULE_HTTP "HTTP"
#define MODULE_SYS "SYSTEM"

#endif // CONFIG_H