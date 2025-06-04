// Config.h

#ifndef CONFIG_H
#define CONFIG_H

#include "secrets.h" // Contains WIFI_SSID, WIFI_PASSWORD, AWS_IOT_ENDPOINT, THINGNAME, AWS_CERT_CA, AWS_CERT_CRT, AWS_CERT_PRIVATE

// ----- PIN DEFINITIONS -----
#define GPS_RX_PIN 16        // RX ESP32 connected to TX GPS module
#define GPS_TX_PIN 17        // TX ESP32 connected to RX GPS module
#define RELAY_PIN 23         // Relay control pin
#define RELAY_ON LOW         // Adjust for relay type (active-low)
#define RELAY_OFF HIGH       // Adjust for relay type

// ----- AWS IOT CONFIGURATION -----
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

// ----- TIME CONSTANTS (ms) -----
#define GPS_SEND_INTERVAL 5000          // Send GPS data every 5 seconds
#define RELAY_CHECK_INTERVAL 10000      // Check relay status every 10 seconds
#define HTTP_TIMEOUT 15000              // HTTP request timeout
#define MAX_STATE_TIME 15000            // Maximum time in a state
#define SYSTEM_STUCK_TIMEOUT 300000     // System stuck timeout (5 minutes)
#define GPS_BUFFER_CLEAR_INTERVAL 30000 // GPS buffer clearing interval
#define WATCHDOG_TIMEOUT 120000         // Watchdog timeout (2 minutes)

// ----- DEVICE CONFIGURATION -----
#define DEVICE_ID 1
#define RELAY_ID 1
#define UTC_OFFSET 7        // WIB (UTC+7)

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
#define MODULE_WIFI "WIFI"
#define MODULE_AWS "AWS"
#define MODULE_RELAY "RELAY"
#define MODULE_SYS "SYSTEM"

#endif // CONFIG_H