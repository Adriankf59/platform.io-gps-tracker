// Config.h - Konfigurasi Sistem GPS Tracker
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
// KONFIGURASI JARINGAN
// ========================================
// Konfigurasi APN untuk koneksi GPRS
#define APN ""               // Kosongkan untuk auto-detect APN

// Konfigurasi WebSocket
#define WS_URL "ws://vehitrack.my.id/websocket"  // URL server WebSocket
#define WS_PING_INTERVAL 45000                   // Interval ping WebSocket (45 detik)
#define WS_RECONNECT_DELAY 5000                  // Delay dasar untuk reconnect (5 detik)

// ========================================
// KONFIGURASI TIMING SISTEM
// ========================================
#define GPS_BUFFER_CLEAR_INTERVAL 30000  // Interval pembersihan buffer GPS (30 detik)
#define WATCHDOG_TIMEOUT 120000          // Timeout watchdog timer (2 menit)

// ========================================
// KONFIGURASI KOMUNIKASI SERIAL
// ========================================
#define MODEM_BAUD_RATE 115200          // Baud rate untuk komunikasi dengan modem

// ========================================
// PENGATURAN RETRY/PERCOBAAN ULANG
// ========================================
#define MAX_RESET_RETRIES 3              // Maksimal percobaan reset modem
#define MAX_CONNECTION_FAILURES 3        // Maksimal kegagalan koneksi sebelum reset

// ========================================
// NAMA MODUL UNTUK LOGGING
// ========================================
#define MODULE_MAIN "MAIN"               // Modul utama
#define MODULE_GPS "GPS"                 // Modul GPS
#define MODULE_MODEM "MODEM"             // Modul GSM/GPRS
#define MODULE_RELAY "RELAY"             // Modul relay
#define MODULE_SYS "SYS"                 // Modul sistem
#define MODULE_WS "WEBSOCKET"            // Modul WebSocket

// ========================================
// FITUR OPSIONAL (TIDAK AKTIF)
// ========================================
// Uncomment untuk mengaktifkan fitur dynamic GPS interval
//define ENABLE_DYNAMIC_GPS_INTERVAL
#ifdef ENABLE_DYNAMIC_GPS_INTERVAL
  #define GPS_SEND_INTERVAL_MOVING 1000      // Interval saat bergerak (1 detik)
  #define GPS_SEND_INTERVAL_STATIC 10000     // Interval saat diam (10 detik)
  #define MOVEMENT_SPEED_THRESHOLD 3.0       // Threshold kecepatan (km/jam)
  #define MOVEMENT_DETECTION_SAMPLES 2       // Jumlah sample untuk deteksi
#endif

// Uncomment untuk mengaktifkan HTTP sebagai backup
// #define ENABLE_HTTP_FALLBACK
#ifdef ENABLE_HTTP_FALLBACK
  #define MODULE_HTTP "HTTP"                      // Modul HTTP
  #define SERVER_HOST "vehitrack.my.id"           // Host server HTTP
  #define HTTP_TIMEOUT 15000                      // Timeout request HTTP (15 detik)
  #define GPS_ENDPOINT "/directus/items/vehicle_datas"  // Endpoint untuk data GPS
  #define MAX_HTTP_RETRIES 3                      // Maksimal retry HTTP
#endif

// ========================================
// KONFIGURASI YANG DIHAPUS (TIDAK DIGUNAKAN)
// ========================================
// Konstanta berikut dihapus karena tidak digunakan dalam kode:
// - MAX_STATE_TIME: Tidak ada implementasi state timeout
// - SYSTEM_STUCK_TIMEOUT: Tidak ada deteksi sistem stuck
// - DEVICE_ID & RELAY_ID: Tidak direferensi dalam kode
// - UTC_OFFSET: GPS selalu menggunakan UTC
// - GPRS_USER & GPRS_PASS: Tidak diperlukan untuk koneksi
// - VEHICLE_DATA_ENDPOINT & VEHICLE_ENDPOINT: HTTP tidak aktif

// ========================================
// CATATAN PENGEMBANGAN
// ========================================
// 1. Power mode configuration saat ini hardcoded di main.cpp
//    Pertimbangkan untuk memindahkan ke sini jika diperlukan
// 
// 2. GPS update rate bisa dikonfigurasi jika modul mendukung:
//    #define GPS_UPDATE_RATE 10  // Hz
//
// 3. Untuk debugging, aktifkan dengan menambahkan di platformio.ini:
//    build_flags = -D DEBUG_MODE

#endif // CONFIG_H