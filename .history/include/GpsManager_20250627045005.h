// GpsManager.h - Manajer Modul GPS untuk ESP32 Tracker (Enhanced Version)
#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// Konfigurasi GPS untuk update rate tinggi
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"

// GPS Start modes
#define PMTK_CMD_HOT_START  "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK104*37"
#define PMTK_CMD_FULL_COLD_START "$PMTK103*30"

// NMEA Output configuration
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

// SBAS Configuration
#define PMTK_ENABLE_SBAS "$PMTK301,2*2E"
#define PMTK_ENABLE_WAAS "$PMTK313,1*2E"

// Power Management
#define PMTK_STANDBY_MODE "$PMTK161,0*28"
#define PMTK_AWAKE "$PMTK010,002*2D"

// GPS Status
enum GpsFixStatus {
  GPS_FIX_NONE = 0,
  GPS_FIX_2D = 1,
  GPS_FIX_3D = 2,
  GPS_FIX_DGPS = 3
};

// GPS Quality Indicators
struct GpsQuality {
  int satellites;
  float hdop;
  float pdop;
  float vdop;
  int fixQuality;  // 0=invalid, 1=GPS, 2=DGPS, 3=PPS, 4=RTK, 5=Float RTK, 6=estimated
  bool hasAltitude;
  unsigned long lastUpdateTime;
};

class GpsManager {
private:
  TinyGPSPlus& gps;
  HardwareSerial& serialGPS;
  
  // Timing
  unsigned long lastBufferClearTime;
  unsigned long lastFixTime;
  unsigned long fixLostTime;
  unsigned long lastPositionTime;
  unsigned long moduleStartTime;
  unsigned long firstFixTime;
  
  // Status tracking
  bool lastFixValid;
  bool newFixFlag;
  bool firstFixAcquired;
  float currentHDOP;
  GpsFixStatus currentFixStatus;
  GpsQuality quality;
  
  // Data posisi terakhir
  float lastLatitude;
  float lastLongitude;
  float distanceFromLastPosition;
  
  // Filter posisi sederhana (median dari 3 sample)
  float positionFilter[3][2]; // 3 samples dari lat/lon
  int filterIndex;
  bool filterReady;
  
  // Statistics
  unsigned long totalSentences;
  unsigned long validSentences;
  unsigned long checksumErrors;
  
  // Configuration state
  bool highUpdateRateEnabled;
  int currentUpdateRate;
  bool sbasEnabled;
  
  // Helper methods
  void sendCommand(const char* cmd);
  void applyPositionFilter();
  float calculateDistance(float lat1, float lon1, float lat2, float lon2);
  void updateQualityIndicators();
  void parseNMEASentence();
  
public:
  GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial);
  
  // ===== FUNGSI UTAMA =====
  void begin();              // Inisialisasi modul GPS
  void update();             // Update data GPS (panggil di loop)
  void clearBuffer();        // Bersihkan buffer serial GPS
  
  // ===== INITIALIZATION & CONFIGURATION =====
  void performColdStart();   // Force cold start (clear all data)
  void performWarmStart();   // Warm start (time & position known)
  void performHotStart();    // Hot start (all data valid)
  void factoryReset();       // Reset to factory defaults
  
  // ===== STATUS GPS =====
  bool isValid() const;      // Cek apakah GPS fix valid (dengan HDOP check)
  bool hasNewFix();          // Cek dan reset flag fix baru
  bool isFirstFixAcquired() const { return firstFixAcquired; }
  unsigned long getTimeToFirstFix() const;
  unsigned long getTimeSinceLastFix() const;
  GpsFixStatus getFixStatus() const { return currentFixStatus; }
  const char* getFixStatusString() const;
  
  // ===== DATA POSISI =====
  float getLatitude() const;  // Latitude dengan filter
  float getLongitude() const; // Longitude dengan filter
  float getRawLatitude() const { return gps.location.lat(); }
  float getRawLongitude() const { return gps.location.lng(); }
  
  // ===== DATA PERGERAKAN =====
  float getSpeed() const { return gps.speed.isValid() ? gps.speed.kmph() : 0.0; }
  float getHeading() const { return gps.course.isValid() ? gps.course.deg() : 0.0; }
  float getAltitude() const { return gps.altitude.isValid() ? gps.altitude.meters() : 0.0; }
  
  // ===== KUALITAS SINYAL =====
  int getSatellites() const;
  float getHDOP() const { return currentHDOP; }
  float getPDOP() const { return quality.pdop; }
  float getVDOP() const { return quality.vdop; }
  int getFixQuality() const { return quality.fixQuality; }
  bool isHighAccuracy() const { return currentHDOP < 1.0 && getSatellites() >= 8; }
  bool isDGPSActive() const { return quality.fixQuality >= 2; }
  const GpsQuality& getQualityInfo() const { return quality; }
  
  // ===== TIMESTAMP =====
  void getTimestamp(char* timestampStr, size_t maxLen);
  bool hasValidTime() const { return gps.date.isValid() && gps.time.isValid(); }
  unsigned long getGpsTime() const;  // Get GPS time as Unix timestamp
  
  // ===== DETEKSI PERGERAKAN =====
  float getDistanceFromLastPosition() const { return distanceFromLastPosition; }
  unsigned long getTimeSinceLastMovement() const;
  bool isMoving(float speedThreshold = 3.0) const { return getSpeed() > speedThreshold; }
  bool hasMovedSignificantly(float distanceThreshold = 5.0) const { 
    return distanceFromLastPosition > distanceThreshold; 
  }
  
  // ===== KONFIGURASI OPSIONAL =====
  void enableHighUpdateRate(int hz = 10);  // Set update rate GPS (1, 5, atau 10 Hz)
  void setUpdateRate(int hz);              // Generic update rate setter
  void enableSBAS(bool enable = true);     // Enable/disable SBAS
  void enableWAAS(bool enable = true);     // Enable/disable WAAS
  void setNMEAOutput(const char* config);  // Custom NMEA output
  void enterStandbyMode();                 // Enter power saving mode
  void wakeUp();                           // Wake from standby
  
  // ===== STATISTICS & DIAGNOSTICS =====
  unsigned long getTotalSentences() const { return totalSentences; }
  unsigned long getValidSentences() const { return validSentences; }
  unsigned long getChecksumErrors() const { return checksumErrors; }
  float getSentenceSuccessRate() const {
    return totalSentences > 0 ? (validSentences * 100.0 / totalSentences) : 0;
  }
  void printDiagnostics();
  
  // ===== ADVANCED FEATURES =====
  bool waitForFix(unsigned long timeout);  // Blocking wait for fix
  void optimizeForColdStart();             // Optimize settings for cold start
  void optimizeForMoving();                // Optimize for moving vehicle
  void optimizeForStationary();            // Optimize for stationary use
  
  // ===== RAW ACCESS =====
  TinyGPSPlus& getRawGPS() { return gps; }
  HardwareSerial& getSerial() { return serialGPS; }
  
  // Untuk unit testing
  friend class GpsManagerTest;
};

// Helper functions
namespace GpsUtils {
  String formatDMS(float coordinate, bool isLatitude);
  float DMStoDegrees(int degrees, int minutes, float seconds);
  String getCompassDirection(float heading);
  float calculateBearing(float lat1, float lon1, float lat2, float lon2);
}

#endif // GPS_MANAGER_H