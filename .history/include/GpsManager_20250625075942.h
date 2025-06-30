// GpsManager.h - Manajer Modul GPS untuk ESP32 Tracker
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

class GpsManager {
private:
  TinyGPSPlus& gps;
  HardwareSerial& serialGPS;
  
  // Timing
  unsigned long lastBufferClearTime;
  unsigned long lastFixTime;
  unsigned long fixLostTime;
  unsigned long lastPositionTime;
  
  // Status tracking
  bool lastFixValid;
  bool newFixFlag;
  float currentHDOP;
  
  // Data posisi terakhir
  float lastLatitude;
  float lastLongitude;
  float distanceFromLastPosition;
  
  // Filter posisi sederhana (median dari 3 sample)
  float positionFilter[3][2]; // 3 samples dari lat/lon
  int filterIndex;
  bool filterReady;
  
  // Helper methods
  void sendCommand(const char* cmd);
  void applyPositionFilter();
  float calculateDistance(float lat1, float lon1, float lat2, float lon2);
  
public:
  GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial);
  
  // ===== FUNGSI UTAMA =====
  void begin();              // Inisialisasi modul GPS
  void update();             // Update data GPS (panggil di loop)
  void clearBuffer();        // Bersihkan buffer serial GPS
  
  // ===== STATUS GPS =====
  bool isValid() const;      // Cek apakah GPS fix valid (dengan HDOP check)
  bool hasNewFix();          // Cek dan reset flag fix baru
  
  // ===== DATA POSISI =====
  float getLatitude() const;  // Latitude dengan filter
  float getLongitude() const; // Longitude dengan filter
  
  // ===== DATA PERGERAKAN =====
  float getSpeed() const { return gps.speed.isValid() ? gps.speed.kmph() : 0.0; }
  float getHeading() const { return gps.course.isValid() ? gps.course.deg() : 0.0; }
  float getAltitude() const { return gps.altitude.isValid() ? gps.altitude.meters() : 0.0; }
  
  // ===== KUALITAS SINYAL =====
  int getSatellites() const;
  float getHDOP() const { return currentHDOP; }
  bool isHighAccuracy() const { return currentHDOP < 1.0 && getSatellites() >= 8; }
  
  // ===== TIMESTAMP =====
  void getTimestamp(char* timestampStr, size_t maxLen);
  bool hasValidTime() const { return gps.date.isValid() && gps.time.isValid(); }
  
  // ===== DETEKSI PERGERAKAN =====
  float getDistanceFromLastPosition() const { return distanceFromLastPosition; }
  unsigned long getTimeSinceLastMovement() const;
  bool isMoving(float speedThreshold = 3.0) const { return getSpeed() > speedThreshold; }
  
  // ===== KONFIGURASI OPSIONAL =====
  void enableHighUpdateRate(int hz = 10);  // Set update rate GPS (1, 5, atau 10 Hz)
  
  // ===== RAW DATA (untuk debugging) =====
  float getRawLatitude() const { return gps.location.lat(); }
  float getRawLongitude() const { return gps.location.lng(); }
  
  // Untuk unit testing
  friend class GpsManagerTest;
};

#endif // GPS_MANAGER_H