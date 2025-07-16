// OfflineDataManager.h - Manajemen Data GPS Offline untuk ESP32 Tracker
#ifndef OFFLINE_DATA_MANAGER_H
#define OFFLINE_DATA_MANAGER_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// Konfigurasi offline storage
#define OFFLINE_DATA_FILE "/offline_gps.json"     // File penyimpanan data offline
#define OFFLINE_INDEX_FILE "/offline_index.txt"   // File index untuk tracking
#define MAX_OFFLINE_RECORDS 100                   // Maximum 100 records (sekitar 20KB)
#define OFFLINE_RECORD_SIZE 200                   // Estimasi ukuran per record dalam bytes
#define OFFLINE_STORAGE_WARNING 15                // Warning saat storage hampir penuh
#define OFFLINE_BATCH_SEND_SIZE 5                 // Kirim 5 records per batch
#define OFFLINE_SEND_INTERVAL 2000                // Interval antar batch (2 detik)

// Status offline data manager
enum OfflineStatus {
  OFFLINE_STATUS_DISABLED,      // Fitur disabled
  OFFLINE_STATUS_READY,         // Siap menyimpan data
  OFFLINE_STATUS_STORING,       // Sedang menyimpan data
  OFFLINE_STATUS_SENDING,       // Sedang mengirim data tersimpan
  OFFLINE_STATUS_ERROR          // Error dalam operasi
};

// Struktur data GPS offline
struct OfflineGpsRecord {
  float latitude;
  float longitude;
  float speed;
  int satellites;
  float battery;
  unsigned long timestamp;      // Unix timestamp
  char timestampStr[30];        // ISO8601 string
  char gpsId[40];              // GPS ID
  bool sent;                   // Flag apakah sudah terkirim
  
  // Constructor
  OfflineGpsRecord() : latitude(0), longitude(0), speed(0), satellites(0), 
                      battery(0), timestamp(0), sent(false) {
    memset(timestampStr, 0, sizeof(timestampStr));
    memset(gpsId, 0, sizeof(gpsId));
  }
};

// Statistik offline storage
struct OfflineStats {
  int totalRecordsStored;       // Total records yang pernah disimpan
  int currentRecordsCount;      // Records saat ini di storage
  int recordsSentSuccessfully;  // Records yang berhasil dikirim
  int recordsSendFailed;        // Records yang gagal dikirim
  unsigned long oldestRecord;   // Timestamp record tertua
  unsigned long newestRecord;   // Timestamp record terbaru
  unsigned long lastStoreTime;  // Waktu terakhir store data
  unsigned long lastSendTime;   // Waktu terakhir send data
  size_t storageUsed;          // Storage yang digunakan (bytes)
  size_t storageAvailable;     // Storage yang tersedia (bytes)
  
  void reset() {
    totalRecordsStored = 0;
    currentRecordsCount = 0;
    recordsSentSuccessfully = 0;
    recordsSendFailed = 0;
    oldestRecord = 0;
    newestRecord = 0;
    lastStoreTime = 0;
    lastSendTime = 0;
    storageUsed = 0;
    storageAvailable = 0;
  }
};

class OfflineDataManager {
private:
  OfflineStatus currentStatus;
  bool isInitialized;
  bool isEnabled;
  
  // Statistics instance
  OfflineStats offlineStats;
  
  // File system operations
  bool initializeFileSystem();
  bool saveRecordToFile(const OfflineGpsRecord& record);
  bool loadRecordsFromFile();
  bool deleteRecordFromFile(int index);
  bool updateFileAfterSend();
  bool clearAllRecords();
  
  // Internal record management
  OfflineGpsRecord records[MAX_OFFLINE_RECORDS];
  int recordCount;
  int nextRecordIndex;
  
  // Sending state management
  int currentSendIndex;
  unsigned long lastSendAttempt;
  bool sendInProgress;
  
  // Statistics update
  void updateStats();
  void updateStorageInfo();
  
  // Helper functions
  bool isStorageFull() const;
  bool isStorageNearFull() const;
  void removeOldestRecord();
  String recordToJson(const OfflineGpsRecord& record);
  bool jsonToRecord(const String& json, OfflineGpsRecord& record);
  
public:
  OfflineDataManager();
  
  // ===== INITIALIZATION =====
  bool begin(bool enableOfflineStorage = true);
  void end();
  bool isReady() const { return isInitialized && isEnabled; }
  
  // ===== CONFIGURATION =====
  void enable() { isEnabled = true; }
  void disable() { isEnabled = false; }
  bool isEnabledStatus() const { return isEnabled; }
  
  // ===== DATA STORAGE =====
  bool storeGpsData(float lat, float lon, float speed, int satellites, 
                   const String& timestamp, float battery = 12.5);
  bool storeGpsData(const OfflineGpsRecord& record);
  
  // ===== DATA RETRIEVAL AND SENDING =====
  bool hasOfflineData() const { return recordCount > 0; }
  int getOfflineRecordCount() const { return recordCount; }
  bool startSendingOfflineData();
  bool continueeSendingOfflineData(); // Non-blocking send continuation
  bool sendNextBatch(int batchSize = OFFLINE_BATCH_SEND_SIZE);
  void stopSending();
  
  // ===== DATA MANAGEMENT =====
  bool clearAllOfflineData();
  bool removeOldRecords(unsigned long olderThanTimestamp);
  bool removeOldestRecords(int count);
  
  // ===== STATUS AND MONITORING =====
  OfflineStatus getStatus() const { return currentStatus; }
  const char* getStatusString() const;
  const OfflineStats& getStats() const { return offlineStats; }
  
  // ===== DIAGNOSTICS =====
  void printStorageInfo();
  void printOfflineRecords();
  void printStats();
  String getStorageReport();
  
  // ===== MAINTENANCE =====
  void performMaintenance();        // Cleanup old records, defragment, etc.
  bool validateStorage();           // Validate storage integrity
  bool repairStorage();            // Attempt to repair corrupted storage
  
  // ===== CALLBACKS =====
  // Set callback untuk notifikasi saat data berhasil dikirim
  void setOnDataSentCallback(void (*callback)(int recordsSent, int recordsRemaining));
  void setOnStorageFullCallback(void (*callback)(int recordsStored));
  void setOnErrorCallback(void (*callback)(const char* error));
  
private:
  // Callback functions
  void (*onDataSentCallback)(int recordsSent, int recordsRemaining) = nullptr;
  void (*onStorageFullCallback)(int recordsStored) = nullptr;
  void (*onErrorCallback)(const char* error) = nullptr;
  
  // Helper untuk callback
  void notifyDataSent(int sent, int remaining);
  void notifyStorageFull(int stored);
  void notifyError(const char* error);
};

// ===== GLOBAL FUNCTIONS =====
// Utility functions untuk integration dengan sistem utama
namespace OfflineDataUtils {
  // Format timestamp untuk offline storage
  String formatOfflineTimestamp(unsigned long unixTime);
  
  // Estimate storage requirements
  size_t estimateStorageSize(int recordCount);
  
  // Check if offline storage is needed based on connection status
  bool shouldStoreOffline(bool hasGpsConnection, bool hasNetworkConnection);
  
  // Validate GPS data before storing
  bool isValidGpsData(float lat, float lon, float speed, int satellites);
  
  // Create GPS record from current system state
  OfflineGpsRecord createGpsRecord(float lat, float lon, float speed, 
                                  int satellites, const String& timestamp, 
                                  float battery);
}

#endif // OFFLINE_DATA_MANAGER_H