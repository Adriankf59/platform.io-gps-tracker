// OfflineDataManager.h - Enhanced Offline Data Manager dengan COMPLETE FIXES
#ifndef OFFLINE_DATA_MANAGER_H
#define OFFLINE_DATA_MANAGER_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <cstdlib>  // ADDED: For malloc/free
#include "Config.h"
#include "Logger.h"
#include "Utils.h"

// FIXED: Use Config.h values only - no redefinition
// All configuration constants are defined in Config.h
// MAX_OFFLINE_RECORDS, OFFLINE_RECORD_SIZE, etc. from Config.h

// FIXED: Cache configuration untuk performance optimization
#define UNSENT_COUNT_CACHE_VALIDITY 1000          // 1 second cache validity

// Status offline data manager
enum OfflineStatus {
  OFFLINE_STATUS_DISABLED,      
  OFFLINE_STATUS_READY,         
  OFFLINE_STATUS_STORING,       
  OFFLINE_STATUS_SENDING,       
  OFFLINE_STATUS_PRIORITY_SYNC, 
  OFFLINE_STATUS_ERROR          
};

// Struktur data GPS offline
struct OfflineGpsRecord {
  float latitude;
  float longitude;
  float speed;
  int satellites;
  float battery;
  unsigned long timestamp;      
  char timestampStr[30];        
  char gpsId[40];              
  bool sent;                   
  int sendAttempts;            
  
  OfflineGpsRecord() : latitude(0), longitude(0), speed(0), satellites(0), 
                      battery(0), timestamp(0), sent(false), sendAttempts(0) {
    memset(timestampStr, 0, sizeof(timestampStr));
    memset(gpsId, 0, sizeof(gpsId));
  }
};

// Statistik offline storage
struct OfflineStats {
  int totalRecordsStored;       
  int currentRecordsCount;      
  int recordsSentSuccessfully;  
  int recordsSendFailed;        
  unsigned long oldestRecord;   
  unsigned long newestRecord;   
  unsigned long lastStoreTime;  
  unsigned long lastSendTime;   
  size_t storageUsed;          
  size_t storageAvailable;     
  bool prioritySyncActive;     
  int prioritySyncProgress;    
  
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
    prioritySyncActive = false;
    prioritySyncProgress = 0;
  }
};

// Forward declaration for callback
typedef bool (*SendDataCallback)(float lat, float lon, float speed, 
                                int satellites, const char* timestamp, 
                                float battery);

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
  OfflineGpsRecord records[OFFLINE_MAX_RECORDS];
  int recordCount;
  int nextRecordIndex;
  
  // Sending state management
  int currentSendIndex;
  unsigned long lastSendAttempt;
  bool sendInProgress;
  bool priorityMode;
  
  // Send callback function pointer
  SendDataCallback sendDataFunction;
  
  // FIXED: Cache for performance optimization
  mutable int cachedUnsentCount;
  mutable unsigned long lastUnsentCountUpdate;
  
  // Statistics update
  void updateStats();
  void updateStorageInfo();
  
  // Helper functions
  bool isStorageFull() const;
  bool isStorageNearFull() const;
  void removeOldestRecord();
  String recordToJson(const OfflineGpsRecord& record);
  bool jsonToRecord(const String& json, OfflineGpsRecord& record);
  bool sendSingleRecord(OfflineGpsRecord& record);
  void compactRecords();
  
  // FIXED: Enhanced helper methods for proper functionality
  int findNextUnsentRecord(int startIndex);
  bool sendBatchFromIndex(int startIndex, int maxBatchSize);
  void completeSyncProcess();
  
  // FIXED: Cache management methods
  void invalidateUnsentCountCache();
  
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
  void setSendDataCallback(SendDataCallback callback) { sendDataFunction = callback; }
  
  // ===== DATA STORAGE =====
  bool storeGpsData(float lat, float lon, float speed, int satellites, 
                   const String& timestamp, float battery = 12.5);
  bool storeGpsData(const OfflineGpsRecord& record);
  
  // ===== DATA RETRIEVAL AND SENDING =====
  bool hasOfflineData() const { return recordCount > 0; }
  int getOfflineRecordCount() const { return recordCount; }
  
  // FIXED: Enhanced method to get only unsent records count dengan caching
  int getUnsentRecordCount() const;
  
  bool startSendingOfflineData(bool priorityMode = false);
  bool continueSendingOfflineData(); // FIXED: Non-blocking send continuation dengan proper logic
  bool sendNextBatch(int batchSize = 5); // FIXED: Use hardcoded default value
  void stopSending();
  bool isPrioritySyncActive() const { return priorityMode; }
  int getSyncProgress() const { return offlineStats.prioritySyncProgress; }
  
  // ===== DATA MANAGEMENT =====
  bool clearAllOfflineData();
  bool removeOldRecords(unsigned long olderThanTimestamp);
  bool removeExpiredRecords(unsigned long maxAge = 86400); // FIXED: Use hardcoded default (24 hours)
  bool removeOldestRecords(int count);
  unsigned long getOldestRecordAge() const;
  
  // ===== STATUS AND MONITORING =====
  OfflineStatus getStatus() const { return currentStatus; }
  const char* getStatusString() const;
  const OfflineStats& getStats() const { return offlineStats; }
  
  // ===== DIAGNOSTICS =====
  void printStorageInfo();
  void printOfflineRecords();
  void printStats();
  String getStorageReport();
  
  // FIXED: Enhanced diagnostics methods
  void printDetailedStatus();
  void printSyncStatistics();
  
  // ===== MAINTENANCE =====
  void performMaintenance();        
  bool validateStorage();           
  bool repairStorage();            
  
  // ===== TESTING =====
  // FIXED: Sync test method implementation
  void runSyncTest();
  
  // ===== CALLBACKS =====
  void setOnDataSentCallback(void (*callback)(int recordsSent, int recordsRemaining));
  void setOnStorageFullCallback(void (*callback)(int recordsStored));
  void setOnErrorCallback(void (*callback)(const char* error));
  void setOnSyncProgressCallback(void (*callback)(int progress, int total));
  
private:
  // Callback functions
  void (*onDataSentCallback)(int recordsSent, int recordsRemaining) = nullptr;
  void (*onStorageFullCallback)(int recordsStored) = nullptr;
  void (*onErrorCallback)(const char* error) = nullptr;
  void (*onSyncProgressCallback)(int progress, int total) = nullptr;
  
  // Helper untuk callback
  void notifyDataSent(int sent, int remaining);
  void notifyStorageFull(int stored);
  void notifyError(const char* error);
  void notifySyncProgress(int progress, int total);
};

// ===== GLOBAL FUNCTIONS =====
namespace OfflineDataUtils {
  String formatOfflineTimestamp(unsigned long unixTime);
  size_t estimateStorageSize(int recordCount);
  bool shouldStoreOffline(bool hasGpsConnection, bool hasNetworkConnection);
  bool isValidGpsData(float lat, float lon, float speed, int satellites);
  OfflineGpsRecord createGpsRecord(float lat, float lon, float speed, 
                                  int satellites, const String& timestamp, 
                                  float battery);
}

#endif // OFFLINE_DATA_MANAGER_H