// OfflineDataManager.cpp - Implementasi Manajemen Data GPS Offline
#include "OfflineDataManager.h"

// Constructor
OfflineDataManager::OfflineDataManager() 
  : currentStatus(OFFLINE_STATUS_DISABLED),
    isInitialized(false),
    isEnabled(false),
    recordCount(0),
    nextRecordIndex(0),
    currentSendIndex(0),
    lastSendAttempt(0),
    sendInProgress(false) {
  
  // Initialize arrays
  memset(records, 0, sizeof(records));
  
  // Initialize stats
  offlineStats.reset();
}

// ===== INITIALIZATION =====
bool OfflineDataManager::begin(bool enableOfflineStorage) {
  LOG_INFO(MODULE_SYS, "🗄️ Initializing Offline Data Manager...");
  
  isEnabled = enableOfflineStorage;
  
  if (!isEnabled) {
    LOG_INFO(MODULE_SYS, "Offline storage disabled by configuration");
    currentStatus = OFFLINE_STATUS_DISABLED;
    return true;
  }
  
  // Initialize SPIFFS
  if (!initializeFileSystem()) {
    LOG_ERROR(MODULE_SYS, "❌ Failed to initialize file system");
    currentStatus = OFFLINE_STATUS_ERROR;
    return false;
  }
  
  // Load existing records
  if (!loadRecordsFromFile()) {
    LOG_WARN(MODULE_SYS, "⚠️ No existing offline data or failed to load");
    // Not a critical error, continue with empty storage
  }
  
  // Update statistics
  updateStats();
  updateStorageInfo();
  
  isInitialized = true;
  currentStatus = OFFLINE_STATUS_READY;
  
  LOG_INFO(MODULE_SYS, "✅ Offline Data Manager initialized");
  LOG_INFO(MODULE_SYS, "📊 Loaded %d existing records", recordCount);
  
  if (recordCount > 0) {
    LOG_INFO(MODULE_SYS, "📅 Oldest: %lu, Newest: %lu", 
             offlineStats.oldestRecord, offlineStats.newestRecord);
  }
  
  printStorageInfo();
  
  return true;
}

void OfflineDataManager::end() {
  if (!isInitialized) return;
  
  LOG_INFO(MODULE_SYS, "🗄️ Shutting down Offline Data Manager...");
  
  // Save any pending data
  if (recordCount > 0) {
    LOG_INFO(MODULE_SYS, "💾 Saving %d records before shutdown", recordCount);
    // Final save is handled automatically by file operations
  }
  
  isInitialized = false;
  currentStatus = OFFLINE_STATUS_DISABLED;
  
  LOG_INFO(MODULE_SYS, "✅ Offline Data Manager shutdown complete");
}

// ===== FILE SYSTEM OPERATIONS =====
bool OfflineDataManager::initializeFileSystem() {
  if (!SPIFFS.begin(true)) {
    LOG_ERROR(MODULE_SYS, "❌ SPIFFS initialization failed");
    return false;
  }
  
  // Check available space
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  size_t freeBytes = totalBytes - usedBytes;
  
  LOG_INFO(MODULE_SYS, "📁 SPIFFS: %u KB total, %u KB used, %u KB free", 
           totalBytes/1024, usedBytes/1024, freeBytes/1024);
  
  // Check if we have enough space for offline storage
  size_t requiredSpace = OfflineDataUtils::estimateStorageSize(MAX_OFFLINE_RECORDS);
  if (freeBytes < requiredSpace) {
    LOG_WARN(MODULE_SYS, "⚠️ Limited space: need %u KB, have %u KB", 
             requiredSpace/1024, freeBytes/1024);
  }
  
  return true;
}

bool OfflineDataManager::loadRecordsFromFile() {
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    LOG_DEBUG(MODULE_SYS, "No existing offline data file");
    return false;
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, "r");
  if (!file) {
    LOG_ERROR(MODULE_SYS, "❌ Failed to open offline data file for reading");
    return false;
  }
  
  LOG_DEBUG(MODULE_SYS, "📖 Loading offline data from file (%u bytes)", file.size());
  
  // Read file content
  String content = file.readString();
  file.close();
  
  if (content.length() == 0) {
    LOG_DEBUG(MODULE_SYS, "Empty offline data file");
    return false;
  }
  
  // Parse JSON array
  DynamicJsonDocument doc(8192); // 8KB buffer for loading
  DeserializationError error = deserializeJson(doc, content);
  
  if (error) {
    LOG_ERROR(MODULE_SYS, "❌ JSON parse error: %s", error.c_str());
    return false;
  }
  
  if (!doc.is<JsonArray>()) {
    LOG_ERROR(MODULE_SYS, "❌ Invalid JSON format - expected array");
    return false;
  }
  
  JsonArray recordsArray = doc.as<JsonArray>();
  recordCount = 0;
  
  for (JsonObject recordObj : recordsArray) {
    if (recordCount >= MAX_OFFLINE_RECORDS) {
      LOG_WARN(MODULE_SYS, "⚠️ Too many records in file, truncating");
      break;
    }
    
    OfflineGpsRecord& record = records[recordCount];
    
    // Parse record fields
    record.latitude = recordObj["lat"] | 0.0f;
    record.longitude = recordObj["lng"] | 0.0f;
    record.speed = recordObj["speed"] | 0.0f;
    record.satellites = recordObj["sats"] | 0;
    record.battery = recordObj["battery"] | 12.5f;
    record.timestamp = recordObj["timestamp"] | 0UL;
    record.sent = recordObj["sent"] | false;
    
    // Copy strings safely
    const char* timestampStr = recordObj["timestampStr"] | "";
    const char* gpsId = recordObj["gpsId"] | GPS_ID;
    
    strncpy(record.timestampStr, timestampStr, sizeof(record.timestampStr) - 1);
    strncpy(record.gpsId, gpsId, sizeof(record.gpsId) - 1);
    
    recordCount++;
  }
  
  LOG_INFO(MODULE_SYS, "✅ Loaded %d offline records", recordCount);
  return recordCount > 0;
}

bool OfflineDataManager::saveRecordToFile(const OfflineGpsRecord& record) {
  // Add record to memory array first
  if (recordCount >= MAX_OFFLINE_RECORDS) {
    if (isStorageNearFull()) {
      LOG_WARN(MODULE_SYS, "⚠️ Storage near full, removing oldest record");
      removeOldestRecord();
    } else {
      LOG_ERROR(MODULE_SYS, "❌ Storage full, cannot save record");
      notifyStorageFull(recordCount);
      return false;
    }
  }
  
  // Add to memory
  records[recordCount] = record;
  recordCount++;
  
  // Save all records to file (simple but reliable approach)
  return updateFileAfterSend();
}

bool OfflineDataManager::updateFileAfterSend() {
  LOG_DEBUG(MODULE_SYS, "💾 Updating offline data file with %d records", recordCount);
  
  // Create JSON array
  DynamicJsonDocument doc(8192);
  JsonArray recordsArray = doc.to<JsonArray>();
  
  for (int i = 0; i < recordCount; i++) {
    JsonObject recordObj = recordsArray.createNestedObject();
    const OfflineGpsRecord& record = records[i];
    
    recordObj["lat"] = record.latitude;
    recordObj["lng"] = record.longitude;
    recordObj["speed"] = record.speed;
    recordObj["sats"] = record.satellites;
    recordObj["battery"] = record.battery;
    recordObj["timestamp"] = record.timestamp;
    recordObj["timestampStr"] = record.timestampStr;
    recordObj["gpsId"] = record.gpsId;
    recordObj["sent"] = record.sent;
  }
  
  // Write to file
  File file = SPIFFS.open(OFFLINE_DATA_FILE, "w");
  if (!file) {
    LOG_ERROR(MODULE_SYS, "❌ Failed to open offline data file for writing");
    notifyError("Failed to open file for writing");
    return false;
  }
  
  size_t bytesWritten = serializeJson(doc, file);
  file.close();
  
  if (bytesWritten == 0) {
    LOG_ERROR(MODULE_SYS, "❌ Failed to write offline data");
    notifyError("Failed to write data to file");
    return false;
  }
  
  LOG_DEBUG(MODULE_SYS, "✅ Wrote %u bytes to offline data file", bytesWritten);
  updateStorageInfo();
  
  return true;
}

// ===== DATA STORAGE =====
bool OfflineDataManager::storeGpsData(float lat, float lon, float speed, 
                                     int satellites, const String& timestamp, 
                                     float battery) {
  if (!isReady()) {
    LOG_DEBUG(MODULE_SYS, "Offline storage not ready");
    return false;
  }
  
  // Validate GPS data
  if (!OfflineDataUtils::isValidGpsData(lat, lon, speed, satellites)) {
    LOG_WARN(MODULE_SYS, "⚠️ Invalid GPS data, not storing offline");
    return false;
  }
  
  currentStatus = OFFLINE_STATUS_STORING;
  
  // Create record
  OfflineGpsRecord record;
  record.latitude = lat;
  record.longitude = lon;
  record.speed = speed;
  record.satellites = satellites;
  record.battery = battery;
  record.timestamp = millis() / 1000; // Simple timestamp
  record.sent = false;
  
  // Copy strings
  strncpy(record.timestampStr, timestamp.c_str(), sizeof(record.timestampStr) - 1);
  strncpy(record.gpsId, GPS_ID, sizeof(record.gpsId) - 1);
  
  // Save to file
  bool success = saveRecordToFile(record);
  
  if (success) {
    offlineStats.totalRecordsStored++;
    offlineStats.lastStoreTime = millis();
    updateStats();
    
    LOG_INFO(MODULE_SYS, "💾 GPS data stored offline [%d/%d]", 
             recordCount, MAX_OFFLINE_RECORDS);
    LOG_DEBUG(MODULE_SYS, "📍 Stored: %.6f, %.6f, %.1f km/h, %d sats", 
              lat, lon, speed, satellites);
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Failed to store GPS data offline");
    notifyError("Failed to store GPS data");
  }
  
  currentStatus = OFFLINE_STATUS_READY;
  return success;
}

bool OfflineDataManager::storeGpsData(const OfflineGpsRecord& record) {
  return storeGpsData(record.latitude, record.longitude, record.speed,
                     record.satellites, String(record.timestampStr), 
                     record.battery);
}

// ===== DATA SENDING =====
bool OfflineDataManager::startSendingOfflineData() {
  if (!hasOfflineData()) {
    LOG_DEBUG(MODULE_SYS, "No offline data to send");
    return true; // Success - nothing to send
  }
  
  LOG_INFO(MODULE_SYS, "📤 Starting to send %d offline records", recordCount);
  
  currentStatus = OFFLINE_STATUS_SENDING;
  currentSendIndex = 0;
  sendInProgress = true;
  
  return true;
}

bool OfflineDataManager::continueeSendingOfflineData() {
  if (!sendInProgress || currentStatus != OFFLINE_STATUS_SENDING) {
    return false; // Nothing to continue
  }
  
  // Rate limiting - don't send too frequently
  if (millis() - lastSendAttempt < OFFLINE_SEND_INTERVAL) {
    return true; // Still in progress, just waiting
  }
  
  // Send next batch
  bool batchSent = sendNextBatch(OFFLINE_BATCH_SEND_SIZE);
  lastSendAttempt = millis();
  
  if (!batchSent) {
    LOG_WARN(MODULE_SYS, "⚠️ Failed to send batch, will retry");
    return true; // Still in progress, will retry
  }
  
  // Check if all data sent
  if (currentSendIndex >= recordCount) {
    LOG_INFO(MODULE_SYS, "✅ All offline data sent successfully");
    
    // Clear all sent records
    clearAllOfflineData();
    
    sendInProgress = false;
    currentStatus = OFFLINE_STATUS_READY;
    
    notifyDataSent(recordCount, 0);
    
    return false; // Completed
  }
  
  return true; // Still in progress
}

bool OfflineDataManager::sendNextBatch(int batchSize) {
  // This function should be implemented to integrate with your WebSocket sending
  // For now, it's a placeholder that simulates sending
  
  LOG_DEBUG(MODULE_SYS, "📤 Sending batch starting from index %d", currentSendIndex);
  
  int sent = 0;
  int maxSend = min(batchSize, recordCount - currentSendIndex);
  
  for (int i = 0; i < maxSend; i++) {
    int recordIndex = currentSendIndex + i;
    if (recordIndex >= recordCount) break;
    
    OfflineGpsRecord& record = records[recordIndex];
    
    if (record.sent) {
      sent++;
      continue; // Skip already sent records
    }
    
    // TODO: Integrate with your WebSocket sending function
    // bool success = wsManager.sendVehicleData(
    //     record.latitude, record.longitude, record.speed,
    //     record.satellites, String(record.timestampStr), record.battery);
    
    // For now, simulate successful sending
    bool success = true; // Replace with actual sending code
    
    if (success) {
      record.sent = true;
      sent++;
      offlineStats.recordsSentSuccessfully++;
      
      LOG_DEBUG(MODULE_SYS, "✅ Sent offline record %d: %.6f, %.6f", 
                recordIndex, record.latitude, record.longitude);
    } else {
      offlineStats.recordsSendFailed++;
      LOG_WARN(MODULE_SYS, "❌ Failed to send offline record %d", recordIndex);
      break; // Stop batch on first failure
    }
    
    // Small delay between records to avoid overwhelming the connection
    Utils::safeDelay(200);
  }
  
  currentSendIndex += sent;
  offlineStats.lastSendTime = millis();
  
  LOG_INFO(MODULE_SYS, "📤 Sent %d records, %d remaining", 
           sent, recordCount - currentSendIndex);
  
  notifyDataSent(sent, recordCount - currentSendIndex);
  
  return sent > 0;
}

void OfflineDataManager::stopSending() {
  if (sendInProgress) {
    LOG_INFO(MODULE_SYS, "⏹️ Stopping offline data transmission");
    sendInProgress = false;
    currentStatus = OFFLINE_STATUS_READY;
  }
}

// ===== DATA MANAGEMENT =====
bool OfflineDataManager::clearAllOfflineData() {
  LOG_INFO(MODULE_SYS, "🗑️ Clearing all offline data (%d records)", recordCount);
  
  recordCount = 0;
  currentSendIndex = 0;
  memset(records, 0, sizeof(records));
  
  // Delete file
  if (SPIFFS.exists(OFFLINE_DATA_FILE)) {
    if (SPIFFS.remove(OFFLINE_DATA_FILE)) {
      LOG_DEBUG(MODULE_SYS, "✅ Offline data file deleted");
    } else {
      LOG_WARN(MODULE_SYS, "⚠️ Failed to delete offline data file");
    }
  }
  
  // Reset stats
  offlineStats.currentRecordsCount = 0;
  offlineStats.oldestRecord = 0;
  offlineStats.newestRecord = 0;
  updateStorageInfo();
  
  return true;
}

bool OfflineDataManager::removeOldRecords(unsigned long olderThanTimestamp) {
  int removedCount = 0;
  
  // Shift records, removing old ones
  for (int i = 0; i < recordCount; i++) {
    if (records[i].timestamp < olderThanTimestamp) {
      removedCount++;
    } else if (removedCount > 0) {
      // Shift record down
      records[i - removedCount] = records[i];
    }
  }
  
  recordCount -= removedCount;
  
  if (removedCount > 0) {
    LOG_INFO(MODULE_SYS, "🗑️ Removed %d old records", removedCount);
    updateFileAfterSend();
    updateStats();
  }
  
  return removedCount > 0;
}

void OfflineDataManager::removeOldestRecord() {
  if (recordCount > 0) {
    // Shift all records down
    for (int i = 1; i < recordCount; i++) {
      records[i - 1] = records[i];
    }
    recordCount--;
    LOG_DEBUG(MODULE_SYS, "🗑️ Removed oldest record, %d remaining", recordCount);
  }
}

// ===== STATUS AND MONITORING =====
const char* OfflineDataManager::getStatusString() const {
  switch (currentStatus) {
    case OFFLINE_STATUS_DISABLED: return "DISABLED";
    case OFFLINE_STATUS_READY: return "READY";
    case OFFLINE_STATUS_STORING: return "STORING";
    case OFFLINE_STATUS_SENDING: return "SENDING";
    case OFFLINE_STATUS_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void OfflineDataManager::updateStats() {
  offlineStats.currentRecordsCount = recordCount;
  
  if (recordCount > 0) {
    // Find oldest and newest timestamps
    offlineStats.oldestRecord = records[0].timestamp;
    offlineStats.newestRecord = records[0].timestamp;
    
    for (int i = 1; i < recordCount; i++) {
      if (records[i].timestamp < offlineStats.oldestRecord) {
        offlineStats.oldestRecord = records[i].timestamp;
      }
      if (records[i].timestamp > offlineStats.newestRecord) {
        offlineStats.newestRecord = records[i].timestamp;
      }
    }
  }
}

void OfflineDataManager::updateStorageInfo() {
  if (!isInitialized) return;
  
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  
  offlineStats.storageUsed = usedBytes;
  offlineStats.storageAvailable = totalBytes - usedBytes;
}

bool OfflineDataManager::isStorageFull() const {
  return recordCount >= MAX_OFFLINE_RECORDS;
}

bool OfflineDataManager::isStorageNearFull() const {
  return recordCount >= (MAX_OFFLINE_RECORDS - OFFLINE_STORAGE_WARNING);
}

// ===== DIAGNOSTICS =====
void OfflineDataManager::printStorageInfo() {
  updateStorageInfo();
  
  LOG_INFO(MODULE_SYS, "=== OFFLINE STORAGE INFO ===");
  LOG_INFO(MODULE_SYS, "Status       : %s", getStatusString());
  LOG_INFO(MODULE_SYS, "Records      : %d/%d", recordCount, MAX_OFFLINE_RECORDS);
  LOG_INFO(MODULE_SYS, "Storage Used : %u KB", offlineStats.storageUsed / 1024);
  LOG_INFO(MODULE_SYS, "Storage Free : %u KB", offlineStats.storageAvailable / 1024);
  
  if (recordCount > 0) {
    LOG_INFO(MODULE_SYS, "Oldest Record: %lu", offlineStats.oldestRecord);
    LOG_INFO(MODULE_SYS, "Newest Record: %lu", offlineStats.newestRecord);
  }
  
  LOG_INFO(MODULE_SYS, "============================");
}

void OfflineDataManager::printStats() {
  LOG_INFO(MODULE_SYS, "=== OFFLINE DATA STATISTICS ===");
  LOG_INFO(MODULE_SYS, "Total Stored    : %d", offlineStats.totalRecordsStored);
  LOG_INFO(MODULE_SYS, "Currently Stored: %d", offlineStats.currentRecordsCount);
  LOG_INFO(MODULE_SYS, "Successfully Sent: %d", offlineStats.recordsSentSuccessfully);
  LOG_INFO(MODULE_SYS, "Send Failed     : %d", offlineStats.recordsSendFailed);
  
  if (offlineStats.totalRecordsStored > 0) {
    float successRate = (offlineStats.recordsSentSuccessfully * 100.0f) / 
                       offlineStats.totalRecordsStored;
    LOG_INFO(MODULE_SYS, "Success Rate    : %.1f%%", successRate);
  }
  
  if (offlineStats.lastStoreTime > 0) {
    LOG_INFO(MODULE_SYS, "Last Store      : %lu ms ago", millis() - offlineStats.lastStoreTime);
  }
  
  if (offlineStats.lastSendTime > 0) {
    LOG_INFO(MODULE_SYS, "Last Send       : %lu ms ago", millis() - offlineStats.lastSendTime);
  }
  
  LOG_INFO(MODULE_SYS, "===============================");
}

void OfflineDataManager::printOfflineRecords() {
  if (recordCount == 0) {
    LOG_INFO(MODULE_SYS, "No offline records stored");
    return;
  }
  
  LOG_INFO(MODULE_SYS, "=== OFFLINE RECORDS (%d) ===", recordCount);
  
  for (int i = 0; i < min(recordCount, 10); i++) { // Show max 10 records
    const OfflineGpsRecord& record = records[i];
    LOG_INFO(MODULE_SYS, "[%d] %.6f, %.6f | %.1f km/h | %d sats | %s%s", 
             i, record.latitude, record.longitude, record.speed, 
             record.satellites, record.timestampStr,
             record.sent ? " [SENT]" : "");
  }
  
  if (recordCount > 10) {
    LOG_INFO(MODULE_SYS, "... and %d more records", recordCount - 10);
  }
  
  LOG_INFO(MODULE_SYS, "============================");
}

String OfflineDataManager::getStorageReport() {
  updateStats();
  updateStorageInfo();
  
  String report = "=== OFFLINE STORAGE REPORT ===\n";
  report += "Status: " + String(getStatusString()) + "\n";
  report += "Records: " + String(recordCount) + "/" + String(MAX_OFFLINE_RECORDS) + "\n";
  report += "Storage: " + String(offlineStats.storageUsed/1024) + " KB used, " + 
            String(offlineStats.storageAvailable/1024) + " KB free\n";
  
  if (recordCount > 0) {
    report += "Age Range: " + String((millis()/1000) - offlineStats.oldestRecord) + 
              " to " + String((millis()/1000) - offlineStats.newestRecord) + " seconds\n";
  }
  
  if (sendInProgress) {
    report += "Sending: " + String(currentSendIndex) + "/" + String(recordCount) + " sent\n";
  }
  
  return report;
}

// ===== MAINTENANCE =====
void OfflineDataManager::performMaintenance() {
  LOG_DEBUG(MODULE_SYS, "🔧 Performing offline storage maintenance");
  
  // Remove very old records (older than 24 hours)
  unsigned long cutoffTime = (millis() / 1000) - 86400; // 24 hours ago
  removeOldRecords(cutoffTime);
  
  // Update statistics
  updateStats();
  updateStorageInfo();
  
  // Check storage health
  if (isStorageNearFull()) {
    LOG_WARN(MODULE_SYS, "⚠️ Offline storage near full, consider increasing send frequency");
  }
}

bool OfflineDataManager::validateStorage() {
  if (!isInitialized) return false;
  
  // Check if file exists and is readable
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    return true; // No file is valid state
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, "r");
  if (!file) {
    LOG_ERROR(MODULE_SYS, "❌ Cannot open offline data file for validation");
    return false;
  }
  
  size_t fileSize = file.size();
  file.close();
  
  // Basic size validation
  if (fileSize == 0) {
    return true; // Empty file is valid
  }
  
  if (fileSize > (MAX_OFFLINE_RECORDS * OFFLINE_RECORD_SIZE * 2)) {
    LOG_WARN(MODULE_SYS, "⚠️ Offline data file unexpectedly large: %u bytes", fileSize);
    return false;
  }
  
  LOG_DEBUG(MODULE_SYS, "✅ Storage validation passed");
  return true;
}

bool OfflineDataManager::repairStorage() {
  LOG_WARN(MODULE_SYS, "🔧 Attempting to repair offline storage");
  
  // Backup current data in memory
  OfflineGpsRecord backupRecords[MAX_OFFLINE_RECORDS];
  int backupCount = recordCount;
  memcpy(backupRecords, records, sizeof(OfflineGpsRecord) * recordCount);
  
  // Clear everything
  clearAllOfflineData();
  
  // Try to restore from backup
  for (int i = 0; i < backupCount; i++) {
    if (OfflineDataUtils::isValidGpsData(backupRecords[i].latitude, 
                                        backupRecords[i].longitude,
                                        backupRecords[i].speed,
                                        backupRecords[i].satellites)) {
      records[recordCount] = backupRecords[i];
      recordCount++;
    }
  }
  
  // Save repaired data
  bool success = updateFileAfterSend();
  
  if (success) {
    LOG_INFO(MODULE_SYS, "✅ Storage repaired, recovered %d/%d records", 
             recordCount, backupCount);
  } else {
    LOG_ERROR(MODULE_SYS, "❌ Storage repair failed");
  }
  
  return success;
}

// ===== CALLBACKS =====
void OfflineDataManager::setOnDataSentCallback(void (*callback)(int recordsSent, int recordsRemaining)) {
  onDataSentCallback = callback;
}

void OfflineDataManager::setOnStorageFullCallback(void (*callback)(int recordsStored)) {
  onStorageFullCallback = callback;
}

void OfflineDataManager::setOnErrorCallback(void (*callback)(const char* error)) {
  onErrorCallback = callback;
}

void OfflineDataManager::notifyDataSent(int sent, int remaining) {
  if (onDataSentCallback) {
    onDataSentCallback(sent, remaining);
  }
}

void OfflineDataManager::notifyStorageFull(int stored) {
  if (onStorageFullCallback) {
    onStorageFullCallback(stored);
  }
}

void OfflineDataManager::notifyError(const char* error) {
  if (onErrorCallback) {
    onErrorCallback(error);
  }
}

// ===== UTILITY FUNCTIONS =====
namespace OfflineDataUtils {
  
  String formatOfflineTimestamp(unsigned long unixTime) {
    // Simple timestamp formatting for offline storage
    char buffer[30];
    time_t rawTime = unixTime;
    struct tm *timeInfo = gmtime(&rawTime);
    
    sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02dZ",
            timeInfo->tm_year + 1900, 
            timeInfo->tm_mon + 1, 
            timeInfo->tm_mday,
            timeInfo->tm_hour, 
            timeInfo->tm_min, 
            timeInfo->tm_sec);
    
    return String(buffer);
  }
  
  size_t estimateStorageSize(int recordCount) {
    return recordCount * OFFLINE_RECORD_SIZE + 1024; // Add some overhead
  }
  
  bool shouldStoreOffline(bool hasGpsConnection, bool hasNetworkConnection) {
    // Store offline if GPS is available but network is not
    return hasGpsConnection && !hasNetworkConnection;
  }
  
  bool isValidGpsData(float lat, float lon, float speed, int satellites) {
    // Basic GPS data validation
    if (lat < -90.0 || lat > 90.0) return false;
    if (lon < -180.0 || lon > 180.0) return false;
    if (speed < 0.0 || speed > 300.0) return false; // 300 km/h max reasonable speed
    if (satellites < 0 || satellites > 50) return false;
    
    // Check for obviously invalid coordinates (0,0 unless actually there)
    if (lat == 0.0 && lon == 0.0) return false;
    
    return true;
  }
  
  OfflineGpsRecord createGpsRecord(float lat, float lon, float speed, 
                                  int satellites, const String& timestamp, 
                                  float battery) {
    OfflineGpsRecord record;
    
    record.latitude = lat;
    record.longitude = lon;
    record.speed = speed;
    record.satellites = satellites;
    record.battery = battery;
    record.timestamp = millis() / 1000;
    record.sent = false;
    
    strncpy(record.timestampStr, timestamp.c_str(), sizeof(record.timestampStr) - 1);
    strncpy(record.gpsId, GPS_ID, sizeof(record.gpsId) - 1);
    
    return record;
  }
}
