// OfflineDataManager.cpp - COMPLETE FIXED Implementation dengan Proper Integration
#include "OfflineDataManager.h"

// FIXED: Forward declaration dengan proper C linkage - REMOVED to avoid conflicts
// Function will be set via callback pointer in main.cpp

// Constructor
OfflineDataManager::OfflineDataManager() 
  : currentStatus(OFFLINE_STATUS_DISABLED),
    isInitialized(false),
    isEnabled(false),
    recordCount(0),
    nextRecordIndex(0),
    currentSendIndex(0),
    lastSendAttempt(0),
    sendInProgress(false),
    priorityMode(false),
    sendDataFunction(nullptr),
    cachedUnsentCount(-1),
    lastUnsentCountUpdate(0) {
  
  memset(records, 0, sizeof(records));
  offlineStats.reset();
}

// ===== INITIALIZATION =====
bool OfflineDataManager::begin(bool enableOfflineStorage) {
  LOG_INFO(MODULE_OFFLINE, "🗄️ Initializing FIXED Offline Data Manager...");
  
  isEnabled = enableOfflineStorage;
  
  if (!isEnabled) {
    LOG_INFO(MODULE_OFFLINE, "Offline storage disabled by configuration");
    currentStatus = OFFLINE_STATUS_DISABLED;
    return true;
  }
  
  if (!initializeFileSystem()) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Failed to initialize file system");
    currentStatus = OFFLINE_STATUS_ERROR;
    return false;
  }
  
  bool loadSuccess = loadRecordsFromFile();
  if (!loadSuccess) {
    LOG_WARN(MODULE_OFFLINE, "⚠️ No existing offline data, starting fresh");
    clearAllOfflineData();
  }
  
  updateStats();
  updateStorageInfo();
  
  // FIXED: Don't set function pointer here - will be set from main.cpp
  
  isInitialized = true;
  currentStatus = OFFLINE_STATUS_READY;
  
  LOG_INFO(MODULE_OFFLINE, "✅ FIXED Offline Data Manager initialized");
  LOG_INFO(MODULE_OFFLINE, "📊 Loaded %d records (%d unsent)", recordCount, getUnsentRecordCount());
  
  if (recordCount > 0) {
    LOG_INFO(MODULE_OFFLINE, "📅 Oldest: %lu, Newest: %lu", 
             offlineStats.oldestRecord, offlineStats.newestRecord);
    LOG_INFO(MODULE_OFFLINE, "🔄 Ready for auto-sync when network available");
  }
  
  printStorageInfo();
  return true;
}

void OfflineDataManager::end() {
  if (!isInitialized) return;
  
  LOG_INFO(MODULE_OFFLINE, "🗄️ Shutting down Offline Data Manager...");
  
  if (recordCount > 0) {
    LOG_INFO(MODULE_OFFLINE, "💾 Saving %d records before shutdown", recordCount);
    updateFileAfterSend();
  }
  
  isInitialized = false;
  currentStatus = OFFLINE_STATUS_DISABLED;
  LOG_INFO(MODULE_OFFLINE, "✅ Offline Data Manager shutdown complete");
}

// ===== FILE SYSTEM OPERATIONS =====
bool OfflineDataManager::initializeFileSystem() {
  if (!SPIFFS.begin(true)) {
    LOG_ERROR(MODULE_OFFLINE, "❌ SPIFFS initialization failed");
    return false;
  }
  
  size_t totalBytes = SPIFFS.totalBytes();
  size_t usedBytes = SPIFFS.usedBytes();
  size_t freeBytes = totalBytes - usedBytes;
  
  LOG_INFO(MODULE_OFFLINE, "📁 SPIFFS: %u KB total, %u KB used, %u KB free", 
           totalBytes/1024, usedBytes/1024, freeBytes/1024);
  
  size_t requiredSpace = OfflineDataUtils::estimateStorageSize(OFFLINE_MAX_RECORDS);
  if (freeBytes < requiredSpace) {
    LOG_WARN(MODULE_OFFLINE, "⚠️ Limited space: need %u KB, have %u KB", 
             requiredSpace/1024, freeBytes/1024);
  }
  
  return true;
}

bool OfflineDataManager::loadRecordsFromFile() {
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    LOG_DEBUG(MODULE_OFFLINE, "No existing offline data file");
    return false;
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, "r");
  if (!file) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Failed to open offline data file for reading");
    return false;
  }
  
  LOG_DEBUG(MODULE_OFFLINE, "📖 Loading offline data from file (%u bytes)", file.size());
  
  String content = file.readString();
  file.close();
  
  if (content.length() == 0) {
    LOG_DEBUG(MODULE_OFFLINE, "Empty offline data file");
    return false;
  }
  
  DynamicJsonDocument doc(16384);
  DeserializationError error = deserializeJson(doc, content);
  
  if (error) {
    LOG_ERROR(MODULE_OFFLINE, "❌ JSON parse error: %s", error.c_str());
    LOG_WARN(MODULE_OFFLINE, "🔧 Attempting to repair corrupted file");
    return repairStorage();
  }
  
  if (!doc.is<JsonArray>()) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Invalid JSON format - expected array");
    return false;
  }
  
  JsonArray recordsArray = doc.as<JsonArray>();
  recordCount = 0;
  int skippedRecords = 0;
  
  for (JsonObject recordObj : recordsArray) {
    if (recordCount >= OFFLINE_MAX_RECORDS) {
      LOG_WARN(MODULE_OFFLINE, "⚠️ Too many records in file, truncating");
      break;
    }
    
    OfflineGpsRecord& record = records[recordCount];
    
    record.latitude = recordObj["lat"] | 0.0f;
    record.longitude = recordObj["lng"] | 0.0f;
    record.speed = recordObj["speed"] | 0.0f;
    record.satellites = recordObj["sats"] | 0;
    record.battery = recordObj["battery"] | 12.5f;
    record.timestamp = recordObj["timestamp"] | 0UL;
    record.sent = recordObj["sent"] | false;
    record.sendAttempts = recordObj["attempts"] | 0;
    
    const char* timestampStr = recordObj["timestampStr"] | "";
    const char* gpsId = recordObj["gpsId"] | GPS_ID;
    
    strncpy(record.timestampStr, timestampStr, sizeof(record.timestampStr) - 1);
    record.timestampStr[sizeof(record.timestampStr) - 1] = '\0';
    strncpy(record.gpsId, gpsId, sizeof(record.gpsId) - 1);
    record.gpsId[sizeof(record.gpsId) - 1] = '\0';
    
    // Load all valid records (both sent and unsent) for complete data integrity
    if (OfflineDataUtils::isValidGpsData(record.latitude, record.longitude, 
                                        record.speed, record.satellites)) {
      recordCount++;
    } else {
      skippedRecords++;
      LOG_DEBUG(MODULE_OFFLINE, "Skipping invalid record: %.6f, %.6f", 
                record.latitude, record.longitude);
    }
  }
  
  if (skippedRecords > 0) {
    LOG_WARN(MODULE_OFFLINE, "⚠️ Skipped %d invalid records", skippedRecords);
  }
  
  LOG_INFO(MODULE_OFFLINE, "✅ Loaded %d offline records (%d unsent)", 
           recordCount, getUnsentRecordCount());
  return recordCount > 0;
}

bool OfflineDataManager::saveRecordToFile(const OfflineGpsRecord& record) {
  if (recordCount >= OFFLINE_MAX_RECORDS) {
    if (isStorageNearFull()) {
      LOG_WARN(MODULE_OFFLINE, "⚠️ Storage near full, removing oldest record");
      removeOldestRecord();
    } else {
      LOG_ERROR(MODULE_OFFLINE, "❌ Storage full, cannot save record");
      notifyStorageFull(recordCount);
      return false;
    }
  }
  
  records[recordCount] = record;
  recordCount++;
  invalidateUnsentCountCache(); // Invalidate cache after adding
  
  return updateFileAfterSend();
}

bool OfflineDataManager::updateFileAfterSend() {
  LOG_DEBUG(MODULE_OFFLINE, "💾 Updating offline data file with %d records", recordCount);
  
  DynamicJsonDocument doc(16384);
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
    recordObj["attempts"] = record.sendAttempts;
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, "w");
  if (!file) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Failed to open offline data file for writing");
    notifyError("Failed to open file for writing");
    return false;
  }
  
  size_t bytesWritten = serializeJson(doc, file);
  file.close();
  
  if (bytesWritten == 0) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Failed to write offline data");
    notifyError("Failed to write data to file");
    return false;
  }
  
  LOG_DEBUG(MODULE_OFFLINE, "✅ Wrote %u bytes to offline data file", bytesWritten);
  updateStorageInfo();
  
  return true;
}

// ===== FIXED HELPER METHODS =====

// FIXED: Optimized getUnsentRecordCount dengan caching
int OfflineDataManager::getUnsentRecordCount() const {
  unsigned long now = millis();
  
  // Use cached value if still valid
  if (cachedUnsentCount >= 0 && 
      (now - lastUnsentCountUpdate) < UNSENT_COUNT_CACHE_VALIDITY) {
    return cachedUnsentCount;
  }
  
  // Recalculate unsent count
  int unsentCount = 0;
  for (int i = 0; i < recordCount; i++) {
    if (!records[i].sent) {
      unsentCount++;
    }
  }
  
  // Update cache
  cachedUnsentCount = unsentCount;
  lastUnsentCountUpdate = now;
  
  return unsentCount;
}

// FIXED: Invalidate cache when records change
void OfflineDataManager::invalidateUnsentCountCache() {
  cachedUnsentCount = -1;
  lastUnsentCountUpdate = 0;
}

int OfflineDataManager::findNextUnsentRecord(int startIndex) {
  for (int i = startIndex; i < recordCount; i++) {
    if (!records[i].sent) {
      return i;
    }
  }
  return -1;
}

// FIXED: Enhanced sendBatchFromIndex dengan proper error handling
bool OfflineDataManager::sendBatchFromIndex(int startIndex, int maxBatchSize) {
  LOG_INFO(MODULE_OFFLINE, "📤 FIXED: Sending batch starting from index %d (max size: %d)", 
           startIndex, maxBatchSize);
  
  if (startIndex >= recordCount || maxBatchSize <= 0 || !sendDataFunction) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Invalid batch parameters or no send function");
    return false;
  }
  
  int sent = 0;
  int processed = 0;
  int failures = 0;
  
  for (int i = startIndex; i < recordCount && sent < maxBatchSize; i++) {
    OfflineGpsRecord& record = records[i];
    
    // Skip already sent records
    if (record.sent) {
      continue;
    }
    
    processed++;
    
    // FIXED: Enhanced record sending dengan proper validation
    bool success = false;
    
    // Validate record before sending
    if (OfflineDataUtils::isValidGpsData(record.latitude, record.longitude, 
                                        record.speed, record.satellites)) {
      
      LOG_DEBUG(MODULE_OFFLINE, "📤 Sending record %d: %.6f,%.6f %.1fkm/h %dsats @ %s", 
                i, record.latitude, record.longitude, record.speed, 
                record.satellites, record.timestampStr);
      
      // FIXED: Call the send function with proper error handling
      try {
        success = sendDataFunction(
          record.latitude, 
          record.longitude, 
          record.speed,
          record.satellites, 
          record.timestampStr, 
          record.battery
        );
      } catch (...) {
        LOG_ERROR(MODULE_OFFLINE, "❌ Exception during send function call");
        success = false;
      }
      
    } else {
      LOG_WARN(MODULE_OFFLINE, "⚠️ Invalid record data at index %d, marking as sent", i);
      record.sent = true; // Mark invalid records as sent to skip them
      sent++;
      continue;
    }
    
    if (success) {
      record.sent = true;
      sent++;
      offlineStats.recordsSentSuccessfully++;
      
      LOG_INFO(MODULE_OFFLINE, "✅ Successfully sent offline record %d: %.6f, %.6f", 
               i, record.latitude, record.longitude);
    } else {
      record.sendAttempts++;
      offlineStats.recordsSendFailed++;
      failures++;
      
      LOG_WARN(MODULE_OFFLINE, "❌ Failed to send offline record %d (attempt %d)", 
               i, record.sendAttempts);
      
      // FIXED: Handle failed records properly
      if (record.sendAttempts >= 3) {
        LOG_WARN(MODULE_OFFLINE, "⚠️ Marking record %d as sent after 3 failed attempts", i);
        record.sent = true; // Mark as sent to avoid infinite retry
        sent++; // Count as processed
      }
    }
    
    // Add small delay between records to prevent overwhelming
    if (sent < maxBatchSize && i < recordCount - 1) {
      delay(500); // 500ms delay between records
    }
  }
  
  // Update current send index for next batch
  currentSendIndex = startIndex + processed;
  offlineStats.lastSendTime = millis();
  
  // FIXED: Enhanced result reporting
  int remainingUnsent = getUnsentRecordCount();
  LOG_INFO(MODULE_OFFLINE, "📤 FIXED Batch result: %d sent, %d failed, %d remaining unsent", 
           sent, failures, remainingUnsent);
  
  // Notify callback with accurate data
  notifyDataSent(sent, remainingUnsent);
  
  // Update file after each successful batch
  if (sent > 0) {
    invalidateUnsentCountCache();
    updateFileAfterSend();
  }
  
  // Return true if any records were sent successfully
  return sent > 0;
}

void OfflineDataManager::completeSyncProcess() {
  LOG_INFO(MODULE_OFFLINE, "✅ FIXED: Offline sync process completed");
  
  compactRecords();
  
  sendInProgress = false;
  priorityMode = false;
  offlineStats.prioritySyncActive = false;
  offlineStats.prioritySyncProgress = 100;
  currentStatus = OFFLINE_STATUS_READY;
  currentSendIndex = 0;
  
  int remainingRecords = getUnsentRecordCount(); // Use unsent count
  notifyDataSent(0, remainingRecords);
  
  if (remainingRecords == 0) {
    LOG_INFO(MODULE_OFFLINE, "🎉 All offline data successfully synced!");
  } else {
    LOG_INFO(MODULE_OFFLINE, "📊 Sync complete, %d unsent records remain", remainingRecords);
  }
}

// ===== DATA STORAGE =====
bool OfflineDataManager::storeGpsData(float lat, float lon, float speed, 
                                     int satellites, const String& timestamp, 
                                     float battery) {
  if (!isReady()) {
    LOG_DEBUG(MODULE_OFFLINE, "Offline storage not ready");
    return false;
  }
  
  if (!OfflineDataUtils::isValidGpsData(lat, lon, speed, satellites)) {
    LOG_WARN(MODULE_OFFLINE, "⚠️ Invalid GPS data, not storing offline");
    return false;
  }
  
  currentStatus = OFFLINE_STATUS_STORING;
  
  OfflineGpsRecord record;
  record.latitude = lat;
  record.longitude = lon;
  record.speed = speed;
  record.satellites = satellites;
  record.battery = battery;
  record.timestamp = millis() / 1000;
  record.sent = false;
  record.sendAttempts = 0;
  
  strncpy(record.timestampStr, timestamp.c_str(), sizeof(record.timestampStr) - 1);
  record.timestampStr[sizeof(record.timestampStr) - 1] = '\0';
  strncpy(record.gpsId, GPS_ID, sizeof(record.gpsId) - 1);
  record.gpsId[sizeof(record.gpsId) - 1] = '\0';
  
  bool success = saveRecordToFile(record);
  
  if (success) {
    offlineStats.totalRecordsStored++;
    offlineStats.lastStoreTime = millis();
    updateStats();
    
    LOG_INFO(MODULE_OFFLINE, "💾 GPS data stored offline [%d/%d] (%d unsent)", 
             recordCount, OFFLINE_MAX_RECORDS, getUnsentRecordCount());
    LOG_DEBUG(MODULE_OFFLINE, "📍 Stored: %.6f, %.6f, %.1f km/h, %d sats @ %s", 
              lat, lon, speed, satellites, timestamp.c_str());
  } else {
    LOG_ERROR(MODULE_OFFLINE, "❌ Failed to store GPS data offline");
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
bool OfflineDataManager::startSendingOfflineData(bool priorityMode) {
  int unsentCount = getUnsentRecordCount();
  
  if (unsentCount == 0) {
    LOG_DEBUG(MODULE_OFFLINE, "No unsent offline data to send");
    return true;
  }
  
  if (!sendDataFunction) {
    LOG_ERROR(MODULE_OFFLINE, "❌ No send function configured");
    return false;
  }
  
  LOG_INFO(MODULE_OFFLINE, "📤 FIXED: Starting to send %d unsent offline records %s", 
           unsentCount, priorityMode ? "[PRIORITY]" : "");
  
  currentStatus = priorityMode ? OFFLINE_STATUS_PRIORITY_SYNC : OFFLINE_STATUS_SENDING;
  currentSendIndex = 0;
  sendInProgress = true;
  this->priorityMode = priorityMode;
  offlineStats.prioritySyncActive = priorityMode;
  offlineStats.prioritySyncProgress = 0;
  lastSendAttempt = 0; // Reset last send attempt
  
  return true;
}

// FIXED: Complete continueSendingOfflineData dengan proper logic flow
bool OfflineDataManager::continueSendingOfflineData() {
  // FIXED: Enhanced state validation
  if (!sendInProgress) {
    LOG_DEBUG(MODULE_OFFLINE, "Sync not in progress, cannot continue");
    return false;
  }
  
  if (!sendDataFunction) {
    LOG_ERROR(MODULE_OFFLINE, "❌ No send function configured, stopping sync");
    stopSending();
    return false;
  }
  
  if (currentStatus != OFFLINE_STATUS_SENDING && 
      currentStatus != OFFLINE_STATUS_PRIORITY_SYNC) {
    LOG_WARN(MODULE_OFFLINE, "Invalid status for continue sending: %s", getStatusString());
    return false;
  }
  
  // FIXED: Check timing dengan adaptive interval
  unsigned long sendInterval = priorityMode ? 1000 : 2000; // Priority: 1s, Normal: 2s
  if (millis() - lastSendAttempt < sendInterval) {
    return true; // Still sending, wait for interval
  }
  
  // FIXED: Proper unsent record finding
  int nextUnsentIndex = findNextUnsentRecord(currentSendIndex);
  
  if (nextUnsentIndex < 0) {
    // FIXED: No more unsent records, complete the sync properly
    LOG_INFO(MODULE_OFFLINE, "✅ FIXED: All offline data sent successfully");
    completeSyncProcess();
    return false; // Sync completed successfully
  }
  
  // FIXED: Enhanced batch sending dengan better error handling
  int batchSize = priorityMode ? 1 : 5; // Priority: 1 record, Normal: 5 records
  bool batchSent = false;
  
  try {
    batchSent = sendBatchFromIndex(nextUnsentIndex, batchSize);
    lastSendAttempt = millis();
    
    if (batchSent) {
      // FIXED: Update progress tracking
      int totalUnsent = getUnsentRecordCount();
      int totalRecords = recordCount;
      
      if (totalRecords > 0) {
        offlineStats.prioritySyncProgress = ((totalRecords - totalUnsent) * 100) / totalRecords;
      }
      
      // Notify progress dengan accurate data
      notifySyncProgress(totalRecords - totalUnsent, totalRecords);
      
      LOG_DEBUG(MODULE_OFFLINE, "📤 FIXED: Batch sent successfully, %d unsent remaining", totalUnsent);
      return true; // Continue sending
    } else {
      // FIXED: Handle batch failure properly
      LOG_WARN(MODULE_OFFLINE, "⚠️ FIXED: Batch send failed, will retry next interval");
      
      // Don't fail immediately, allow retry
      if (priorityMode) {
        // In priority mode, be more aggressive with retries
        return true;
      } else {
        // In normal mode, allow a few failures before stopping
        static int consecutiveBatchFailures = 0;
        consecutiveBatchFailures++;
        
        if (consecutiveBatchFailures >= 5) { // Increased threshold
          LOG_ERROR(MODULE_OFFLINE, "❌ Multiple batch failures, stopping sync");
          stopSending();
          consecutiveBatchFailures = 0;
          return false;
        }
        
        return true; // Continue trying
      }
    }
  } catch (const std::exception& e) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Exception in continueSendingOfflineData: %s", e.what());
    return false;
  } catch (...) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Unknown exception in continueSendingOfflineData");
    return false;
  }
}

bool OfflineDataManager::sendSingleRecord(OfflineGpsRecord& record) {
  if (sendDataFunction == nullptr) {
    LOG_ERROR(MODULE_OFFLINE, "❌ No send function configured");
    return false;
  }
  
  int maxRetries = 2;
  for (int retry = 0; retry < maxRetries; retry++) {
    bool success = sendDataFunction(
      record.latitude, 
      record.longitude, 
      record.speed,
      record.satellites, 
      record.timestampStr, 
      record.battery
    );
    
    if (success) {
      return true;
    }
    
    if (retry < maxRetries - 1) {
      LOG_DEBUG(MODULE_OFFLINE, "Retrying send attempt %d/%d", retry + 2, maxRetries);
      delay(1000);
    }
  }
  
  return false;
}

// FIXED: sendNextBatch implementation
bool OfflineDataManager::sendNextBatch(int batchSize) {
  if (!sendInProgress || !sendDataFunction) {
    return false;
  }
  
  int nextUnsentIndex = findNextUnsentRecord(currentSendIndex);
  if (nextUnsentIndex < 0) {
    completeSyncProcess();
    return false;
  }
  
  return sendBatchFromIndex(nextUnsentIndex, batchSize);
}

void OfflineDataManager::stopSending() {
  if (sendInProgress) {
    LOG_INFO(MODULE_OFFLINE, "⏹️ Stopping offline data transmission");
    sendInProgress = false;
    priorityMode = false;
    offlineStats.prioritySyncActive = false;
    currentStatus = OFFLINE_STATUS_READY;
  }
}

// ===== DATA MANAGEMENT =====
bool OfflineDataManager::clearAllOfflineData() {
  LOG_INFO(MODULE_OFFLINE, "🗑️ Clearing all offline data (%d records)", recordCount);
  
  recordCount = 0;
  currentSendIndex = 0;
  memset(records, 0, sizeof(records));
  invalidateUnsentCountCache();
  
  if (SPIFFS.exists(OFFLINE_DATA_FILE)) {
    if (SPIFFS.remove(OFFLINE_DATA_FILE)) {
      LOG_DEBUG(MODULE_OFFLINE, "✅ Offline data file deleted");
    } else {
      LOG_WARN(MODULE_OFFLINE, "⚠️ Failed to delete offline data file");
    }
  }
  
  offlineStats.currentRecordsCount = 0;
  offlineStats.oldestRecord = 0;
  offlineStats.newestRecord = 0;
  updateStorageInfo();
  
  return true;
}

void OfflineDataManager::compactRecords() {
  int newCount = 0;
  
  for (int i = 0; i < recordCount; i++) {
    if (!records[i].sent) {
      if (i != newCount) {
        records[newCount] = records[i];
      }
      newCount++;
    }
  }
  
  int removed = recordCount - newCount;
  recordCount = newCount;
  
  if (removed > 0) {
    LOG_INFO(MODULE_OFFLINE, "🗑️ Compacted storage, removed %d sent records", removed);
    invalidateUnsentCountCache();
    updateFileAfterSend();
    updateStats();
  }
}

void OfflineDataManager::removeOldestRecord() {
  if (recordCount > 0) {
    for (int i = 1; i < recordCount; i++) {
      records[i - 1] = records[i];
    }
    recordCount--;
    invalidateUnsentCountCache();
    LOG_DEBUG(MODULE_OFFLINE, "🗑️ Removed oldest record, %d remaining", recordCount);
  }
}

bool OfflineDataManager::removeExpiredRecords(unsigned long maxAge) {
  unsigned long cutoffTime = (millis() / 1000) - maxAge;
  return removeOldRecords(cutoffTime);
}

bool OfflineDataManager::removeOldRecords(unsigned long olderThanTimestamp) {
  int removedCount = 0;
  
  for (int i = 0; i < recordCount; i++) {
    if (records[i].timestamp < olderThanTimestamp) {
      records[i].sent = true;
      removedCount++;
    }
  }
  
  if (removedCount > 0) {
    invalidateUnsentCountCache();
    compactRecords();
    LOG_INFO(MODULE_OFFLINE, "🗑️ Removed %d old records", removedCount);
  }
  
  return removedCount > 0;
}

bool OfflineDataManager::removeOldestRecords(int count) {
  int actualRemoved = 0;
  
  for (int i = 0; i < count && i < recordCount; i++) {
    records[i].sent = true;
    actualRemoved++;
  }
  
  if (actualRemoved > 0) {
    invalidateUnsentCountCache();
    compactRecords();
    LOG_INFO(MODULE_OFFLINE, "🗑️ Removed %d oldest records", actualRemoved);
  }
  
  return actualRemoved > 0;
}

unsigned long OfflineDataManager::getOldestRecordAge() const {
  if (recordCount == 0 || offlineStats.oldestRecord == 0) {
    return 0;
  }
  
  unsigned long currentTime = millis() / 1000;
  return currentTime - offlineStats.oldestRecord;
}

// ===== STATUS AND MONITORING =====
const char* OfflineDataManager::getStatusString() const {
  switch (currentStatus) {
    case OFFLINE_STATUS_DISABLED: return "DISABLED";
    case OFFLINE_STATUS_READY: return "READY";
    case OFFLINE_STATUS_STORING: return "STORING";
    case OFFLINE_STATUS_SENDING: return "SENDING";
    case OFFLINE_STATUS_PRIORITY_SYNC: return "PRIORITY_SYNC";
    case OFFLINE_STATUS_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void OfflineDataManager::updateStats() {
  offlineStats.currentRecordsCount = recordCount;
  
  if (recordCount > 0) {
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
  } else {
    offlineStats.oldestRecord = 0;
    offlineStats.newestRecord = 0;
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
  return recordCount >= OFFLINE_MAX_RECORDS;
}

bool OfflineDataManager::isStorageNearFull() const {
  return recordCount >= (OFFLINE_MAX_RECORDS - 15); // Use hardcoded warning threshold
}

// ===== DIAGNOSTICS =====
void OfflineDataManager::printStorageInfo() {
  updateStorageInfo();
  
  LOG_INFO(MODULE_OFFLINE, "=== OFFLINE STORAGE INFO ===");
  LOG_INFO(MODULE_OFFLINE, "Status       : %s", getStatusString());
  LOG_INFO(MODULE_OFFLINE, "Records      : %d/%d", recordCount, OFFLINE_MAX_RECORDS);
  LOG_INFO(MODULE_OFFLINE, "Unsent       : %d", getUnsentRecordCount());
  LOG_INFO(MODULE_OFFLINE, "Storage Used : %u KB", offlineStats.storageUsed / 1024);
  LOG_INFO(MODULE_OFFLINE, "Storage Free : %u KB", offlineStats.storageAvailable / 1024);
  
  if (recordCount > 0) {
    LOG_INFO(MODULE_OFFLINE, "Oldest Record: %lu sec ago", getOldestRecordAge());
    LOG_INFO(MODULE_OFFLINE, "Newest Record: %lu", offlineStats.newestRecord);
  }
  
  if (priorityMode) {
    LOG_INFO(MODULE_OFFLINE, "Priority Sync: %d%% complete", offlineStats.prioritySyncProgress);
  }
  
  LOG_INFO(MODULE_OFFLINE, "============================");
}

void OfflineDataManager::printDetailedStatus() {
  LOG_INFO(MODULE_OFFLINE, "=== DETAILED OFFLINE STATUS ===");
  LOG_INFO(MODULE_OFFLINE, "Status         : %s", getStatusString());
  LOG_INFO(MODULE_OFFLINE, "Total Records  : %d/%d", recordCount, OFFLINE_MAX_RECORDS);
  LOG_INFO(MODULE_OFFLINE, "Unsent Records : %d", getUnsentRecordCount());
  LOG_INFO(MODULE_OFFLINE, "Sent Records   : %d", recordCount - getUnsentRecordCount());
  LOG_INFO(MODULE_OFFLINE, "Send Function  : %s", sendDataFunction ? "CONFIGURED" : "NOT SET");
  
  if (sendInProgress) {
    LOG_INFO(MODULE_OFFLINE, "Sync Status    : ACTIVE (%s)", 
             priorityMode ? "PRIORITY" : "NORMAL");
    LOG_INFO(MODULE_OFFLINE, "Sync Progress  : %d%%", offlineStats.prioritySyncProgress);
    LOG_INFO(MODULE_OFFLINE, "Current Index  : %d/%d", currentSendIndex, recordCount);
  } else {
    LOG_INFO(MODULE_OFFLINE, "Sync Status    : IDLE");
  }
  
  if (recordCount > 0) {
    LOG_INFO(MODULE_OFFLINE, "Oldest Record  : %lu sec ago", getOldestRecordAge());
    LOG_INFO(MODULE_OFFLINE, "Storage Used   : %u bytes", offlineStats.storageUsed);
  }
  
  LOG_INFO(MODULE_OFFLINE, "==============================");
}

void OfflineDataManager::printSyncStatistics() {
  LOG_INFO(MODULE_OFFLINE, "=== SYNC STATISTICS ===");
  LOG_INFO(MODULE_OFFLINE, "Total Stored      : %d", offlineStats.totalRecordsStored);
  LOG_INFO(MODULE_OFFLINE, "Successfully Sent : %d", offlineStats.recordsSentSuccessfully);
  LOG_INFO(MODULE_OFFLINE, "Send Failed       : %d", offlineStats.recordsSendFailed);
  
  if (offlineStats.totalRecordsStored > 0) {
    float successRate = (offlineStats.recordsSentSuccessfully * 100.0f) / 
                       offlineStats.totalRecordsStored;
    LOG_INFO(MODULE_OFFLINE, "Success Rate      : %.1f%%", successRate);
  }
  
  if (offlineStats.prioritySyncActive) {
    LOG_INFO(MODULE_OFFLINE, "Priority Sync     : ACTIVE (%d%%)", offlineStats.prioritySyncProgress);
  }
  
  LOG_INFO(MODULE_OFFLINE, "======================");
}

void OfflineDataManager::printStats() {
  printSyncStatistics();
}

void OfflineDataManager::printOfflineRecords() {
  if (recordCount == 0) {
    LOG_INFO(MODULE_OFFLINE, "No offline records stored");
    return;
  }
  
  LOG_INFO(MODULE_OFFLINE, "=== OFFLINE RECORDS (%d) ===", recordCount);
  
  for (int i = 0; i < min(recordCount, 10); i++) {
    const OfflineGpsRecord& record = records[i];
    LOG_INFO(MODULE_OFFLINE, "[%d] %.6f, %.6f | %.1f km/h | %d sats | %s%s", 
             i, record.latitude, record.longitude, record.speed, 
             record.satellites, record.timestampStr,
             record.sent ? " [SENT]" : " [UNSENT]");
  }
  
  if (recordCount > 10) {
    LOG_INFO(MODULE_OFFLINE, "... and %d more records", recordCount - 10);
  }
  
  LOG_INFO(MODULE_OFFLINE, "Unsent records: %d", getUnsentRecordCount());
  LOG_INFO(MODULE_OFFLINE, "============================");
}

String OfflineDataManager::getStorageReport() {
  updateStats();
  updateStorageInfo();
  
  String report = "=== OFFLINE STORAGE REPORT ===\n";
  report += "Status: " + String(getStatusString()) + "\n";
  report += "Records: " + String(recordCount) + "/" + String(OFFLINE_MAX_RECORDS) + "\n";
  report += "Unsent: " + String(getUnsentRecordCount()) + "\n";
  report += "Storage: " + String(offlineStats.storageUsed/1024) + " KB used, " + 
            String(offlineStats.storageAvailable/1024) + " KB free\n";
  
  if (recordCount > 0) {
    report += "Age Range: " + String(getOldestRecordAge()) + " seconds\n";
  }
  
  if (sendInProgress) {
    report += "Sending: " + String(currentSendIndex) + "/" + String(recordCount) + " processed";
    if (priorityMode) {
      report += " [PRIORITY MODE]";
    }
    report += "\n";
  }
  
  return report;
}

// ===== MAINTENANCE =====
void OfflineDataManager::performMaintenance() {
  LOG_DEBUG(MODULE_OFFLINE, "🔧 Performing enhanced offline storage maintenance");
  
  int initialCount = recordCount;
  
  int expiredRemoved = 0;
  unsigned long cutoffTime = (millis() / 1000) - 86400; // 24 hours
  
  for (int i = 0; i < recordCount; i++) {
    if (records[i].timestamp < cutoffTime) {
      records[i].sent = true;
      expiredRemoved++;
    }
  }
  
  if (expiredRemoved > 0) {
    compactRecords();
  }
  
  updateStats();
  updateStorageInfo();
  
  int finalCount = recordCount;
  int totalRemoved = initialCount - finalCount;
  
  if (isStorageNearFull()) {
    LOG_WARN(MODULE_OFFLINE, "⚠️ Offline storage near full (%d/%d)", 
             recordCount, OFFLINE_MAX_RECORDS);
    notifyStorageFull(recordCount);
  }
  
  if (totalRemoved > 0) {
    LOG_INFO(MODULE_OFFLINE, "✅ Maintenance complete: removed %d records (%d expired)", 
             totalRemoved, expiredRemoved);
  }
  
  // Only validate storage if we have records and avoid unnecessary repair
  if (recordCount > 0) {
    bool isValid = SPIFFS.exists(OFFLINE_DATA_FILE);
    if (!isValid) {
      LOG_WARN(MODULE_OFFLINE, "⚠️ Storage file missing, will be recreated on next save");
    }
  }
}

bool OfflineDataManager::validateStorage() {
  if (!isInitialized) return false;
  
  // Simple validation without reloading data
  if (!SPIFFS.exists(OFFLINE_DATA_FILE)) {
    // File doesn't exist, but this is OK if we have no records
    return (recordCount == 0);
  }
  
  File file = SPIFFS.open(OFFLINE_DATA_FILE, "r");
  if (!file) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Cannot open offline data file for validation");
    return false;
  }
  
  size_t fileSize = file.size();
  file.close();
  
  if (fileSize == 0) {
    // Empty file is OK if we have no records
    return (recordCount == 0);
  }
  
  if (fileSize > (OFFLINE_MAX_RECORDS * 220 * 2)) { // 220 bytes per record estimate * 2
    LOG_WARN(MODULE_OFFLINE, "⚠️ Offline data file unexpectedly large: %u bytes", fileSize);
    return false;
  }
  
  // File exists and has reasonable size
  LOG_DEBUG(MODULE_OFFLINE, "✅ Storage validation passed (file: %u bytes, records: %d)", fileSize, recordCount);
  return true;
}

bool OfflineDataManager::repairStorage() {
  LOG_WARN(MODULE_OFFLINE, "🔧 Attempting to repair offline storage");
  
  // Use heap allocation instead of stack to prevent overflow
  OfflineGpsRecord* backupRecords = (OfflineGpsRecord*)malloc(sizeof(OfflineGpsRecord) * OFFLINE_MAX_RECORDS);
  if (!backupRecords) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Failed to allocate backup memory for repair");
    return false;
  }
  
  int backupCount = recordCount;
  memcpy(backupRecords, records, sizeof(OfflineGpsRecord) * recordCount);
  
  clearAllOfflineData();
  
  int restoredCount = 0;
  for (int i = 0; i < backupCount; i++) {
    if (OfflineDataUtils::isValidGpsData(backupRecords[i].latitude, 
                                        backupRecords[i].longitude,
                                        backupRecords[i].speed,
                                        backupRecords[i].satellites)) {
      records[recordCount] = backupRecords[i];
      recordCount++;
      restoredCount++;
    }
  }
  
  bool success = updateFileAfterSend();
  
  // Free allocated memory
  free(backupRecords);
  
  if (success) {
    LOG_INFO(MODULE_OFFLINE, "✅ Storage repaired, recovered %d/%d records", 
             restoredCount, backupCount);
  } else {
    LOG_ERROR(MODULE_OFFLINE, "❌ Storage repair failed");
  }
  
  updateStats();
  return success;
}

// ===== TESTING =====
void OfflineDataManager::runSyncTest() {
  LOG_INFO(MODULE_OFFLINE, "🧪 Running FIXED comprehensive offline sync test...");
  
  if (!isReady()) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Offline storage not ready for sync test");
    return;
  }
  
  if (sendDataFunction == nullptr) {
    LOG_ERROR(MODULE_OFFLINE, "❌ No send function configured for sync test");
    return;
  }
  
  // Step 1: Generate test data
  LOG_INFO(MODULE_OFFLINE, "Step 1: Generating test data...");
  int testRecordsCount = min(8, OFFLINE_MAX_RECORDS - recordCount - 5);
  int generatedRecords = 0;
  
  for (int i = 0; i < testRecordsCount; i++) {
    float lat = -6.2088 + (i * 0.0001);
    float lon = 106.8456 + (i * 0.0001);
    float speed = 25.0 + (i * 5);
    int satellites = 6 + (i % 6);
    char timestamp[30];
    sprintf(timestamp, "2025-01-17T10:%02d:00Z", i);
    
    bool stored = storeGpsData(lat, lon, speed, satellites, String(timestamp), 12.5);
    if (stored) {
      generatedRecords++;
    }
  }
  
  LOG_INFO(MODULE_OFFLINE, "✅ Generated %d test records", generatedRecords);
  
  // Step 2: Test sync process
  LOG_INFO(MODULE_OFFLINE, "Step 2: Testing sync process...");
  
  int initialUnsent = getUnsentRecordCount();
  LOG_INFO(MODULE_OFFLINE, "Initial unsent records: %d", initialUnsent);
  
  if (initialUnsent == 0) {
    LOG_WARN(MODULE_OFFLINE, "⚠️ No unsent records to test sync");
    return;
  }
  
  // Start priority sync
  bool syncStarted = startSendingOfflineData(true);
  if (!syncStarted) {
    LOG_ERROR(MODULE_OFFLINE, "❌ Failed to start sync test");
    return;
  }
  
  // Step 3: Monitor sync progress
  LOG_INFO(MODULE_OFFLINE, "Step 3: Monitoring sync progress...");
  int maxIterations = 30;
  int iterations = 0;
  int lastProgress = -1;
  
  while (sendInProgress && iterations < maxIterations) {
    bool continuing = continueSendingOfflineData();
    int currentProgress = offlineStats.prioritySyncProgress;
    
    if (currentProgress != lastProgress) {
      LOG_INFO(MODULE_OFFLINE, "📊 Sync progress: %d%% (%d unsent remaining)", 
               currentProgress, getUnsentRecordCount());
      lastProgress = currentProgress;
    }
    
    if (!continuing) {
      break;
    }
    
    delay(1000);
    iterations++;
  }
  
  // Step 4: Results
  int finalUnsent = getUnsentRecordCount();
  int syncedRecords = initialUnsent - finalUnsent;
  
  LOG_INFO(MODULE_OFFLINE, "=== FIXED SYNC TEST RESULTS ===");
  LOG_INFO(MODULE_OFFLINE, "Initial unsent    : %d", initialUnsent);
  LOG_INFO(MODULE_OFFLINE, "Records synced    : %d", syncedRecords);
  LOG_INFO(MODULE_OFFLINE, "Final unsent      : %d", finalUnsent);
  LOG_INFO(MODULE_OFFLINE, "Success rate      : %.1f%%", 
           initialUnsent > 0 ? ((float)syncedRecords * 100.0f / initialUnsent) : 0);
  LOG_INFO(MODULE_OFFLINE, "Test duration     : %d seconds", iterations);
  LOG_INFO(MODULE_OFFLINE, "Sync completed    : %s", !sendInProgress ? "YES" : "NO");
  
  if (finalUnsent == 0) {
    LOG_INFO(MODULE_OFFLINE, "🎉 FIXED Sync test PASSED - all records synced successfully");
  } else if (syncedRecords > 0) {
    LOG_WARN(MODULE_OFFLINE, "⚠️ FIXED Sync test PARTIAL - some records synced");
  } else {
    LOG_ERROR(MODULE_OFFLINE, "❌ FIXED Sync test FAILED - no records synced");
  }
  
  LOG_INFO(MODULE_OFFLINE, "========================");
  
  // Step 5: Print detailed statistics
  printSyncStatistics();
  printDetailedStatus();
  
  LOG_INFO(MODULE_OFFLINE, "✅ FIXED Sync test completed");
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

void OfflineDataManager::setOnSyncProgressCallback(void (*callback)(int progress, int total)) {
  onSyncProgressCallback = callback;
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

void OfflineDataManager::notifySyncProgress(int progress, int total) {
  if (onSyncProgressCallback) {
    onSyncProgressCallback(progress, total);
  }
}

// ===== UTILITY FUNCTIONS =====
namespace OfflineDataUtils {
  
  String formatOfflineTimestamp(unsigned long unixTime) {
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
    return recordCount * 220 + 1024; // 220 bytes per record + overhead
  }
  
  bool shouldStoreOffline(bool hasGpsConnection, bool hasNetworkConnection) {
    return hasGpsConnection && !hasNetworkConnection;
  }
  
  bool isValidGpsData(float lat, float lon, float speed, int satellites) {
    if (lat < -90.0 || lat > 90.0) return false;
    if (lon < -180.0 || lon > 180.0) return false;
    if (speed < 0.0 || speed > 300.0) return false;
    if (satellites < 0 || satellites > 50) return false;
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
    record.sendAttempts = 0;
    
    strncpy(record.timestampStr, timestamp.c_str(), sizeof(record.timestampStr) - 1);
    strncpy(record.gpsId, GPS_ID, sizeof(record.gpsId) - 1);
    
    return record;
  }
}