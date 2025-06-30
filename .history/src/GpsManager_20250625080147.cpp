// GpsManager.cpp - Implementasi Manajer GPS
#include "GpsManager.h"
#include <math.h>

// Konstruktor
GpsManager::GpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial) 
  : gps(gpsInstance), 
    serialGPS(serial),
    lastBufferClearTime(0),
    lastFixTime(0),
    fixLostTime(0),
    lastFixValid(false),
    newFixFlag(false),
    lastLatitude(0),
    lastLongitude(0),
    lastPositionTime(0),
    distanceFromLastPosition(0),
    filterIndex(0),
    filterReady(false),
    currentHDOP(99.9) {
  
  // Inisialisasi filter posisi
  memset(positionFilter, 0, sizeof(positionFilter));
}

// Inisialisasi modul GPS
void GpsManager::begin() {
  // Mulai dengan baud rate standar
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  LOG_INFO(MODULE_GPS, "Modul GPS diinisialisasi pada pin RX:%d TX:%d", GPS_RX_PIN, GPS_TX_PIN);
  
  delay(1000);
  
  // Coba konfigurasi untuk update rate tinggi jika diinginkan
  // enableHighUpdateRate(10); // Uncomment jika GPS support 10Hz
}

// Kirim command ke modul GPS
void GpsManager::sendCommand(const char* cmd) {
  serialGPS.println(cmd);
  LOG_DEBUG(MODULE_GPS, "Command GPS dikirim: %s", cmd);
  delay(100);
}

// Enable update rate tinggi
void GpsManager::enableHighUpdateRate(int hz) {
  const char* command = nullptr;
  
  switch(hz) {
    case 1:
      command = PMTK_SET_NMEA_UPDATE_1HZ;
      break;
    case 5:
      command = PMTK_SET_NMEA_UPDATE_5HZ;
      break;
    case 10:
      command = PMTK_SET_NMEA_UPDATE_10HZ;
      break;
    default:
      LOG_WARN(MODULE_GPS, "Update rate tidak didukung: %d Hz", hz);
      return;
  }
  
  if (command) {
    sendCommand(command);
    LOG_INFO(MODULE_GPS, "GPS update rate diset ke %d Hz", hz);
  }
}

// Update data GPS
void GpsManager::update() {
  // Baca semua data GPS yang tersedia
  while (serialGPS.available() > 0) {
    char c = serialGPS.read();
    gps.encode(c);
    
    // Cek apakah ada update posisi
    if (gps.location.isUpdated()) {
      applyPositionFilter();
    }
  }
  
  // Update HDOP jika tersedia
  if (gps.hdop.isValid()) {
    currentHDOP = gps.hdop.hdop();
  }
  
  // Track perubahan GPS fix
  bool currentFixValid = gps.location.isValid() && currentHDOP < 3.0;
  unsigned long currentTime = millis();
  
  // Deteksi GPS fix diperoleh
  if (currentFixValid && !lastFixValid) {
    LOG_INFO(MODULE_GPS, "✅ GPS fix diperoleh! Lat: %.6f, Lon: %.6f, HDOP: %.1f", 
             getLatitude(), getLongitude(), currentHDOP);
    lastFixTime = currentTime;
    fixLostTime = 0;
    newFixFlag = true;
  }
  
  // Deteksi GPS fix hilang
  if (!currentFixValid && lastFixValid) {
    LOG_WARN(MODULE_GPS, "⚠️ GPS fix hilang! HDOP: %.1f", currentHDOP);
    fixLostTime = currentTime;
  }
  
  // Hitung jarak dari posisi terakhir
  if (currentFixValid && lastLatitude != 0 && lastLongitude != 0) {
    float currentLat = getLatitude();
    float currentLon = getLongitude();
    
    // Hitung jarak
    distanceFromLastPosition = calculateDistance(
      lastLatitude, lastLongitude, currentLat, currentLon
    );
    
    // Update posisi terakhir jika bergerak signifikan (> 5 meter)
    if (distanceFromLastPosition > 5.0) {
      lastLatitude = currentLat;
      lastLongitude = currentLon;
      lastPositionTime = currentTime;
    }
  } else if (currentFixValid) {
    // Inisialisasi posisi terakhir
    lastLatitude = getLatitude();
    lastLongitude = getLongitude();
    lastPositionTime = currentTime;
  }
  
  lastFixValid = currentFixValid;
  
  // Bersihkan buffer GPS secara periodik
  if (currentTime - lastBufferClearTime >= GPS_BUFFER_CLEAR_INTERVAL) {
    clearBuffer();
    lastBufferClearTime = currentTime;
  }
}

// Terapkan filter posisi
void GpsManager::applyPositionFilter() {
  if (!gps.location.isValid()) return;
  
  // Tambah posisi baru ke filter
  positionFilter[filterIndex][0] = gps.location.lat();
  positionFilter[filterIndex][1] = gps.location.lng();
  filterIndex = (filterIndex + 1) % 3;
  
  // Tandai filter siap setelah terisi penuh
  if (filterIndex == 0 && !filterReady) {
    filterReady = true;
  }
}

// Hitung jarak antara dua koordinat (dalam meter)
float GpsManager::calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Haversine formula sederhana untuk jarak pendek
  const float R = 6371000; // Radius bumi dalam meter
  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  
  float a = sin(dLat/2) * sin(dLat/2) +
            cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
            sin(dLon/2) * sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  
  return R * c;
}

// Cek apakah GPS fix valid
bool GpsManager::isValid() const {
  return gps.location.isValid() && currentHDOP < 2.0; // HDOP < 2.0 untuk akurasi bagus
}

// Cek dan reset flag fix baru
bool GpsManager::hasNewFix() {
  bool result = newFixFlag;
  newFixFlag = false;
  return result;
}

// Dapatkan latitude dengan filter
float GpsManager::getLatitude() const {
  if (!filterReady || !gps.location.isValid()) {
    return gps.location.lat();
  }
  
  // Return median dari 3 sample
  float lats[3];
  for (int i = 0; i < 3; i++) {
    lats[i] = positionFilter[i][0];
  }
  
  // Bubble sort untuk 3 elemen
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (lats[j] > lats[j + 1]) {
        float temp = lats[j];
        lats[j] = lats[j + 1];
        lats[j + 1] = temp;
      }
    }
  }
  
  return lats[1]; // Return median
}

// Dapatkan longitude dengan filter
float GpsManager::getLongitude() const {
  if (!filterReady || !gps.location.isValid()) {
    return gps.location.lng();
  }
  
  // Return median dari 3 sample
  float lons[3];
  for (int i = 0; i < 3; i++) {
    lons[i] = positionFilter[i][1];
  }
  
  // Bubble sort untuk 3 elemen
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2 - i; j++) {
      if (lons[j] > lons[j + 1]) {
        float temp = lons[j];
        lons[j] = lons[j + 1];
        lons[j + 1] = temp;
      }
    }
  }
  
  return lons[1]; // Return median
}

// Bersihkan buffer serial GPS
void GpsManager::clearBuffer() {
  int clearedBytes = Utils::clearSerialBuffer(serialGPS);
  
  if (clearedBytes > 100) { // Hanya log jika data signifikan
    LOG_DEBUG(MODULE_GPS, "Buffer GPS dibersihkan: %d bytes", clearedBytes);
  }
}

// Dapatkan timestamp dalam format ISO8601
void GpsManager::getTimestamp(char* timestampStr, size_t maxLen) {
  // Default timestamp dalam UTC
  strcpy(timestampStr, "2025-01-15T21:09:00Z");
  
  if (gps.date.isValid() && gps.time.isValid()) {
    // Selalu gunakan UTC (offset 0)
    Utils::formatISO8601(timestampStr, maxLen,
                        gps.date.year(), gps.date.month(), gps.date.day(),
                        gps.time.hour(), gps.time.minute(), gps.time.second(),
                        0); // UTC offset = 0
    
    LOG_TRACE(MODULE_GPS, "GPS Time (UTC): %s", timestampStr);
  } else {
    LOG_TRACE(MODULE_GPS, "GPS time tidak valid, menggunakan default");
  }
}

// Dapatkan jumlah satelit
int GpsManager::getSatellites() const {
  return gps.satellites.isValid() ? gps.satellites.value() : 0;
}

// Dapatkan waktu sejak pergerakan terakhir
unsigned long GpsManager::getTimeSinceLastMovement() const {
  if (lastPositionTime == 0) return 0;
  return millis() - lastPositionTime;
}