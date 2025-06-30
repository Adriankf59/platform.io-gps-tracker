// RealtimeGpsManager.cpp - Implementation of Enhanced GPS Manager
#include "RealtimeGpsManager.h"
#include <math.h>

#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232

RealtimeGpsManager::RealtimeGpsManager(TinyGPSPlus& gpsInstance, HardwareSerial& serial)
  : gps(gpsInstance), serialGPS(serial) {
  
  // Initialize Kalman filter
  kalman.initialized = false;
  kalman.lastUpdate = 0;
  
  // Initialize position buffer
  for (int i = 0; i < POSITION_BUFFER_SIZE; i++) {
    positionBuffer[i].valid = false;
  }
}

void RealtimeGpsManager::begin() {
  serialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  LOG_INFO(MODULE_GPS, "Realtime GPS module initialized");
}

void RealtimeGpsManager::update() {
  // Read all available GPS data
  while (serialGPS.available() > 0) {
    char c = serialGPS.read();
    gps.encode(c);
    
    // Update HDOP if available
    if (gps.hdop.isValid()) {
      hdop = gps.hdop.hdop();
    }
  }
  
  // Check if we have a new valid position
  if (gps.location.isValid() && gps.location.isUpdated()) {
    unsigned long currentTime = millis();
    
    // Create new position entry
    GpsPosition newPos;
    newPos.latitude = gps.location.lat();
    newPos.longitude = gps.location.lng();
    newPos.speed = gps.speed.kmph();
    newPos.course = gps.course.deg();
    newPos.timestamp = currentTime;
    newPos.valid = true;
    
    // Validate position is realistic
    if (isPositionRealistic(newPos.latitude, newPos.longitude, newPos.speed)) {
      // Add to buffer
      addToBuffer(newPos);
      
      // Update Kalman filter
      updateKalmanFilter(newPos.latitude, newPos.longitude, currentTime);
      
      lastValidFix = currentTime;
      
      LOG_DEBUG(MODULE_GPS, "GPS Update: %.6f, %.6f @ %.1f km/h", 
                newPos.latitude, newPos.longitude, newPos.speed);
    } else {
      LOG_WARN(MODULE_GPS, "Unrealistic GPS position rejected");
    }
  }
}

InterpolatedPosition RealtimeGpsManager::getCurrentPosition() {
  return predictPosition(millis());
}

InterpolatedPosition RealtimeGpsManager::getPredictedPosition(unsigned long futureTimeMs) {
  return predictPosition(millis() + futureTimeMs);
}

InterpolatedPosition RealtimeGpsManager::predictPosition(unsigned long targetTime) {
  InterpolatedPosition result;
  
  // If Kalman filter not initialized, return raw GPS
  if (!kalman.initialized || !hasValidFix()) {
    result.latitude = getRawLatitude();
    result.longitude = getRawLongitude();
    result.speed = getRawSpeed();
    result.course = getCourse();
    result.accuracy = getPositionAccuracy();
    return result;
  }
  
  // Calculate time delta
  float dt = (targetTime - kalman.lastUpdate) / 1000.0; // Convert to seconds
  
  // Limit prediction time to avoid drift
  if (dt > 5.0) dt = 5.0;
  if (dt < 0) dt = 0;
  
  // If vehicle is stationary, don't predict movement
  if (!isMoving()) {
    result.latitude = kalman.lat;
    result.longitude = kalman.lon;
    result.speed = 0;
    result.course = getCourse();
    result.accuracy = getPositionAccuracy();
    return result;
  }
  
  // Predict position using motion model
  // Position = current + velocity * time + 0.5 * acceleration * time²
  double latMetersPerDegree = 111320.0;
  double lonMetersPerDegree = 111320.0 * cos(kalman.lat * DEG_TO_RAD);
  
  // Convert velocities from m/s to degrees/s
  double latVelDeg = kalman.latVelocity / latMetersPerDegree;
  double lonVelDeg = kalman.lonVelocity / lonMetersPerDegree;
  
  // Convert accelerations from m/s² to degrees/s²
  double latAccelDeg = kalman.latAccel / latMetersPerDegree;
  double lonAccelDeg = kalman.lonAccel / lonMetersPerDegree;
  
  // Predict position
  result.latitude = kalman.lat + latVelDeg * dt + 0.5 * latAccelDeg * dt * dt;
  result.longitude = kalman.lon + lonVelDeg * dt + 0.5 * lonAccelDeg * dt * dt;
  
  // Predict speed (with decay if no recent updates)
  float speedDecayFactor = 1.0;
  if (dt > 1.0) {
    speedDecayFactor = exp(-(dt - 1.0) * 0.3); // Exponential decay
  }
  result.speed = getRawSpeed() * speedDecayFactor;
  
  // Calculate predicted course from velocity vector
  if (kalman.latVelocity != 0 || kalman.lonVelocity != 0) {
    result.course = atan2(kalman.lonVelocity, kalman.latVelocity) * RAD_TO_DEG;
    if (result.course < 0) result.course += 360;
  } else {
    result.course = getCourse();
  }
  
  // Estimate accuracy degradation over time
  result.accuracy = getPositionAccuracy() * (1.0 + dt * 0.5);
  
  return result;
}

void RealtimeGpsManager::updateKalmanFilter(double lat, double lon, unsigned long currentTime) {
  if (!kalman.initialized) {
    // Initialize Kalman state
    kalman.lat = lat;
    kalman.lon = lon;
    kalman.latVelocity = 0;
    kalman.lonVelocity = 0;
    kalman.latAccel = 0;
    kalman.lonAccel = 0;
    kalman.lastUpdate = currentTime;
    kalman.initialized = true;
    return;
  }
  
  // Calculate time delta
  float dt = (currentTime - kalman.lastUpdate) / 1000.0; // seconds
  if (dt <= 0 || dt > 10.0) return; // Skip unrealistic time jumps
  
  // Convert positions to meters
  double latMetersPerDegree = 111320.0;
  double lonMetersPerDegree = 111320.0 * cos(kalman.lat * DEG_TO_RAD);
  
  double deltaLatMeters = (lat - kalman.lat) * latMetersPerDegree;
  double deltaLonMeters = (lon - kalman.lon) * lonMetersPerDegree;
  
  // Calculate velocities
  double newLatVelocity = deltaLatMeters / dt;
  double newLonVelocity = deltaLonMeters / dt;
  
  // Calculate accelerations
  double latAccel = (newLatVelocity - kalman.latVelocity) / dt;
  double lonAccel = (newLonVelocity - kalman.lonVelocity) / dt;
  
  // Limit accelerations to realistic values
  double accelMagnitude = sqrt(latAccel * latAccel + lonAccel * lonAccel);
  if (accelMagnitude > maxAcceleration) {
    double scale = maxAcceleration / accelMagnitude;
    latAccel *= scale;
    lonAccel *= scale;
  }
  
  // Apply Kalman filter (simplified)
  const float processNoise = 0.1;
  const float measurementNoise = 3.0; // meters
  
  // Update with weighted average
  float alpha = 0.7; // Filter coefficient (0-1, higher = trust new measurement more)
  
  kalman.lat = alpha * lat + (1 - alpha) * kalman.lat;
  kalman.lon = alpha * lon + (1 - alpha) * kalman.lon;
  kalman.latVelocity = alpha * newLatVelocity + (1 - alpha) * kalman.latVelocity;
  kalman.lonVelocity = alpha * newLonVelocity + (1 - alpha) * kalman.lonVelocity;
  kalman.latAccel = alpha * latAccel + (1 - alpha) * kalman.latAccel;
  kalman.lonAccel = alpha * lonAccel + (1 - alpha) * kalman.lonAccel;
  
  kalman.lastUpdate = currentTime;
}

double RealtimeGpsManager::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  // Haversine formula
  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  
  double a = sin(dLat/2) * sin(dLat/2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
             sin(dLon/2) * sin(dLon/2);
  
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return 6371000 * c; // Earth radius in meters
}

float RealtimeGpsManager::calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * DEG_TO_RAD;
  
  double y = sin(dLon) * cos(lat2 * DEG_TO_RAD);
  double x = cos(lat1 * DEG_TO_RAD) * sin(lat2 * DEG_TO_RAD) -
             sin(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) * cos(dLon);
  
  double bearing = atan2(y, x) * RAD_TO_DEG;
  return fmod((bearing + 360), 360);
}

void RealtimeGpsManager::addToBuffer(const GpsPosition& pos) {
  positionBuffer[bufferIndex] = pos;
  bufferIndex = (bufferIndex + 1) % POSITION_BUFFER_SIZE;
}

GpsPosition RealtimeGpsManager::getSmoothedPosition() {
  GpsPosition smoothed;
  smoothed.valid = false;
  
  int validCount = 0;
  double sumLat = 0, sumLon = 0, sumSpeed = 0;
  
  // Calculate weighted average of recent positions
  for (int i = 0; i < POSITION_BUFFER_SIZE; i++) {
    if (positionBuffer[i].valid) {
      // Weight by recency
      int age = (bufferIndex - i + POSITION_BUFFER_SIZE) % POSITION_BUFFER_SIZE;
      float weight = 1.0 / (age + 1);
      
      sumLat += positionBuffer[i].latitude * weight;
      sumLon += positionBuffer[i].longitude * weight;
      sumSpeed += positionBuffer[i].speed * weight;
      validCount++;
    }
  }
  
  if (validCount > 0) {
    smoothed.latitude = sumLat / validCount;
    smoothed.longitude = sumLon / validCount;
    smoothed.speed = sumSpeed / validCount;
    smoothed.course = getCourse(); // Use latest course
    smoothed.timestamp = millis();
    smoothed.valid = true;
  }
  
  return smoothed;
}

bool RealtimeGpsManager::isPositionRealistic(double lat, double lon, float speed) {
  // Check if coordinates are valid
  if (lat < -90 || lat > 90 || lon < -180 || lon > 180) {
    return false;
  }
  
  // Check if speed is realistic (max 200 km/h for normal vehicles)
  if (speed > 200) {
    return false;
  }
  
  // If we have previous position, check distance jump
  if (kalman.initialized) {
    double distance = calculateDistance(kalman.lat, kalman.lon, lat, lon);
    float timeDelta = (millis() - kalman.lastUpdate) / 1000.0;
    
    // Calculate required speed for this jump
    float requiredSpeed = (distance / timeDelta) * 3.6; // m/s to km/h
    
    // Allow some margin for GPS error
    if (requiredSpeed > 250) { // Unrealistic jump
      return false;
    }
  }
  
  return true;
}

float RealtimeGpsManager::getPositionAccuracy() {
  // Estimate accuracy based on HDOP and satellite count
  float baseAccuracy = 5.0; // Base GPS accuracy in meters
  
  if (hdop > 0) {
    baseAccuracy *= hdop;
  }
  
  // Adjust by satellite count
  int sats = getSatellites();
  if (sats < 4) {
    baseAccuracy *= 3;
  } else if (sats < 6) {
    baseAccuracy *= 2;
  } else if (sats >= 10) {
    baseAccuracy *= 0.7;
  }
  
  // Adjust by time since last fix
  unsigned long fixAge = getTimeSinceLastFix();
  if (fixAge > 2000) {
    baseAccuracy *= (1 + fixAge / 5000.0);
  }
  
  return baseAccuracy;
}

void RealtimeGpsManager::getTimestamp(char* timestampStr, size_t maxLen) {
  // Use GPS time if available, otherwise system time
  if (gps.date.isValid() && gps.time.isValid()) {
    // Add milliseconds for better precision
    int milliseconds = millis() % 1000;
    
    snprintf(timestampStr, maxLen, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second(),
             milliseconds);
  } else {
    // Fallback to default timestamp
    strcpy(timestampStr, "2025-01-01T00:00:00.000Z");
  }
}