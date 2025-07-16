// ImuManager.cpp - MPU6050 IMU Management Implementation
#include "ImuManager.h"
#include <ArduinoJson.h>  // FIXED: Added missing include

// Constructor
ImuManager::ImuManager() 
  : currentState(IMU_STATE_UNINITIALIZED),
    isEnabled(ENABLE_IMU_SUPPORT),
    i2cAddress(IMU_I2C_ADDRESS),
    sampleRate(IMU_SAMPLE_RATE_GPS_VALID),
    isCalibrated(false),
    calibrationSamples(0),
    movementThreshold(IMU_MOVEMENT_THRESHOLD),
    stationaryThreshold(IMU_STATIONARY_THRESHOLD),
    lastMovementTime(0),
    bufferIndex(0),
    lastSampleTime(0),
    totalSamples(0),
    errorCount(0) {
  
  currentData.reset();
  deadReckoning.reset();
  basementDetection.reset();
  
  // Initialize buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    accelBuffer[i] = 0;
  }
}

// ===== INITIALIZATION =====
bool ImuManager::begin() {
  if (!isEnabled) {
    LOG_INFO(MODULE_IMU, "IMU support disabled");
    return true;
  }
  
  LOG_INFO(MODULE_IMU, "Initializing MPU6050...");
  currentState = IMU_STATE_INITIALIZING;
  
  // Initialize I2C
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C clock
  
  // Initialize sensor
  if (!initializeSensor()) {
    currentState = IMU_STATE_ERROR;
    LOG_ERROR(MODULE_IMU, "Failed to initialize MPU6050");
    return false;
  }
  
  // Perform initial calibration
  LOG_INFO(MODULE_IMU, "Calibrating IMU...");
  currentState = IMU_STATE_CALIBRATING;
  performCalibration();
  
  currentState = IMU_STATE_READY;
  LOG_INFO(MODULE_IMU, "✅ MPU6050 initialized successfully");
  
  return true;
}

void ImuManager::end() {
  if (!isEnabled) return;
  
  LOG_INFO(MODULE_IMU, "Shutting down IMU...");
  
  // Stop any active operations
  stopDeadReckoning();
  
  // Put sensor to sleep
  mpu.setSleepEnabled(true);
  
  currentState = IMU_STATE_UNINITIALIZED;
  LOG_INFO(MODULE_IMU, "IMU shutdown complete");
}

bool ImuManager::initializeSensor() {
  // Initialize MPU6050
  mpu.initialize();
  
  // Test connection
  if (!mpu.testConnection()) {
    LOG_ERROR(MODULE_IMU, "MPU6050 connection failed");
    return false;
  }
  
  // Configure sensor
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);    // ±250°/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);    // ±2g
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);               // Low pass filter
  mpu.setSleepEnabled(false);
  
  // Set sample rate
  mpu.setRate(sampleRate);
  
  LOG_INFO(MODULE_IMU, "MPU6050 configured successfully");
  return true;
}

// ===== CALIBRATION =====
bool ImuManager::calibrate() {
  LOG_INFO(MODULE_IMU, "Starting IMU calibration...");
  currentState = IMU_STATE_CALIBRATING;
  
  performCalibration();
  
  if (isCalibrated) {
    currentState = IMU_STATE_READY;
    LOG_INFO(MODULE_IMU, "✅ Calibration complete");
    LOG_INFO(MODULE_IMU, "Accel offsets: X=%.3f, Y=%.3f, Z=%.3f", 
             currentData.accelXOffset, currentData.accelYOffset, currentData.accelZOffset);
    LOG_INFO(MODULE_IMU, "Gyro offsets: X=%.3f, Y=%.3f, Z=%.3f", 
             currentData.gyroXOffset, currentData.gyroYOffset, currentData.gyroZOffset);
    return true;
  } else {
    currentState = IMU_STATE_ERROR;
    LOG_ERROR(MODULE_IMU, "❌ Calibration failed");
    return false;
  }
}

void ImuManager::performCalibration() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  int validSamples = 0;
  
  LOG_INFO(MODULE_IMU, "Collecting calibration samples (keep device still)...");
  
  for (int i = 0; i < IMU_CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to actual units
    float accelX = ax / 16384.0;  // For ±2g range
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    float gyroX = gx / 131.0;     // For ±250°/s range
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;
    
    // Accumulate
    sumAx += accelX;
    sumAy += accelY;
    sumAz += accelZ;
    sumGx += gyroX;
    sumGy += gyroY;
    sumGz += gyroZ;
    validSamples++;
    
    delay(10); // 10ms between samples
    
    if (i % 20 == 0) {
      LOG_DEBUG(MODULE_IMU, "Calibration progress: %d%%", (i * 100) / IMU_CALIBRATION_SAMPLES);
    }
  }
  
  if (validSamples > 0) {
    // Calculate average offsets
    currentData.accelXOffset = sumAx / validSamples;
    currentData.accelYOffset = sumAy / validSamples;
    currentData.accelZOffset = (sumAz / validSamples) - 1.0; // Subtract 1g for Z axis
    currentData.gyroXOffset = sumGx / validSamples;
    currentData.gyroYOffset = sumGy / validSamples;
    currentData.gyroZOffset = sumGz / validSamples;
    
    isCalibrated = true;
    calibrationSamples = validSamples;
  } else {
    isCalibrated = false;
  }
}

void ImuManager::resetCalibration() {
  isCalibrated = false;
  currentData.accelXOffset = 0;
  currentData.accelYOffset = 0;
  currentData.accelZOffset = 0;
  currentData.gyroXOffset = 0;
  currentData.gyroYOffset = 0;
  currentData.gyroZOffset = 0;
  LOG_INFO(MODULE_IMU, "Calibration reset");
}

// ===== DATA ACQUISITION =====
bool ImuManager::update() {
  if (!isEnabled || currentState < IMU_STATE_READY) {
    return false;
  }
  
  // Check sample rate
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime < (1000 / sampleRate)) {
    return false;
  }
  
  // Read sensor
  if (!readSensor()) {
    errorCount++;
    return false;
  }
  
  // Update derived data
  updateMovementDetection();
  
  // Update dead reckoning if active
  if (deadReckoning.active) {
    float deltaTime = (currentTime - deadReckoning.lastUpdateTime) / 1000.0;
    updateDeadReckoning(deltaTime);
    deadReckoning.lastUpdateTime = currentTime;
  }
  
  // Update basement detection
  updateBasementDetection();
  
  lastSampleTime = currentTime;
  totalSamples++;
  
  return true;
}

bool ImuManager::readSensor() {
  int16_t ax, ay, az, gx, gy, gz;
  
  try {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  } catch (...) {
    LOG_ERROR(MODULE_IMU, "Failed to read MPU6050");
    return false;
  }
  
  // Convert to actual units and apply calibration
  currentData.accelX = (ax / 16384.0) - currentData.accelXOffset;
  currentData.accelY = (ay / 16384.0) - currentData.accelYOffset;
  currentData.accelZ = (az / 16384.0) - currentData.accelZOffset;
  currentData.gyroX = (gx / 131.0) - currentData.gyroXOffset;
  currentData.gyroY = (gy / 131.0) - currentData.gyroYOffset;
  currentData.gyroZ = (gz / 131.0) - currentData.gyroZOffset;
  
  // Read temperature
  currentData.temperature = mpu.getTemperature() / 340.0 + 36.53;
  
  // Calculate derived values
  currentData.totalAccel = calculateTotalAccel();
  currentData.tiltAngle = calculateTiltAngle();
  currentData.headingRate = currentData.gyroZ;
  
  // Update heading with gyro integration
  float deltaTime = (millis() - currentData.timestamp) / 1000.0;
  if (currentData.timestamp > 0 && deltaTime > 0) {
    currentData.heading += currentData.gyroZ * deltaTime;
    // Normalize to 0-360
    while (currentData.heading < 0) currentData.heading += 360;
    while (currentData.heading >= 360) currentData.heading -= 360;
  }
  
  currentData.timestamp = millis();
  currentState = IMU_STATE_ACTIVE;
  
  return true;
}

// ===== MOVEMENT DETECTION =====
void ImuManager::updateMovementDetection() {
  // Add to buffer for averaging
  accelBuffer[bufferIndex] = currentData.totalAccel;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  
  // Calculate average acceleration
  float avgAccel = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    avgAccel += accelBuffer[i];
  }
  avgAccel /= BUFFER_SIZE;
  
  // Detect movement based on deviation from 1g
  float accelDeviation = fabs(avgAccel - 1.0);
  
  if (accelDeviation > movementThreshold) {
    currentData.isMoving = true;
    currentData.isStationary = false;
    lastMovementTime = millis();
  } else if (accelDeviation < stationaryThreshold) {
    // Check if stationary for sufficient time
    if (millis() - lastMovementTime > 2000) { // 2 seconds
      currentData.isMoving = false;
      currentData.isStationary = true;
    }
  }
  
  // Log state changes
  static bool lastMovingState = false;
  if (currentData.isMoving != lastMovingState) {
    LOG_INFO(MODULE_IMU, "Movement state changed: %s", 
             currentData.isMoving ? "MOVING" : "STATIONARY");
    lastMovingState = currentData.isMoving;
  }
}

float ImuManager::calculateTotalAccel() {
  return sqrt(currentData.accelX * currentData.accelX + 
              currentData.accelY * currentData.accelY + 
              currentData.accelZ * currentData.accelZ);
}

float ImuManager::calculateTiltAngle() {
  // Calculate tilt from vertical (Z axis)
  return atan2(sqrt(currentData.accelX * currentData.accelX + 
                    currentData.accelY * currentData.accelY), 
               currentData.accelZ) * 180.0 / PI;
}

// ===== DEAD RECKONING =====
void ImuManager::startDeadReckoning(double lat, double lon, float heading) {
  LOG_INFO(MODULE_IMU, "Starting dead reckoning from: %.6f, %.6f", lat, lon);
  
  deadReckoning.lastGpsLat = lat;
  deadReckoning.lastGpsLon = lon;
  deadReckoning.lastGpsHeading = heading;
  deadReckoning.lastGpsTime = millis();
  
  deadReckoning.estimatedLat = lat;
  deadReckoning.estimatedLon = lon;
  deadReckoning.estimatedHeading = heading;
  deadReckoning.estimatedSpeed = 0;
  deadReckoning.distanceTraveled = 0;
  deadReckoning.positionError = 0;
  deadReckoning.confidenceLevel = 1.0;
  
  deadReckoning.active = true;
  deadReckoning.startTime = millis();
  deadReckoning.lastUpdateTime = millis();
  
  // Set higher sample rate for dead reckoning
  setSampleRate(IMU_SAMPLE_RATE_GPS_LOST);
}

void ImuManager::stopDeadReckoning() {
  if (deadReckoning.active) {
    LOG_INFO(MODULE_IMU, "Stopping dead reckoning. Distance traveled: %.1f m", 
             deadReckoning.distanceTraveled);
    deadReckoning.active = false;
    
    // Restore normal sample rate
    setSampleRate(IMU_SAMPLE_RATE_GPS_VALID);
  }
}

void ImuManager::updateDeadReckoning(float deltaTime) {
  if (!deadReckoning.active || deltaTime <= 0) return;
  
  // Simple dead reckoning implementation
  // This is a basic version - can be improved with Kalman filtering
  
  // Estimate speed from acceleration (very simplified)
  if (currentData.isMoving) {
    // Use forward acceleration (X axis in vehicle frame)
    float forwardAccel = currentData.accelX * 9.81; // Convert to m/s²
    
    // Update speed estimate
    deadReckoning.estimatedSpeed += forwardAccel * deltaTime;
    
    // Apply damping to account for drag
    deadReckoning.estimatedSpeed *= 0.98;
    
    // Clamp to reasonable values
    deadReckoning.estimatedSpeed = constrain(deadReckoning.estimatedSpeed, 0, 50); // Max 50 m/s
    
    // Calculate distance
    float distance = deadReckoning.estimatedSpeed * deltaTime;
    deadReckoning.distanceTraveled += distance;
    
    // Update position
    updatePositionEstimate(distance, currentData.heading);
  } else {
    // Decay speed when stationary
    deadReckoning.estimatedSpeed *= 0.9;
  }
  
  // Update error estimation
  float timeSinceGps = (millis() - deadReckoning.lastGpsTime) / 1000.0;
  deadReckoning.positionError = 10 + (timeSinceGps * 2); // 2m/s error growth
  deadReckoning.confidenceLevel = max(0.1, 1.0 - (timeSinceGps / 300.0)); // Decay over 5 minutes
  
  // Reset if error too large
  if (deadReckoning.positionError > IMU_MAX_POSITION_ERROR) {
    LOG_WARN(MODULE_IMU, "Dead reckoning error too large, waiting for GPS");
    deadReckoning.confidenceLevel = 0;
  }
}

void ImuManager::updatePositionEstimate(float distance, float heading) {
  // Convert heading to radians
  float headingRad = heading * PI / 180.0;
  
  // Calculate position change
  double deltaLat = ImuUtils::metersToLatitude(distance * cos(headingRad), 
                                                deadReckoning.estimatedLat);
  double deltaLon = ImuUtils::metersToLongitude(distance * sin(headingRad), 
                                                 deadReckoning.estimatedLat);
  
  // Update estimated position
  deadReckoning.estimatedLat += deltaLat;
  deadReckoning.estimatedLon += deltaLon;
  deadReckoning.estimatedHeading = heading;
}

void ImuManager::updateGpsPosition(double lat, double lon, float heading) {
  if (!deadReckoning.active) return;
  
  // Calculate error between estimated and actual
  double error = ImuUtils::calculateDistance(deadReckoning.estimatedLat, 
                                           deadReckoning.estimatedLon,
                                           lat, lon);
  
  LOG_INFO(MODULE_IMU, "Dead reckoning error: %.1f m", error);
  
  // Reset dead reckoning with new GPS position
  deadReckoning.lastGpsLat = lat;
  deadReckoning.lastGpsLon = lon;
  deadReckoning.lastGpsHeading = heading;
  deadReckoning.lastGpsTime = millis();
  
  deadReckoning.estimatedLat = lat;
  deadReckoning.estimatedLon = lon;
  deadReckoning.estimatedHeading = heading;
  deadReckoning.distanceTraveled = 0;
  deadReckoning.positionError = 0;
  deadReckoning.confidenceLevel = 1.0;
}

// ===== BASEMENT DETECTION =====
void ImuManager::updateBasementDetection() {
  // Detect sudden Z-axis acceleration changes (elevator/ramp)
  float zAccelChange = currentData.accelZ - 1.0; // Remove gravity
  
  if (!basementDetection.inBasement) {
    // Check for basement entry
    if (zAccelChange < BASEMENT_ENTRY_THRESHOLD) {
      basementDetection.possibleEntry = true;
      basementDetection.entryZAccel = zAccelChange;
      basementDetection.entryTime = millis();
      LOG_INFO(MODULE_IMU, "Possible basement entry detected (Z-accel: %.2f g)", zAccelChange);
    }
  } else {
    // Check for basement exit
    if (zAccelChange > -BASEMENT_ENTRY_THRESHOLD) {
      basementDetection.possibleExit = true;
      basementDetection.exitZAccel = zAccelChange;
      basementDetection.exitTime = millis();
      LOG_INFO(MODULE_IMU, "Possible basement exit detected (Z-accel: %.2f g)", zAccelChange);
    }
  }
}

void ImuManager::notifyGpsLoss() {
  basementDetection.gpsLossCount++;
  
  // Confirm basement entry if GPS lost after detecting entry motion
  if (basementDetection.possibleEntry && 
      (millis() - basementDetection.entryTime < BASEMENT_GPS_LOSS_TIME)) {
    basementDetection.inBasement = true;
    basementDetection.possibleEntry = false;
    LOG_INFO(MODULE_IMU, "✅ Basement entry confirmed (GPS lost)");
    
    // Start dead reckoning
    if (!deadReckoning.active) {
      startDeadReckoning(deadReckoning.lastGpsLat, deadReckoning.lastGpsLon, 
                        deadReckoning.lastGpsHeading);
    }
  }
}

void ImuManager::notifyGpsRecovered() {
  // Confirm basement exit if GPS recovered after detecting exit motion
  if (basementDetection.possibleExit && 
      (millis() - basementDetection.exitTime < BASEMENT_EXIT_GPS_TIME)) {
    basementDetection.inBasement = false;
    basementDetection.possibleExit = false;
    basementDetection.gpsLossCount = 0;
    LOG_INFO(MODULE_IMU, "✅ Basement exit confirmed (GPS recovered)");
    
    // Stop dead reckoning
    stopDeadReckoning();
  }
}

// ===== POSITION SOURCE =====
PositionSource ImuManager::getPositionSource() const {
  if (!deadReckoning.active) {
    return POS_SOURCE_GPS;
  } else if (deadReckoning.confidenceLevel > 0.5) {
    return POS_SOURCE_FUSION;
  } else {
    return POS_SOURCE_IMU;
  }
}

// ===== DIAGNOSTICS =====
const char* ImuManager::getStateString() const {
  switch (currentState) {
    case IMU_STATE_UNINITIALIZED: return "UNINITIALIZED";
    case IMU_STATE_INITIALIZING: return "INITIALIZING";
    case IMU_STATE_CALIBRATING: return "CALIBRATING";
    case IMU_STATE_READY: return "READY";
    case IMU_STATE_ACTIVE: return "ACTIVE";
    case IMU_STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void ImuManager::printStatus() {
  LOG_INFO(MODULE_IMU, "=== IMU STATUS ===");
  LOG_INFO(MODULE_IMU, "State: %s", getStateString());
  LOG_INFO(MODULE_IMU, "Enabled: %s", isEnabled ? "YES" : "NO");
  LOG_INFO(MODULE_IMU, "Calibrated: %s", isCalibrated ? "YES" : "NO");
  LOG_INFO(MODULE_IMU, "Movement: %s", currentData.isMoving ? "MOVING" : "STATIONARY");
  LOG_INFO(MODULE_IMU, "Heading: %.1f°", currentData.heading);
  LOG_INFO(MODULE_IMU, "Temperature: %.1f°C", currentData.temperature);
  LOG_INFO(MODULE_IMU, "Dead Reckoning: %s", deadReckoning.active ? "ACTIVE" : "INACTIVE");
  LOG_INFO(MODULE_IMU, "Basement: %s", basementDetection.inBasement ? "YES" : "NO");
  LOG_INFO(MODULE_IMU, "Samples: %lu (%.1f%% errors)", totalSamples, getErrorRate());
}

void ImuManager::printSensorData() {
  LOG_INFO(MODULE_IMU, "=== SENSOR DATA ===");
  LOG_INFO(MODULE_IMU, "Accel: X=%.3f, Y=%.3f, Z=%.3f g", 
           currentData.accelX, currentData.accelY, currentData.accelZ);
  LOG_INFO(MODULE_IMU, "Gyro: X=%.1f, Y=%.1f, Z=%.1f °/s", 
           currentData.gyroX, currentData.gyroY, currentData.gyroZ);
  LOG_INFO(MODULE_IMU, "Total Accel: %.3f g", currentData.totalAccel);
  LOG_INFO(MODULE_IMU, "Tilt Angle: %.1f°", currentData.tiltAngle);
}

void ImuManager::printDeadReckoningInfo() {
  if (!deadReckoning.active) {
    LOG_INFO(MODULE_IMU, "Dead reckoning not active");
    return;
  }
  
  LOG_INFO(MODULE_IMU, "=== DEAD RECKONING ===");
  LOG_INFO(MODULE_IMU, "Position: %.6f, %.6f", 
           deadReckoning.estimatedLat, deadReckoning.estimatedLon);
  LOG_INFO(MODULE_IMU, "Heading: %.1f°", deadReckoning.estimatedHeading);
  LOG_INFO(MODULE_IMU, "Speed: %.1f m/s", deadReckoning.estimatedSpeed);
  LOG_INFO(MODULE_IMU, "Distance: %.1f m", deadReckoning.distanceTraveled);
  LOG_INFO(MODULE_IMU, "Error: %.1f m", deadReckoning.positionError);
  LOG_INFO(MODULE_IMU, "Confidence: %.1f%%", deadReckoning.confidenceLevel * 100);
}

String ImuManager::getJsonData() {
  StaticJsonDocument<512> doc;
  
  // Movement and heading
  doc["moving"] = currentData.isMoving;
  doc["heading"] = currentData.heading;
  doc["headingRate"] = currentData.headingRate;
  
  // Raw sensor data (if enabled)
  if (IMU_SEND_RAW_DATA) {
    JsonObject accel = doc.createNestedObject("accel");
    accel["x"] = currentData.accelX;
    accel["y"] = currentData.accelY;
    accel["z"] = currentData.accelZ;
    
    JsonObject gyro = doc.createNestedObject("gyro");
    gyro["x"] = currentData.gyroX;
    gyro["y"] = currentData.gyroY;
    gyro["z"] = currentData.gyroZ;
  }
  
  // Processed data
  if (IMU_SEND_PROCESSED_DATA) {
    doc["totalAccel"] = currentData.totalAccel;
    doc["tiltAngle"] = currentData.tiltAngle;
    doc["temperature"] = currentData.temperature;
  }
  
  // Dead reckoning data
  if (deadReckoning.active) {
    JsonObject dr = doc.createNestedObject("deadReckoning");
    dr["estimatedLat"] = deadReckoning.estimatedLat;
    dr["estimatedLon"] = deadReckoning.estimatedLon;
    dr["estimatedSpeed"] = deadReckoning.estimatedSpeed;
    dr["distanceTraveled"] = deadReckoning.distanceTraveled;
    dr["positionError"] = deadReckoning.positionError;
    dr["confidence"] = deadReckoning.confidenceLevel;
  }
  
  // Basement status
  doc["inBasement"] = basementDetection.inBasement;
  
  String output;
  serializeJson(doc, output);
  return output;
}

String ImuManager::getDiagnosticInfo() {
  StaticJsonDocument<256> doc;
  
  doc["state"] = getStateString();
  doc["samples"] = totalSamples;
  doc["errors"] = errorCount;
  doc["errorRate"] = getErrorRate();
  doc["calibrated"] = isCalibrated;
  doc["sampleRate"] = sampleRate;
  
  String output;
  serializeJson(doc, output);
  return output;
}

// ===== UTILITY FUNCTIONS =====
namespace ImuUtils {
  
  double metersToLatitude(double meters, double currentLat) {
    // Earth radius in meters
    const double earthRadius = 6371000.0;
    return (meters / earthRadius) * (180.0 / PI);
  }
  
  double metersToLongitude(double meters, double currentLat) {
    const double earthRadius = 6371000.0;
    double latRad = currentLat * PI / 180.0;
    return (meters / (earthRadius * cos(latRad))) * (180.0 / PI);
  }
  
  double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * PI / 180.0;
    lat1 = lat1 * PI / 180.0;
    lat2 = lat2 * PI / 180.0;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    double bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0);
  }
  
  double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double earthRadius = 6371000.0; // meters
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;
    
    double a = sin(dLat/2) * sin(dLat/2) +
               cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return earthRadius * c;
  }
  
  bool isValidAcceleration(float x, float y, float z) {
    // Check for reasonable acceleration values (max ±4g)
    return (abs(x) < 4.0 && abs(y) < 4.0 && abs(z) < 4.0);
  }
  
  bool isValidGyroscope(float x, float y, float z) {
    // Check for reasonable gyroscope values (max ±500°/s)
    return (abs(x) < 500.0 && abs(y) < 500.0 && abs(z) < 500.0);
  }
  
  String getMovementDescription(float totalAccel) {
    float deviation = abs(totalAccel - 1.0);
    
    if (deviation < 0.05) return "Stationary";
    else if (deviation < 0.15) return "Slight movement";
    else if (deviation < 0.3) return "Moderate movement";
    else if (deviation < 0.5) return "Fast movement";
    else return "Rapid acceleration";
  }
  
  bool detectSuddenStop(float currentAccel, float previousAccel) {
    // Detect sudden deceleration (e.g., emergency braking)
    return (previousAccel > 1.3 && currentAccel < 0.7);
  }
  
  bool detectTurn(float gyroZ, float threshold) {
    return abs(gyroZ) > threshold;
  }
}