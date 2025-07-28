/*
 * Kalibrasi Sensor MPU6050/MPU9250 dengan Arduino
 * 
 * Fitur:
 * - Kalibrasi accelerometer dengan 6 orientasi
 * - Kalibrasi gyroscope dengan sensor diam
 * - Menyimpan data kalibrasi ke EEPROM
 * - Tampilan real-time data yang sudah dikalibrasi
 * 
 * Koneksi:
 * MPU6050/MPU9250 -> Arduino
 * VCC -> 3.3V
 * GND -> GND
 * SCL -> A5 (SCL)
 * SDA -> A4 (SDA)
 */

#include <Wire.h>
#include <EEPROM.h>

// Konfigurasi MPU
#define MPU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

// Struktur data kalibrasi
struct CalibrationData {
  float accel_offset[3];
  float gyro_offset[3];
  float accel_scale[3];
  float gyro_scale[3];
  unsigned long timestamp;
  bool is_valid;
};

CalibrationData calibration;

// Variabel untuk kalibrasi
const int NUM_SAMPLES = 1000;
const int EEPROM_ADDR = 0;

// Enum untuk state kalibrasi
enum CalibrationState {
  IDLE,
  ACCEL_CALIBRATION,
  GYRO_CALIBRATION,
  COMPLETE
};

CalibrationState currentState = IDLE;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("=== Kalibrasi Sensor MPU ===");
  Serial.println("Pilih mode:");
  Serial.println("1. Kalibrasi Accelerometer");
  Serial.println("2. Kalibrasi Gyroscope");
  Serial.println("3. Kalibrasi Lengkap");
  Serial.println("4. Load Kalibrasi dari EEPROM");
  Serial.println("5. Test Kalibrasi");
  Serial.println("6. Reset Kalibrasi");
  
  // Inisialisasi MPU
  initMPU();
  
  // Load kalibrasi dari EEPROM
  loadCalibrationFromEEPROM();
}

void loop() {
  if (Serial.available()) {
    char choice = Serial.read();
    processMenuChoice(choice);
  }
  
  // Update state machine
  updateCalibrationState();
  
  delay(100);
}

void initMPU() {
  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission();
  
  // Set sample rate to 1kHz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x19);
  Wire.write(0x07);
  Wire.endTransmission();
  
  // Set low pass filter
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x06);
  Wire.endTransmission();
  
  // Set gyro range to ±2000°/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission();
  
  // Set accelerometer range to ±16g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission();
  
  Serial.println("MPU initialized successfully");
}

void processMenuChoice(char choice) {
  switch (choice) {
    case '1':
      startAccelCalibration();
      break;
    case '2':
      startGyroCalibration();
      break;
    case '3':
      startFullCalibration();
      break;
    case '4':
      loadCalibrationFromEEPROM();
      break;
    case '5':
      testCalibration();
      break;
    case '6':
      resetCalibration();
      break;
    default:
      Serial.println("Pilihan tidak valid");
      break;
  }
}

void startAccelCalibration() {
  Serial.println("=== Kalibrasi Accelerometer ===");
  Serial.println("Posisikan sensor dalam 6 orientasi:");
  Serial.println("1. Posisi datar (Z up)");
  Serial.println("2. Posisi terbalik (Z down)");
  Serial.println("3. Berdiri X up");
  Serial.println("4. Berdiri X down");
  Serial.println("5. Berdiri Y up");
  Serial.println("6. Berdiri Y down");
  
  currentState = ACCEL_CALIBRATION;
}

void startGyroCalibration() {
  Serial.println("=== Kalibrasi Gyroscope ===");
  Serial.println("Pastikan sensor benar-benar diam...");
  Serial.println("Tekan Enter untuk memulai...");
  
  currentState = GYRO_CALIBRATION;
}

void startFullCalibration() {
  startAccelCalibration();
  // Gyro calibration akan otomatis setelah accel selesai
}

void calibrateAccelerometer() {
  float accel_data[6][3]; // 6 orientasi, 3 axis
  float expected_values[6][3] = {
    {0, 0, 9.81},    // Z up
    {0, 0, -9.81},   // Z down
    {9.81, 0, 0},    // X up
    {-9.81, 0, 0},   // X down
    {0, 9.81, 0},    // Y up
    {0, -9.81, 0}    // Y down
  };
  
  for (int orientation = 0; orientation < 6; orientation++) {
    Serial.print("Orientasi ");
    Serial.print(orientation + 1);
    Serial.println(" - Posisikan sensor dan tekan Enter...");
    
    while (!Serial.available()) {
      delay(100);
    }
    Serial.read(); // Clear buffer
    
    Serial.println("Mengumpulkan data...");
    
    float sum[3] = {0, 0, 0};
    for (int sample = 0; sample < NUM_SAMPLES; sample++) {
      int16_t ax, ay, az;
      readAccelData(&ax, &ay, &az);
      
      // Convert to m/s²
      float ax_mps2 = ax / 2048.0; // ±16g range
      float ay_mps2 = ay / 2048.0;
      float az_mps2 = az / 2048.0;
      
      sum[0] += ax_mps2;
      sum[1] += ay_mps2;
      sum[2] += az_mps2;
      
      if (sample % 100 == 0) {
        Serial.print("Progress: ");
        Serial.print(sample);
        Serial.print("/");
        Serial.println(NUM_SAMPLES);
      }
      
      delay(10);
    }
    
    accel_data[orientation][0] = sum[0] / NUM_SAMPLES;
    accel_data[orientation][1] = sum[1] / NUM_SAMPLES;
    accel_data[orientation][2] = sum[2] / NUM_SAMPLES;
    
    Serial.print("Orientasi ");
    Serial.print(orientation + 1);
    Serial.print(" selesai: ");
    Serial.print(accel_data[orientation][0]);
    Serial.print(", ");
    Serial.print(accel_data[orientation][1]);
    Serial.print(", ");
    Serial.println(accel_data[orientation][2]);
  }
  
  // Hitung offset dan scale
  calculateAccelCalibration(accel_data, expected_values);
  
  Serial.println("Kalibrasi accelerometer selesai!");
  printCalibrationData();
}

void calibrateGyroscope() {
  Serial.println("Mengumpulkan data gyroscope...");
  
  float sum[3] = {0, 0, 0};
  for (int sample = 0; sample < NUM_SAMPLES; sample++) {
    int16_t gx, gy, gz;
    readGyroData(&gx, &gy, &gz);
    
    // Convert to rad/s
    float gx_rads = gx * 0.0174533 / 16.4; // ±2000°/s range
    float gy_rads = gy * 0.0174533 / 16.4;
    float gz_rads = gz * 0.0174533 / 16.4;
    
    sum[0] += gx_rads;
    sum[1] += gy_rads;
    sum[2] += gz_rads;
    
    if (sample % 200 == 0) {
      Serial.print("Progress: ");
      Serial.print(sample);
      Serial.print("/");
      Serial.println(NUM_SAMPLES);
    }
    
    delay(10);
  }
  
  calibration.gyro_offset[0] = sum[0] / NUM_SAMPLES;
  calibration.gyro_offset[1] = sum[1] / NUM_SAMPLES;
  calibration.gyro_offset[2] = sum[2] / NUM_SAMPLES;
  
  // Set scale factors to 1 (gyro biasanya tidak perlu scale calibration)
  calibration.gyro_scale[0] = 1.0;
  calibration.gyro_scale[1] = 1.0;
  calibration.gyro_scale[2] = 1.0;
  
  Serial.println("Kalibrasi gyroscope selesai!");
  printCalibrationData();
}

void calculateAccelCalibration(float accel_data[6][3], float expected_values[6][3]) {
  // Hitung offset (bias)
  float offset_sum[3] = {0, 0, 0};
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      offset_sum[j] += accel_data[i][j];
    }
  }
  
  calibration.accel_offset[0] = offset_sum[0] / 6.0;
  calibration.accel_offset[1] = offset_sum[1] / 6.0;
  calibration.accel_offset[2] = offset_sum[2] / 6.0;
  
  // Hitung scale factors
  for (int axis = 0; axis < 3; axis++) {
    float max_measured = 0;
    float max_expected = 0;
    
    for (int i = 0; i < 6; i++) {
      float measured = abs(accel_data[i][axis] - calibration.accel_offset[axis]);
      float expected = abs(expected_values[i][axis]);
      
      if (expected > max_expected) {
        max_expected = expected;
        max_measured = measured;
      }
    }
    
    if (max_measured > 0) {
      calibration.accel_scale[axis] = max_expected / max_measured;
    } else {
      calibration.accel_scale[axis] = 1.0;
    }
  }
  
  // Set timestamp dan valid flag
  calibration.timestamp = millis();
  calibration.is_valid = true;
  
  // Simpan ke EEPROM
  saveCalibrationToEEPROM();
}

void readAccelData(int16_t* ax, int16_t* ay, int16_t* az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  
  *ax = Wire.read() << 8 | Wire.read();
  *ay = Wire.read() << 8 | Wire.read();
  *az = Wire.read() << 8 | Wire.read();
}

void readGyroData(int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  
  *gx = Wire.read() << 8 | Wire.read();
  *gy = Wire.read() << 8 | Wire.read();
  *gz = Wire.read() << 8 | Wire.read();
}

void applyCalibration(int16_t raw_ax, int16_t raw_ay, int16_t raw_az,
                     int16_t raw_gx, int16_t raw_gy, int16_t raw_gz,
                     float* cal_ax, float* cal_ay, float* cal_az,
                     float* cal_gx, float* cal_gy, float* cal_gz) {
  
  // Convert raw accelerometer data to m/s²
  float ax_mps2 = raw_ax / 2048.0;
  float ay_mps2 = raw_ay / 2048.0;
  float az_mps2 = raw_az / 2048.0;
  
  // Apply accelerometer calibration
  *cal_ax = (ax_mps2 - calibration.accel_offset[0]) * calibration.accel_scale[0];
  *cal_ay = (ay_mps2 - calibration.accel_offset[1]) * calibration.accel_scale[1];
  *cal_az = (az_mps2 - calibration.accel_offset[2]) * calibration.accel_scale[2];
  
  // Convert raw gyroscope data to rad/s
  float gx_rads = raw_gx * 0.0174533 / 16.4;
  float gy_rads = raw_gy * 0.0174533 / 16.4;
  float gz_rads = raw_gz * 0.0174533 / 16.4;
  
  // Apply gyroscope calibration
  *cal_gx = (gx_rads - calibration.gyro_offset[0]) * calibration.gyro_scale[0];
  *cal_gy = (gy_rads - calibration.gyro_offset[1]) * calibration.gyro_scale[1];
  *cal_gz = (gz_rads - calibration.gyro_offset[2]) * calibration.gyro_scale[2];
}

void testCalibration() {
  if (!calibration.is_valid) {
    Serial.println("Kalibrasi belum dilakukan atau tidak valid!");
    return;
  }
  
  Serial.println("=== Test Kalibrasi ===");
  Serial.println("Format: Raw -> Calibrated");
  Serial.println("Tekan Enter untuk berhenti...");
  
  while (!Serial.available()) {
    int16_t ax, ay, az, gx, gy, gz;
    float cal_ax, cal_ay, cal_az, cal_gx, cal_gy, cal_gz;
    
    readAccelData(&ax, &ay, &az);
    readGyroData(&gx, &gy, &gz);
    
    applyCalibration(ax, ay, az, gx, gy, gz,
                    &cal_ax, &cal_ay, &cal_az,
                    &cal_gx, &cal_gy, &cal_gz);
    
    Serial.print("Accel: ");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az);
    Serial.print(" -> ");
    Serial.print(cal_ax, 3); Serial.print(",");
    Serial.print(cal_ay, 3); Serial.print(",");
    Serial.println(cal_az, 3);
    
    Serial.print("Gyro:  ");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz);
    Serial.print(" -> ");
    Serial.print(cal_gx, 3); Serial.print(",");
    Serial.print(cal_gy, 3); Serial.print(",");
    Serial.println(cal_gz, 3);
    
    Serial.println("---");
    
    delay(500);
  }
  
  Serial.read(); // Clear buffer
}

void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_ADDR, calibration);
  Serial.println("Kalibrasi disimpan ke EEPROM");
}

void loadCalibrationFromEEPROM() {
  EEPROM.get(EEPROM_ADDR, calibration);
  
  if (calibration.is_valid) {
    Serial.println("Kalibrasi dimuat dari EEPROM:");
    printCalibrationData();
  } else {
    Serial.println("Tidak ada kalibrasi valid di EEPROM");
  }
}

void resetCalibration() {
  // Reset calibration data
  for (int i = 0; i < 3; i++) {
    calibration.accel_offset[i] = 0;
    calibration.gyro_offset[i] = 0;
    calibration.accel_scale[i] = 1.0;
    calibration.gyro_scale[i] = 1.0;
  }
  calibration.is_valid = false;
  
  // Clear EEPROM
  for (int i = 0; i < sizeof(CalibrationData); i++) {
    EEPROM.write(EEPROM_ADDR + i, 0);
  }
  
  Serial.println("Kalibrasi direset");
}

void printCalibrationData() {
  Serial.println("=== Data Kalibrasi ===");
  Serial.print("Accel Offset: ");
  Serial.print(calibration.accel_offset[0], 4);
  Serial.print(", ");
  Serial.print(calibration.accel_offset[1], 4);
  Serial.print(", ");
  Serial.println(calibration.accel_offset[2], 4);
  
  Serial.print("Accel Scale:  ");
  Serial.print(calibration.accel_scale[0], 4);
  Serial.print(", ");
  Serial.print(calibration.accel_scale[1], 4);
  Serial.print(", ");
  Serial.println(calibration.accel_scale[2], 4);
  
  Serial.print("Gyro Offset:  ");
  Serial.print(calibration.gyro_offset[0], 4);
  Serial.print(", ");
  Serial.print(calibration.gyro_offset[1], 4);
  Serial.print(", ");
  Serial.println(calibration.gyro_offset[2], 4);
  
  Serial.print("Gyro Scale:   ");
  Serial.print(calibration.gyro_scale[0], 4);
  Serial.print(", ");
  Serial.print(calibration.gyro_scale[1], 4);
  Serial.print(", ");
  Serial.println(calibration.gyro_scale[2], 4);
  
  Serial.print("Timestamp: ");
  Serial.println(calibration.timestamp);
  Serial.print("Valid: ");
  Serial.println(calibration.is_valid ? "Yes" : "No");
}

void updateCalibrationState() {
  static int orientation_count = 0;
  static unsigned long last_state_change = 0;
  
  switch (currentState) {
    case ACCEL_CALIBRATION:
      if (orientation_count < 6) {
        // Menunggu input user untuk setiap orientasi
        if (Serial.available()) {
          Serial.read(); // Clear buffer
          calibrateAccelerometer();
          orientation_count = 0;
          currentState = GYRO_CALIBRATION;
        }
      }
      break;
      
    case GYRO_CALIBRATION:
      if (millis() - last_state_change > 2000) { // Delay 2 detik
        calibrateGyroscope();
        currentState = COMPLETE;
        last_state_change = millis();
      }
      break;
      
    case COMPLETE:
      Serial.println("=== Kalibrasi Selesai ===");
      Serial.println("Pilih mode selanjutnya:");
      Serial.println("1. Kalibrasi Accelerometer");
      Serial.println("2. Kalibrasi Gyroscope");
      Serial.println("3. Kalibrasi Lengkap");
      Serial.println("4. Load Kalibrasi dari EEPROM");
      Serial.println("5. Test Kalibrasi");
      Serial.println("6. Reset Kalibrasi");
      currentState = IDLE;
      break;
      
    case IDLE:
    default:
      // Do nothing, waiting for user input
      break;
  }
}