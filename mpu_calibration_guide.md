# Panduan Kalibrasi Sensor MPU (Motion Processing Unit)

## Pengenalan
Sensor MPU (Motion Processing Unit) adalah sensor yang menggabungkan accelerometer, gyroscope, dan magnetometer untuk mengukur gerakan dan orientasi. Kalibrasi yang tepat sangat penting untuk mendapatkan data yang akurat.

## Jenis-jenis Kalibrasi MPU

### 1. Kalibrasi Accelerometer
Accelerometer mengukur percepatan dalam 3 sumbu (X, Y, Z).

#### Langkah-langkah Kalibrasi:
1. **Persiapan:**
   - Letakkan sensor pada permukaan datar dan stabil
   - Pastikan sensor tidak bergerak selama kalibrasi
   - Siapkan kode kalibrasi

2. **Proses Kalibrasi:**
   - Ambil 100-1000 sampel untuk setiap orientasi
   - Orientasi yang diperlukan:
     - Posisi datar (Z-axis up)
     - Posisi terbalik (Z-axis down)
     - Posisi berdiri (X-axis up)
     - Posisi berdiri (Y-axis up)

3. **Perhitungan Offset:**
   ```
   Offset_X = (Max_X + Min_X) / 2
   Offset_Y = (Max_Y + Min_Y) / 2
   Offset_Z = (Max_Z + Min_Z) / 2
   ```

### 2. Kalibrasi Gyroscope
Gyroscope mengukur kecepatan sudut pada 3 sumbu.

#### Langkah-langkah Kalibrasi:
1. **Persiapan:**
   - Pastikan sensor benar-benar diam
   - Hindari getaran dan gerakan

2. **Proses Kalibrasi:**
   - Ambil 1000-2000 sampel saat sensor diam
   - Hitung rata-rata dari semua sampel

3. **Perhitungan Offset:**
   ```
   Gyro_Offset_X = Average(Gyro_X_samples)
   Gyro_Offset_Y = Average(Gyro_Y_samples)
   Gyro_Offset_Z = Average(Gyro_Z_samples)
   ```

### 3. Kalibrasi Magnetometer
Magnetometer mengukur medan magnet untuk menentukan arah utara.

#### Langkah-langkah Kalibrasi:
1. **Persiapan:**
   - Hindari benda logam di sekitar sensor
   - Lakukan kalibrasi di area terbuka

2. **Proses Kalibrasi:**
   - Putar sensor 360° pada setiap sumbu
   - Ambil sampel setiap 10-15 derajat
   - Lakukan untuk semua orientasi

3. **Perhitungan Hard Iron dan Soft Iron:**
   - Hard Iron: Offset tetap dari medan magnet
   - Soft Iron: Distorsi medan magnet

## Contoh Kode Kalibrasi Arduino

```cpp
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Variabel untuk kalibrasi
float accel_offset[3] = {0, 0, 0};
float gyro_offset[3] = {0, 0, 0};
int num_samples = 1000;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 tidak terdeteksi!");
    return;
  }
  
  Serial.println("Memulai kalibrasi...");
  calibrateMPU();
}

void calibrateMPU() {
  Serial.println("Kalibrasi Accelerometer...");
  calibrateAccelerometer();
  
  Serial.println("Kalibrasi Gyroscope...");
  calibrateGyroscope();
  
  Serial.println("Kalibrasi selesai!");
  printCalibrationData();
}

void calibrateAccelerometer() {
  float accel_sum[3] = {0, 0, 0};
  
  for (int i = 0; i < num_samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    accel_sum[0] += ax;
    accel_sum[1] += ay;
    accel_sum[2] += az;
    
    delay(10);
  }
  
  accel_offset[0] = accel_sum[0] / num_samples;
  accel_offset[1] = accel_sum[1] / num_samples;
  accel_offset[2] = accel_sum[2] / num_samples;
}

void calibrateGyroscope() {
  float gyro_sum[3] = {0, 0, 0};
  
  for (int i = 0; i < num_samples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    
    gyro_sum[0] += gx;
    gyro_sum[1] += gy;
    gyro_sum[2] += gz;
    
    delay(10);
  }
  
  gyro_offset[0] = gyro_sum[0] / num_samples;
  gyro_offset[1] = gyro_sum[1] / num_samples;
  gyro_offset[2] = gyro_sum[2] / num_samples;
}

void printCalibrationData() {
  Serial.println("=== Data Kalibrasi ===");
  Serial.print("Accel Offset X: "); Serial.println(accel_offset[0]);
  Serial.print("Accel Offset Y: "); Serial.println(accel_offset[1]);
  Serial.print("Accel Offset Z: "); Serial.println(accel_offset[2]);
  Serial.print("Gyro Offset X: "); Serial.println(gyro_offset[0]);
  Serial.print("Gyro Offset Y: "); Serial.println(gyro_offset[1]);
  Serial.print("Gyro Offset Z: "); Serial.println(gyro_offset[2]);
}

void loop() {
  // Baca data dengan offset kalibrasi
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Terapkan offset kalibrasi
  float calibrated_ax = ax - accel_offset[0];
  float calibrated_ay = ay - accel_offset[1];
  float calibrated_az = az - accel_offset[2];
  
  float calibrated_gx = gx - gyro_offset[0];
  float calibrated_gy = gy - gyro_offset[1];
  float calibrated_gz = gz - gyro_offset[2];
  
  // Tampilkan data yang sudah dikalibrasi
  Serial.print("Accel: ");
  Serial.print(calibrated_ax); Serial.print(", ");
  Serial.print(calibrated_ay); Serial.print(", ");
  Serial.println(calibrated_az);
  
  Serial.print("Gyro: ");
  Serial.print(calibrated_gx); Serial.print(", ");
  Serial.print(calibrated_gy); Serial.print(", ");
  Serial.println(calibrated_gz);
  
  delay(100);
}
```

## Tips Kalibrasi yang Efektif

### 1. Lingkungan Kalibrasi
- **Suhu stabil:** Kalibrasi pada suhu yang sama dengan penggunaan
- **Minimal getaran:** Gunakan permukaan yang stabil
- **Hindari medan magnet:** Jauhkan dari motor, speaker, atau benda logam

### 2. Durasi Kalibrasi
- **Accelerometer:** 5-10 menit per orientasi
- **Gyroscope:** 2-5 menit dalam posisi diam
- **Magnetometer:** 10-15 menit untuk rotasi lengkap

### 3. Validasi Kalibrasi
- Cek nilai offset yang masuk akal
- Test dengan gerakan sederhana
- Bandingkan dengan sensor referensi

## Troubleshooting

### Masalah Umum:
1. **Offset terlalu besar:**
   - Cek koneksi hardware
   - Pastikan sensor tidak rusak
   - Ulangi kalibrasi

2. **Data tidak stabil:**
   - Cek sumber getaran
   - Pastikan power supply stabil
   - Gunakan filter digital

3. **Drift berlebihan:**
   - Kalibrasi ulang secara berkala
   - Gunakan sensor dengan kualitas lebih baik
   - Implementasikan algoritma kompensasi

## Kesimpulan
Kalibrasi MPU yang tepat memerlukan:
- Persiapan yang matang
- Proses yang sistematis
- Validasi hasil
- Pemeliharaan berkala

Dengan kalibrasi yang baik, sensor MPU dapat memberikan data yang akurat dan dapat diandalkan untuk berbagai aplikasi seperti drone, robot, dan sistem navigasi.