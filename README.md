# Kalibrasi Sensor MPU (Motion Processing Unit)

Repository ini berisi panduan lengkap dan tools untuk kalibrasi sensor MPU seperti MPU6050, MPU9250, dan sensor MPU lainnya.

## 📋 Daftar Isi

- [Pengenalan](#pengenalan)
- [Persyaratan Hardware](#persyaratan-hardware)
- [Instalasi](#instalasi)
- [Cara Penggunaan](#cara-penggunaan)
- [Metode Kalibrasi](#metode-kalibrasi)
- [Troubleshooting](#troubleshooting)
- [Contoh Aplikasi](#contoh-aplikasi)

## 🎯 Pengenalan

Sensor MPU (Motion Processing Unit) adalah sensor yang menggabungkan:
- **Accelerometer**: Mengukur percepatan linear
- **Gyroscope**: Mengukur kecepatan sudut
- **Magnetometer**: Mengukur medan magnet (pada MPU9250)

Kalibrasi yang tepat sangat penting untuk mendapatkan data yang akurat dan dapat diandalkan.

## 🔧 Persyaratan Hardware

### Sensor MPU
- MPU6050 (6-axis: accelerometer + gyroscope)
- MPU9250 (9-axis: accelerometer + gyroscope + magnetometer)
- Atau sensor MPU kompatibel lainnya

### Mikrokontroler/Board
- Raspberry Pi (direkomendasikan)
- Arduino
- ESP32/ESP8266
- Atau board dengan interface I2C

### Koneksi Hardware
```
MPU6050/MPU9250 Pinout:
VCC    -> 3.3V
GND    -> GND
SCL    -> GPIO3 (SDA)
SDA    -> GPIO2 (SCL)
```

## 📦 Instalasi

### 1. Clone Repository
```bash
git clone <repository-url>
cd mpu-calibration
```

### 2. Setup Otomatis (Raspberry Pi)
```bash
chmod +x setup_mpu_calibration.sh
./setup_mpu_calibration.sh
```

### 3. Setup Manual
```bash
# Install dependensi sistem
sudo apt-get update
sudo apt-get install python3-pip i2c-tools

# Install Python packages
pip3 install -r requirements.txt

# Enable I2C (Raspberry Pi)
sudo raspi-config
# Interface Options -> I2C -> Enable
```

## 🚀 Cara Penggunaan

### Menggunakan Python Script

1. **Aktifkan Virtual Environment**
```bash
source mpu_env/bin/activate
```

2. **Jalankan Kalibrasi**
```bash
python mpu_calibration_python.py
```

3. **Ikuti Instruksi**
- Posisikan sensor dalam 6 orientasi berbeda
- Tunggu proses pengumpulan data
- Lihat hasil kalibrasi dan plot

### Menggunakan Arduino

1. **Upload Kode**
```cpp
// Upload mpu_calibration_arduino.ino ke Arduino
```

2. **Buka Serial Monitor**
- Baud rate: 9600
- Ikuti instruksi di serial monitor

## 📊 Metode Kalibrasi

### 1. Kalibrasi Accelerometer

**Prinsip**: Menggunakan gravitasi bumi sebagai referensi

**Langkah-langkah**:
1. Posisikan sensor dalam 6 orientasi:
   - Posisi datar (Z-axis up)
   - Posisi terbalik (Z-axis down)
   - Berdiri X-axis up
   - Berdiri X-axis down
   - Berdiri Y-axis up
   - Berdiri Y-axis down

2. Ambil 100-1000 sampel per orientasi

3. Hitung offset dan scale factor

### 2. Kalibrasi Gyroscope

**Prinsip**: Sensor diam seharusnya memberikan nilai nol

**Langkah-langkah**:
1. Pastikan sensor benar-benar diam
2. Ambil 1000-2000 sampel
3. Hitung rata-rata sebagai offset

### 3. Kalibrasi Magnetometer

**Prinsip**: Rotasi 360° untuk mendapatkan kalibrasi hard iron dan soft iron

**Langkah-langkah**:
1. Putar sensor 360° pada setiap sumbu
2. Ambil sampel setiap 10-15 derajat
3. Hitung hard iron offset dan soft iron scale

## 🔍 Troubleshooting

### Sensor Tidak Terdeteksi
```bash
# Cek koneksi I2C
i2cdetect -y 1

# Output yang diharapkan:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# 60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- -- 
# 70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
```

### Data Tidak Stabil
- Cek power supply (gunakan 3.3V stabil)
- Hindari getaran dan gerakan
- Gunakan filter digital

### Offset Terlalu Besar
- Cek koneksi hardware
- Pastikan sensor tidak rusak
- Ulangi kalibrasi

## 📈 Contoh Aplikasi

### 1. Drone/Quadcopter
```python
# Contoh penggunaan untuk drone
class DroneController:
    def __init__(self):
        self.mpu = MPUCalibrator()
        self.mpu.load_calibration_data()
    
    def get_attitude(self):
        accel = self.mpu.get_calibrated_accel()
        gyro = self.mpu.get_calibrated_gyro()
        
        # Hitung roll, pitch, yaw
        roll = np.arctan2(accel[1], accel[2]) * 180/np.pi
        pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2)) * 180/np.pi
        
        return roll, pitch
```

### 2. Robot Balancing
```python
# Contoh untuk robot balancing
class BalancingRobot:
    def __init__(self):
        self.mpu = MPUCalibrator()
        self.mpu.load_calibration_data()
    
    def balance_control(self):
        accel = self.mpu.get_calibrated_accel()
        
        # Hitung sudut kemiringan
        angle = np.arctan2(accel[0], accel[2]) * 180/np.pi
        
        # PID control untuk balancing
        error = 0 - angle  # Target angle = 0
        # Implementasi PID controller
        return motor_speed
```

### 3. Motion Tracking
```python
# Contoh untuk motion tracking
class MotionTracker:
    def __init__(self):
        self.mpu = MPUCalibrator()
        self.mpu.load_calibration_data()
    
    def track_motion(self):
        accel = self.mpu.get_calibrated_accel()
        gyro = self.mpu.get_calibrated_gyro()
        
        # Integrasi untuk mendapatkan posisi
        velocity = integrate(accel)
        position = integrate(velocity)
        
        return position
```

## 📁 Struktur File

```
mpu-calibration/
├── mpu_calibration_guide.md      # Panduan lengkap kalibrasi
├── mpu_calibration_python.py     # Script kalibrasi Python
├── mpu_calibration_arduino.ino   # Kode kalibrasi Arduino
├── setup_mpu_calibration.sh      # Script setup otomatis
├── requirements.txt              # Dependensi Python
├── README.md                     # File ini
├── calibration_data/             # Data kalibrasi tersimpan
└── logs/                         # Log kalibrasi
```

## 🤝 Kontribusi

Kontribusi sangat diterima! Silakan:
1. Fork repository
2. Buat branch fitur baru
3. Commit perubahan
4. Push ke branch
5. Buat Pull Request

## 📄 Lisensi

Proyek ini dilisensikan di bawah MIT License - lihat file [LICENSE](LICENSE) untuk detail.

## 📞 Dukungan

Jika mengalami masalah:
1. Cek [Troubleshooting](#troubleshooting)
2. Buka issue di GitHub
3. Konsultasi dokumentasi sensor MPU

---

**Catatan**: Kalibrasi yang baik memerlukan kesabaran dan ketelitian. Pastikan untuk mengikuti semua langkah dengan teliti untuk hasil yang optimal.