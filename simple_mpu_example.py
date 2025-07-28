#!/usr/bin/env python3
"""
Contoh Penggunaan Sensor MPU yang Sudah Dikalibrasi
Aplikasi sederhana untuk membaca data MPU dan menampilkan orientasi
"""

import time
import math
import numpy as np
from mpu_calibration_python import MPUCalibrator

class SimpleMPUExample:
    def __init__(self):
        """Inisialisasi MPU dengan kalibrasi"""
        self.mpu = MPUCalibrator()
        
        # Coba load kalibrasi yang sudah ada
        try:
            self.mpu.load_calibration_data()
            print("Kalibrasi dimuat dari file")
        except:
            print("Kalibrasi tidak ditemukan, menggunakan nilai default")
        
        # Variabel untuk complementary filter
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.dt = 0.01  # 100Hz sampling rate
        self.alpha = 0.96  # Complementary filter coefficient
        
    def calculate_attitude(self, accel, gyro):
        """
        Menghitung roll, pitch, yaw menggunakan complementary filter
        
        Args:
            accel: Data accelerometer yang sudah dikalibrasi (m/s²)
            gyro: Data gyroscope yang sudah dikalibrasi (rad/s)
        
        Returns:
            Tuple (roll, pitch, yaw) dalam derajat
        """
        # Hitung roll dan pitch dari accelerometer
        accel_roll = math.atan2(accel[1], accel[2]) * 180 / math.pi
        accel_pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)) * 180 / math.pi
        
        # Integrasi gyroscope untuk mendapatkan perubahan sudut
        gyro_roll = self.roll + gyro[0] * self.dt * 180 / math.pi
        gyro_pitch = self.pitch + gyro[1] * self.dt * 180 / math.pi
        gyro_yaw = self.yaw + gyro[2] * self.dt * 180 / math.pi
        
        # Complementary filter
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = gyro_yaw  # Yaw hanya dari gyroscope
        
        return self.roll, self.pitch, self.yaw
    
    def run_attitude_display(self, duration=30):
        """
        Menampilkan orientasi sensor secara real-time
        
        Args:
            duration: Durasi tampilan dalam detik
        """
        print(f"=== Tampilan Orientasi MPU ({duration} detik) ===")
        print("Format: Roll, Pitch, Yaw (derajat)")
        print("Tekan Ctrl+C untuk berhenti")
        print("-" * 50)
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                # Baca data mentah
                accel_raw = np.array([
                    self.mpu.mpu.get_accel_data()['x'],
                    self.mpu.mpu.get_accel_data()['y'],
                    self.mpu.mpu.get_accel_data()['z']
                ])
                
                gyro_raw = np.array([
                    self.mpu.mpu.get_gyro_data()['x'],
                    self.mpu.mpu.get_gyro_data()['y'],
                    self.mpu.mpu.get_gyro_data()['z']
                ])
                
                # Terapkan kalibrasi
                accel_cal = self.mpu.apply_calibration(accel_raw, 'accel')
                gyro_cal = self.mpu.apply_calibration(gyro_raw, 'gyro')
                
                # Hitung orientasi
                roll, pitch, yaw = self.calculate_attitude(accel_cal, gyro_cal)
                
                # Tampilkan hasil
                print(f"Roll: {roll:6.2f}° | Pitch: {pitch:6.2f}° | Yaw: {yaw:6.2f}°")
                
                time.sleep(self.dt)
                
        except KeyboardInterrupt:
            print("\nTampilan dihentikan oleh user")
    
    def run_motion_detection(self, duration=30):
        """
        Deteksi gerakan sederhana
        
        Args:
            duration: Durasi deteksi dalam detik
        """
        print(f"=== Deteksi Gerakan MPU ({duration} detik) ===")
        print("Deteksi: Diam, Bergerak, Berputar")
        print("-" * 50)
        
        start_time = time.time()
        last_accel = None
        last_gyro = None
        
        try:
            while time.time() - start_time < duration:
                # Baca dan kalibrasi data
                accel_raw = np.array([
                    self.mpu.mpu.get_accel_data()['x'],
                    self.mpu.mpu.get_accel_data()['y'],
                    self.mpu.mpu.get_accel_data()['z']
                ])
                
                gyro_raw = np.array([
                    self.mpu.mpu.get_gyro_data()['x'],
                    self.mpu.mpu.get_gyro_data()['y'],
                    self.mpu.mpu.get_gyro_data()['z']
                ])
                
                accel_cal = self.mpu.apply_calibration(accel_raw, 'accel')
                gyro_cal = self.mpu.apply_calibration(gyro_raw, 'gyro')
                
                # Deteksi gerakan
                motion_state = "Diam"
                
                if last_accel is not None:
                    # Hitung perubahan accelerometer
                    accel_change = np.linalg.norm(accel_cal - last_accel)
                    gyro_change = np.linalg.norm(gyro_cal - last_gyro)
                    
                    if accel_change > 0.5:  # Threshold untuk gerakan linear
                        motion_state = "Bergerak"
                    elif gyro_change > 0.1:  # Threshold untuk rotasi
                        motion_state = "Berputar"
                
                last_accel = accel_cal.copy()
                last_gyro = gyro_cal.copy()
                
                # Tampilkan status
                print(f"Status: {motion_state} | Accel: {np.linalg.norm(accel_cal):5.2f} m/s² | Gyro: {np.linalg.norm(gyro_cal):5.2f} rad/s")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nDeteksi dihentikan oleh user")
    
    def run_data_logging(self, duration=10, filename="mpu_data.csv"):
        """
        Mencatat data MPU ke file CSV
        
        Args:
            duration: Durasi pencatatan dalam detik
            filename: Nama file output
        """
        print(f"=== Pencatatan Data MPU ({duration} detik) ===")
        print(f"Data akan disimpan ke: {filename}")
        print("-" * 50)
        
        import csv
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Roll', 'Pitch', 'Yaw'])
            
            start_time = time.time()
            
            try:
                while time.time() - start_time < duration:
                    # Baca dan kalibrasi data
                    accel_raw = np.array([
                        self.mpu.mpu.get_accel_data()['x'],
                        self.mpu.mpu.get_accel_data()['y'],
                        self.mpu.mpu.get_accel_data()['z']
                    ])
                    
                    gyro_raw = np.array([
                        self.mpu.mpu.get_gyro_data()['x'],
                        self.mpu.mpu.get_gyro_data()['y'],
                        self.mpu.mpu.get_gyro_data()['z']
                    ])
                    
                    accel_cal = self.mpu.apply_calibration(accel_raw, 'accel')
                    gyro_cal = self.mpu.apply_calibration(gyro_raw, 'gyro')
                    
                    # Hitung orientasi
                    roll, pitch, yaw = self.calculate_attitude(accel_cal, gyro_cal)
                    
                    # Tulis ke CSV
                    timestamp = time.time() - start_time
                    writer.writerow([
                        f"{timestamp:.3f}",
                        f"{accel_cal[0]:.4f}",
                        f"{accel_cal[1]:.4f}",
                        f"{accel_cal[2]:.4f}",
                        f"{gyro_cal[0]:.4f}",
                        f"{gyro_cal[1]:.4f}",
                        f"{gyro_cal[2]:.4f}",
                        f"{roll:.2f}",
                        f"{pitch:.2f}",
                        f"{yaw:.2f}"
                    ])
                    
                    print(f"Waktu: {timestamp:5.1f}s | Roll: {roll:6.2f}° | Pitch: {pitch:6.2f}° | Yaw: {yaw:6.2f}°")
                    
                    time.sleep(0.01)  # 100Hz sampling
                    
            except KeyboardInterrupt:
                print("\nPencatatan dihentikan oleh user")
        
        print(f"Data berhasil disimpan ke {filename}")


def main():
    """Menu utama aplikasi"""
    print("=== Contoh Penggunaan MPU ===")
    print("Pilih aplikasi:")
    print("1. Tampilan Orientasi Real-time")
    print("2. Deteksi Gerakan")
    print("3. Pencatatan Data ke CSV")
    print("4. Test Kalibrasi")
    
    try:
        choice = input("Masukkan pilihan (1-4): ").strip()
        
        example = SimpleMPUExample()
        
        if choice == '1':
            duration = int(input("Durasi tampilan (detik): ") or "30")
            example.run_attitude_display(duration)
            
        elif choice == '2':
            duration = int(input("Durasi deteksi (detik): ") or "30")
            example.run_motion_detection(duration)
            
        elif choice == '3':
            duration = int(input("Durasi pencatatan (detik): ") or "10")
            filename = input("Nama file (default: mpu_data.csv): ") or "mpu_data.csv"
            example.run_data_logging(duration, filename)
            
        elif choice == '4':
            example.mpu.test_calibration(duration=5)
            example.mpu.plot_calibration_results()
            
        else:
            print("Pilihan tidak valid")
            
    except KeyboardInterrupt:
        print("\nAplikasi dihentikan oleh user")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()