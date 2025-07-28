#!/usr/bin/env python3
"""
Kalibrasi Sensor MPU menggunakan Python
Mendukung MPU6050, MPU9250, dan sensor MPU lainnya
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from mpu6050 import MPU6050
import smbus2 as smbus

class MPUCalibrator:
    def __init__(self, bus_number=1, mpu_address=0x68):
        """
        Inisialisasi kalibrator MPU
        
        Args:
            bus_number: Nomor bus I2C (biasanya 1 untuk Raspberry Pi)
            mpu_address: Alamat I2C sensor MPU (0x68 atau 0x69)
        """
        self.bus = smbus.SMBus(bus_number)
        self.mpu = MPU6050(self.bus, mpu_address)
        self.mpu.wake_up()
        
        # Variabel kalibrasi
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.gyro_offset = np.array([0.0, 0.0, 0.0])
        self.accel_scale = np.array([1.0, 1.0, 1.0])
        self.gyro_scale = np.array([1.0, 1.0, 1.0])
        
        # Data untuk plotting
        self.calibration_data = {
            'accel_raw': [],
            'gyro_raw': [],
            'accel_calibrated': [],
            'gyro_calibrated': []
        }
    
    def collect_samples(self, num_samples=1000, delay=0.01):
        """
        Mengumpulkan sampel data dari sensor
        
        Args:
            num_samples: Jumlah sampel yang diambil
            delay: Delay antar sampel (detik)
        
        Returns:
            Tuple (accel_data, gyro_data)
        """
        accel_data = []
        gyro_data = []
        
        print(f"Mengumpulkan {num_samples} sampel...")
        
        for i in range(num_samples):
            # Baca data accelerometer
            accel = self.mpu.get_accel_data()
            accel_data.append([accel['x'], accel['y'], accel['z']])
            
            # Baca data gyroscope
            gyro = self.mpu.get_gyro_data()
            gyro_data.append([gyro['x'], gyro['y'], gyro['z']])
            
            if i % 100 == 0:
                print(f"Progress: {i}/{num_samples}")
            
            time.sleep(delay)
        
        return np.array(accel_data), np.array(gyro_data)
    
    def calibrate_accelerometer(self, num_samples=1000):
        """
        Kalibrasi accelerometer dengan metode multi-orientasi
        """
        print("=== Kalibrasi Accelerometer ===")
        print("Posisikan sensor dalam 6 orientasi berbeda:")
        print("1. Posisi datar (Z up)")
        print("2. Posisi terbalik (Z down)")
        print("3. Berdiri X up")
        print("4. Berdiri X down")
        print("5. Berdiri Y up")
        print("6. Berdiri Y down")
        
        orientations = []
        expected_values = [
            [0, 0, 9.81],    # Z up
            [0, 0, -9.81],   # Z down
            [9.81, 0, 0],    # X up
            [-9.81, 0, 0],   # X down
            [0, 9.81, 0],    # Y up
            [0, -9.81, 0]    # Y down
        ]
        
        for i, expected in enumerate(expected_values):
            input(f"Posisikan sensor untuk orientasi {i+1} dan tekan Enter...")
            
            accel_data, _ = self.collect_samples(num_samples//6)
            orientations.append(accel_data)
            
            print(f"Orientasi {i+1} selesai")
        
        # Hitung offset dan scale
        self._calculate_accel_calibration(orientations, expected_values)
    
    def _calculate_accel_calibration(self, orientations, expected_values):
        """
        Menghitung parameter kalibrasi accelerometer
        """
        all_data = np.vstack(orientations)
        all_expected = np.array(expected_values * (len(orientations[0])))
        
        # Hitung offset (bias)
        self.accel_offset = np.mean(all_data, axis=0)
        
        # Hitung scale factor
        calibrated_data = all_data - self.accel_offset
        scale_factors = []
        
        for axis in range(3):
            # Gunakan data dari orientasi yang memiliki nilai maksimum
            max_orientations = [i for i, exp in enumerate(expected_values) if abs(exp[axis]) > 5]
            if max_orientations:
                axis_data = calibrated_data[max_orientations[0]::len(expected_values), axis]
                expected_max = max(abs(exp[axis]) for exp in expected_values)
                scale = expected_max / np.mean(np.abs(axis_data))
                scale_factors.append(scale)
            else:
                scale_factors.append(1.0)
        
        self.accel_scale = np.array(scale_factors)
        
        print("Accelerometer calibration complete!")
        print(f"Offset: {self.accel_offset}")
        print(f"Scale: {self.accel_scale}")
    
    def calibrate_gyroscope(self, num_samples=2000):
        """
        Kalibrasi gyroscope dengan sensor diam
        """
        print("=== Kalibrasi Gyroscope ===")
        print("Pastikan sensor benar-benar diam dan tidak bergerak...")
        input("Tekan Enter untuk memulai...")
        
        _, gyro_data = self.collect_samples(num_samples)
        
        # Hitung offset (bias)
        self.gyro_offset = np.mean(gyro_data, axis=0)
        
        # Hitung noise (standard deviation)
        gyro_noise = np.std(gyro_data, axis=0)
        
        print("Gyroscope calibration complete!")
        print(f"Offset: {self.gyro_offset}")
        print(f"Noise (std): {gyro_noise}")
    
    def calibrate_magnetometer(self, num_samples=1000):
        """
        Kalibrasi magnetometer (jika tersedia)
        """
        print("=== Kalibrasi Magnetometer ===")
        print("Putar sensor 360° pada setiap sumbu...")
        
        # Implementasi kalibrasi magnetometer
        # Ini memerlukan sensor dengan magnetometer (MPU9250, dll)
        try:
            mag_data = []
            for i in range(num_samples):
                mag = self.mpu.get_mag_data()
                mag_data.append([mag['x'], mag['y'], mag['z']])
                time.sleep(0.01)
            
            mag_data = np.array(mag_data)
            
            # Hitung hard iron offset
            mag_offset = (np.max(mag_data, axis=0) + np.min(mag_data, axis=0)) / 2
            
            # Hitung soft iron scale
            mag_range = (np.max(mag_data, axis=0) - np.min(mag_data, axis=0)) / 2
            mag_scale = 1.0 / mag_range
            
            print("Magnetometer calibration complete!")
            print(f"Hard Iron Offset: {mag_offset}")
            print(f"Soft Iron Scale: {mag_scale}")
            
        except AttributeError:
            print("Magnetometer tidak tersedia pada sensor ini")
    
    def apply_calibration(self, raw_data, sensor_type='accel'):
        """
        Menerapkan kalibrasi pada data mentah
        
        Args:
            raw_data: Data mentah dari sensor
            sensor_type: 'accel' atau 'gyro'
        
        Returns:
            Data yang sudah dikalibrasi
        """
        if sensor_type == 'accel':
            calibrated = (raw_data - self.accel_offset) * self.accel_scale
        elif sensor_type == 'gyro':
            calibrated = raw_data - self.gyro_offset
        else:
            raise ValueError("sensor_type harus 'accel' atau 'gyro'")
        
        return calibrated
    
    def test_calibration(self, duration=10):
        """
        Test kalibrasi dengan menampilkan data real-time
        """
        print(f"=== Test Kalibrasi ({duration} detik) ===")
        print("Format: Raw -> Calibrated")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            # Baca data mentah
            accel_raw = np.array([
                self.mpu.get_accel_data()['x'],
                self.mpu.get_accel_data()['y'],
                self.mpu.get_accel_data()['z']
            ])
            
            gyro_raw = np.array([
                self.mpu.get_gyro_data()['x'],
                self.mpu.get_gyro_data()['y'],
                self.mpu.get_gyro_data()['z']
            ])
            
            # Terapkan kalibrasi
            accel_cal = self.apply_calibration(accel_raw, 'accel')
            gyro_cal = self.apply_calibration(gyro_raw, 'gyro')
            
            # Simpan untuk plotting
            self.calibration_data['accel_raw'].append(accel_raw)
            self.calibration_data['gyro_raw'].append(gyro_raw)
            self.calibration_data['accel_calibrated'].append(accel_cal)
            self.calibration_data['gyro_calibrated'].append(gyro_cal)
            
            # Tampilkan data
            print(f"Accel: {accel_raw} -> {accel_cal}")
            print(f"Gyro:  {gyro_raw} -> {gyro_cal}")
            print("-" * 50)
            
            time.sleep(0.1)
    
    def plot_calibration_results(self):
        """
        Plot hasil kalibrasi
        """
        if not self.calibration_data['accel_raw']:
            print("Tidak ada data untuk diplot. Jalankan test_calibration() terlebih dahulu.")
            return
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('Hasil Kalibrasi MPU')
        
        # Plot accelerometer
        accel_raw = np.array(self.calibration_data['accel_raw'])
        accel_cal = np.array(self.calibration_data['accel_calibrated'])
        
        axes[0, 0].plot(accel_raw[:, 0], label='Raw X')
        axes[0, 0].plot(accel_cal[:, 0], label='Calibrated X')
        axes[0, 0].set_title('Accelerometer X')
        axes[0, 0].legend()
        
        axes[0, 1].plot(accel_raw[:, 1], label='Raw Y')
        axes[0, 1].plot(accel_cal[:, 1], label='Calibrated Y')
        axes[0, 1].set_title('Accelerometer Y')
        axes[0, 1].legend()
        
        axes[0, 2].plot(accel_raw[:, 2], label='Raw Z')
        axes[0, 2].plot(accel_cal[:, 2], label='Calibrated Z')
        axes[0, 2].set_title('Accelerometer Z')
        axes[0, 2].legend()
        
        # Plot gyroscope
        gyro_raw = np.array(self.calibration_data['gyro_raw'])
        gyro_cal = np.array(self.calibration_data['gyro_calibrated'])
        
        axes[1, 0].plot(gyro_raw[:, 0], label='Raw X')
        axes[1, 0].plot(gyro_cal[:, 0], label='Calibrated X')
        axes[1, 0].set_title('Gyroscope X')
        axes[1, 0].legend()
        
        axes[1, 1].plot(gyro_raw[:, 1], label='Raw Y')
        axes[1, 1].plot(gyro_cal[:, 1], label='Calibrated Y')
        axes[1, 1].set_title('Gyroscope Y')
        axes[1, 1].legend()
        
        axes[1, 2].plot(gyro_raw[:, 2], label='Raw Z')
        axes[1, 2].plot(gyro_cal[:, 2], label='Calibrated Z')
        axes[1, 2].set_title('Gyroscope Z')
        axes[1, 2].legend()
        
        plt.tight_layout()
        plt.show()
    
    def save_calibration_data(self, filename='mpu_calibration.json'):
        """
        Simpan data kalibrasi ke file
        """
        import json
        
        calibration_data = {
            'accel_offset': self.accel_offset.tolist(),
            'accel_scale': self.accel_scale.tolist(),
            'gyro_offset': self.gyro_offset.tolist(),
            'gyro_scale': self.gyro_scale.tolist(),
            'timestamp': time.time()
        }
        
        with open(filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"Data kalibrasi disimpan ke {filename}")
    
    def load_calibration_data(self, filename='mpu_calibration.json'):
        """
        Muat data kalibrasi dari file
        """
        import json
        
        try:
            with open(filename, 'r') as f:
                calibration_data = json.load(f)
            
            self.accel_offset = np.array(calibration_data['accel_offset'])
            self.accel_scale = np.array(calibration_data['accel_scale'])
            self.gyro_offset = np.array(calibration_data['gyro_offset'])
            self.gyro_scale = np.array(calibration_data['gyro_scale'])
            
            print(f"Data kalibrasi dimuat dari {filename}")
            
        except FileNotFoundError:
            print(f"File {filename} tidak ditemukan")
        except Exception as e:
            print(f"Error memuat data kalibrasi: {e}")


def main():
    """
    Contoh penggunaan kalibrator MPU
    """
    print("=== Kalibrasi Sensor MPU ===")
    
    # Inisialisasi kalibrator
    calibrator = MPUCalibrator()
    
    try:
        # Jalankan kalibrasi
        calibrator.calibrate_accelerometer()
        calibrator.calibrate_gyroscope()
        
        # Test kalibrasi
        calibrator.test_calibration(duration=5)
        
        # Plot hasil
        calibrator.plot_calibration_results()
        
        # Simpan data kalibrasi
        calibrator.save_calibration_data()
        
    except KeyboardInterrupt:
        print("\nKalibrasi dihentikan oleh user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        calibrator.bus.close()


if __name__ == "__main__":
    main()