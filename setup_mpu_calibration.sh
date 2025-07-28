#!/bin/bash

# Script Setup Kalibrasi MPU
# Untuk Raspberry Pi dan sistem Linux

echo "=== Setup Kalibrasi Sensor MPU ==="

# Update sistem
echo "1. Update sistem..."
sudo apt-get update
sudo apt-get upgrade -y

# Install dependensi sistem
echo "2. Install dependensi sistem..."
sudo apt-get install -y python3-pip python3-dev python3-venv
sudo apt-get install -y i2c-tools libi2c-dev
sudo apt-get install -y git

# Enable I2C (untuk Raspberry Pi)
if [ -f /boot/config.txt ]; then
    echo "3. Enable I2C interface..."
    if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
        echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
    fi
    if ! grep -q "i2c-dev" /etc/modules; then
        echo "i2c-dev" | sudo tee -a /etc/modules
    fi
fi

# Buat virtual environment
echo "4. Buat virtual environment..."
python3 -m venv mpu_env
source mpu_env/bin/activate

# Install Python packages
echo "5. Install Python packages..."
pip install --upgrade pip
pip install -r requirements.txt

# Test I2C
echo "6. Test I2C interface..."
if command -v i2cdetect &> /dev/null; then
    echo "Scanning I2C devices..."
    i2cdetect -y 1
else
    echo "i2cdetect tidak tersedia"
fi

# Buat direktori untuk data kalibrasi
echo "7. Buat direktori data..."
mkdir -p calibration_data
mkdir -p logs

# Set permission untuk I2C
echo "8. Set permission I2C..."
sudo usermod -a -G i2c $USER

echo ""
echo "=== Setup Selesai ==="
echo ""
echo "Langkah selanjutnya:"
echo "1. Reboot sistem: sudo reboot"
echo "2. Aktifkan virtual environment: source mpu_env/bin/activate"
echo "3. Jalankan kalibrasi: python mpu_calibration_python.py"
echo ""
echo "Tips:"
echo "- Pastikan sensor MPU terhubung dengan benar"
echo "- Cek koneksi dengan: i2cdetect -y 1"
echo "- Sensor MPU biasanya terdeteksi di alamat 0x68 atau 0x69"