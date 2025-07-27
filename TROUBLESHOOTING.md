# Troubleshooting GSM Modem A7670C

## Masalah Koneksi Jaringan

### 1. **SIM Card Tidak Terdeteksi**
**Gejala:** Log menunjukkan "SIM failure" atau "SIM card gagal terdeteksi"

**Solusi:**
- Pastikan SIM card terpasang dengan benar
- Bersihkan kontak SIM card dengan kain lembut
- Coba SIM card lain untuk memastikan hardware tidak bermasalah
- Periksa apakah SIM card sudah diaktifkan oleh operator

### 2. **Sinyal Lemah**
**Gejala:** Signal quality < 10 atau "Sinyal lemah"

**Solusi:**
- Pindahkan device ke area dengan sinyal lebih kuat
- Pastikan antena terpasang dengan benar
- Hindari area dengan banyak penghalang (gedung, basement)
- Coba restart modem dengan `modemManager.startReset()`

### 3. **Gagal Registrasi Jaringan**
**Gejala:** "Tidak terdaftar di jaringan" atau "Mencari jaringan..."

**Solusi:**
- Periksa apakah SIM card sudah terdaftar di jaringan operator
- Pastikan tidak ada masalah dengan operator (maintenance, gangguan)
- Coba restart modem
- Periksa konfigurasi APN sesuai operator

### 4. **Gagal Koneksi GPRS**
**Gejala:** "Gagal koneksi GPRS" atau "PDP context tidak aktif"

**Solusi:**
- Periksa konfigurasi APN di `Config.h`
- Pastikan paket data aktif di SIM card
- Coba APN alternatif untuk operator Anda
- Jalankan diagnostik jaringan dengan `modemManager.performNetworkDiagnostic()`

## Konfigurasi APN per Operator

### Telkomsel
```cpp
#define APN "internet"        // APN utama
// atau
#define APN "telkomsel"       // APN alternatif
```

### Indosat
```cpp
#define APN "internet"        // APN utama
// atau
#define APN "indosat"         // APN alternatif
```

### XL/Axis
```cpp
#define APN "internet"        // APN utama
// atau
#define APN "xl"              // APN alternatif
```

### 3 (Tri)
```cpp
#define APN "3"               // APN utama
// atau
#define APN "internet"        // APN alternatif
```

### Smartfren
```cpp
#define APN "smartfren"       // APN utama
```

## Perintah Debug

### 1. Jalankan Diagnostik Lengkap
```cpp
modemManager.performNetworkDiagnostic();
```

### 2. Cek Status Modem
```cpp
LOG_INFO("Status: %d", modemManager.getStatus());
LOG_INFO("Signal: %d", modemManager.getSignalQuality());
LOG_INFO("Operator: %s", modemManager.getOperator().c_str());
```

### 3. Reset Modem
```cpp
modemManager.startReset();
while (modemManager.continueReset()) {
  delay(100);
}
```

### 4. Cek Konfigurasi APN
```cpp
modemManager.sendATCommand("AT+CGDCONT?");
String response = modemManager.readATResponse(2000);
LOG_INFO("APN Config: %s", response.c_str());
```

## Optimasi Performa

### 1. **Mode Jaringan**
- **Auto Mode (Rekomendasi):** `AT+CNMP=2` - Mendukung 2G/3G/4G
- **LTE Only:** `AT+CNMP=38` - Hanya 4G (bisa gagal jika sinyal 4G lemah)
- **3G Only:** `AT+CNMP=13` - Hanya 3G

### 2. **Band Configuration**
- **Default:** `AT+CBAND="DEFAULT"` - Menggunakan band default operator
- **All Bands:** `AT+CBAND="ALL"` - Semua band (bisa tidak stabil)

### 3. **Power Management**
- **Normal:** Biarkan fast dormancy aktif untuk hemat baterai
- **Always On:** Nonaktifkan fast dormancy untuk koneksi lebih stabil

## Log Analysis

### Log Normal (Berhasil)
```
[INFO] ✅ SIM card siap
[INFO] Modem Info: Manufacturer: SIMCOM INCORPORATED Model: A7670C-LNNV
[INFO] IMEI: 861192078344937
[INFO] Menunggu koneksi jaringan...
[DEBUG] Signal quality: 15
[INFO] ✅ Jaringan terhubung
[INFO] ✅ GPRS terhubung
```

### Log Bermasalah
```
[ERROR] ❌ SIM card gagal terdeteksi
[WARN] ⚠️ Sinyal lemah, coba pindah ke area yang lebih baik
[ERROR] ❌ Tidak terdaftar di jaringan
[ERROR] ❌ Gagal koneksi GPRS
```

## Tips Tambahan

1. **Restart Berkala:** Restart modem setiap 24-72 jam untuk refresh koneksi
2. **Monitoring Sinyal:** Pantau kualitas sinyal secara berkala
3. **Backup APN:** Siapkan beberapa konfigurasi APN untuk testing
4. **Log Level:** Gunakan log level DEBUG untuk troubleshooting detail
5. **Hardware Check:** Periksa koneksi kabel dan power supply secara berkala

## Emergency Recovery

Jika semua solusi di atas gagal:

1. **Hard Reset:** Restart ESP32 dan modem
2. **Factory Reset:** Reset konfigurasi ke default
3. **SIM Replacement:** Coba SIM card lain
4. **Hardware Check:** Periksa hardware modem
5. **Operator Support:** Hubungi operator untuk bantuan