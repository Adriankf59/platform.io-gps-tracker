# Offline Backup & Auto-Sync Guide

## **Fitur Backup Otomatis**

Sistem GPS Tracker ESP32 sekarang memiliki fitur **backup otomatis** yang menyimpan data GPS saat jaringan tidak tersedia dan mengirimkannya secara otomatis saat jaringan kembali tersedia.

## **Cara Kerja**

### **1. Mode Online (Jaringan Tersedia)**
- Data GPS langsung dikirim ke server via WebSocket
- Jika pengiriman gagal, data disimpan offline sebagai backup

### **2. Mode Offline (Jaringan Tidak Tersedia)**
- Data GPS disimpan di SPIFFS (flash memory)
- Maksimal 100 records (sekitar 20KB)
- Data tetap aman meskipun ESP32 restart

### **3. Auto-Sync (Jaringan Kembali)**
- Sistem otomatis mendeteksi ketersediaan jaringan
- Backup data dikirim secara bertahap (batch)
- Retry otomatis jika pengiriman gagal

## **Konfigurasi**

### **Enable/Disable Fitur**
```cpp
// Di Config.h
#define ENABLE_OFFLINE_STORAGE true    // Aktifkan backup offline
#define OFFLINE_AUTO_SYNC true         // Aktifkan auto-sync
```

### **Pengaturan Storage**
```cpp
#define OFFLINE_MAX_RECORDS 100        // Maksimal records
#define OFFLINE_SYNC_BATCH_SIZE 5      // Records per batch
#define OFFLINE_SYNC_INTERVAL 2000     // Interval antar batch (ms)
#define OFFLINE_SYNC_RETRY_INTERVAL 30000  // Retry interval (30s)
```

## **Monitoring & Control**

### **Command Serial**

#### **1. Cek Status Backup**
```
offline
```
**Output:**
```
📦 Offline Storage Status:
   Records stored: 15/100
   Records sent: 10
   Records pending: 5
   Storage used: 3.2KB/960KB
   Last sync: 2 minutes ago
```

#### **2. Cek Statistik Detail**
```
offline_stats
```
**Output:**
```
📊 Offline Storage Statistics:
   Total stored: 45 records
   Successfully sent: 40 records
   Failed sends: 5 records
   Oldest record: 2024-01-15 10:30:00
   Newest record: 2024-01-15 14:45:00
   Storage efficiency: 95.6%
```

#### **3. Manual Sync**
```
offline_sync
```
**Output:**
```
🔄 Manual offline sync triggered
📤 Sending batch 1/5...
✅ Sent 5 records
📤 Sending batch 2/5...
✅ Sent 5 records
✅ Manual sync completed successfully
```

#### **4. Clear Backup Data**
```
offline_clear
```
**Output:**
```
🗑️ Clearing all offline data...
✅ All offline data cleared
```

#### **5. Test Backup System**
```
offline_test
```
**Output:**
```
🧪 Running offline storage stress test...
📦 Storing test data...
📤 Simulating network outage...
🔄 Testing auto-sync...
✅ Stress test completed successfully
```

## **Log Monitoring**

### **Log Normal (Backup Berfungsi)**
```
[INFO] 📦 Storing GPS data offline (network unavailable)
[INFO] 💾 Offline record stored: lat=-6.123456, lng=106.789012
[INFO] 📶 Network available - switching to ONLINE mode
[INFO] 🔄 Network available, triggering offline data sync...
[INFO] 📤 Sending batch 1/5 (5 records)
[INFO] ✅ Sent offline record 0: -6.123456, 106.789012, speed: 25.5
[INFO] ✅ Auto-sync completed: 15 records sent
```

### **Log Error (Perlu Perhatian)**
```
[WARN] ⚠️ Offline storage full! 100 records stored
[ERROR] ❌ Failed to send offline record 5 (attempt 2)
[WARN] ⚠️ Too many consecutive failures, stopping auto-sync
[ERROR] ❌ Offline storage error: SPIFFS write failed
```

## **Troubleshooting**

### **1. Backup Tidak Tersimpan**
**Gejala:** Data hilang saat jaringan down

**Solusi:**
- Cek `ENABLE_OFFLINE_STORAGE` sudah `true`
- Cek SPIFFS space: `memory` command
- Restart ESP32 untuk refresh storage

### **2. Auto-Sync Tidak Berfungsi**
**Gejala:** Backup data tidak terkirim saat jaringan tersedia

**Solusi:**
- Cek `ENABLE_OFFLINE_AUTO_SYNC` sudah `true`
- Cek WebSocket connection: `wsstats` command
- Manual sync: `offline_sync` command

### **3. Storage Penuh**
**Gejala:** "Offline storage full" warning

**Solusi:**
- Tunggu auto-sync selesai
- Manual sync: `offline_sync` command
- Clear old data: `offline_clear` command
- Cek apakah ada data lama yang tidak terkirim

### **4. Pengiriman Gagal**
**Gejala:** "Failed to send offline record"

**Solusi:**
- Cek koneksi jaringan: `network` command
- Cek WebSocket status: `wsstats` command
- Restart WebSocket: `wsreset` command
- Cek signal quality: `status` command

## **Optimasi Performa**

### **1. Batch Size**
- **Kecil (3-5):** Lebih reliable, lebih lambat
- **Besar (10-15):** Lebih cepat, bisa gagal jika jaringan lemah

### **2. Sync Interval**
- **Pendek (1-2s):** Sync lebih cepat, lebih banyak traffic
- **Panjang (5-10s):** Hemat bandwidth, sync lebih lambat

### **3. Retry Strategy**
- **Max Retries:** 3 (default)
- **Retry Interval:** 30 detik
- **Consecutive Failures:** Stop setelah 3 kali gagal

## **Monitoring Real-time**

### **Status Indicators**
- **📦** = Data tersimpan offline
- **📤** = Sedang mengirim backup
- **✅** = Backup berhasil dikirim
- **❌** = Backup gagal dikirim
- **⚠️** = Warning (storage penuh, dll)

### **Metrics to Watch**
1. **Records stored vs sent**
2. **Storage usage percentage**
3. **Sync success rate**
4. **Time since last sync**
5. **Failed send attempts**

## **Best Practices**

### **1. Regular Monitoring**
- Cek status backup setiap hari: `offline` command
- Monitor storage usage secara berkala
- Review failed sends untuk troubleshooting

### **2. Maintenance**
- Clear old data secara berkala jika tidak diperlukan
- Restart ESP32 setiap 72 jam untuk refresh storage
- Monitor SPIFFS health dengan `memory` command

### **3. Network Optimization**
- Pastikan WebSocket connection stabil
- Monitor signal quality untuk area dengan sinyal lemah
- Gunakan APN yang tepat untuk operator

### **4. Testing**
- Test backup system secara berkala: `offline_test`
- Simulate network outage untuk verifikasi
- Monitor auto-sync behavior di berbagai kondisi

## **Emergency Procedures**

### **Jika Backup System Bermasalah**
1. **Immediate:** Manual sync dengan `offline_sync`
2. **Short-term:** Clear data dengan `offline_clear`
3. **Long-term:** Restart ESP32 dan reinitialize storage

### **Jika Storage Corrupted**
1. Clear all data: `offline_clear`
2. Restart ESP32
3. Check SPIFFS: `memory` command
4. Reinitialize jika perlu

### **Jika Auto-Sync Stuck**
1. Check network: `network` command
2. Reset WebSocket: `wsreset` command
3. Manual sync: `offline_sync` command
4. Restart ESP32 jika masih bermasalah