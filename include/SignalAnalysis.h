#ifndef SIGNAL_ANALYSIS_H
#define SIGNAL_ANALYSIS_H

#include "Arduino.h"

// Forward declaration dari ModemManager.h
struct SignalInfo;

// ONLY define constants that are NOT in Config.h
// Constants yang sudah ada di Config.h akan digunakan dari sana

// Signal quality thresholds (CSQ) - Internal use only
#define CSQ_EXCELLENT_THRESHOLD 25
#define CSQ_GOOD_THRESHOLD      20
#define CSQ_FAIR_THRESHOLD      15
#define CSQ_POOR_THRESHOLD      10

// RSRQ thresholds - Internal naming untuk menghindari konflik
#define RSRQ_EXCELLENT_THRESHOLD -8.0f
#define RSRQ_GOOD_THRESHOLD_SA   -12.0f  // SA = SignalAnalysis
#define RSRQ_FAIR_THRESHOLD_SA   -15.0f
#define RSRQ_POOR_THRESHOLD_SA   -20.0f

// RSRP thresholds - Internal naming untuk menghindari konflik
#define RSRP_EXCELLENT_THRESHOLD -80.0f
#define RSRP_GOOD_THRESHOLD_SA   -90.0f  // SA = SignalAnalysis
#define RSRP_FAIR_THRESHOLD_SA   -100.0f
#define RSRP_POOR_THRESHOLD_SA   -110.0f

// Overall signal score thresholds
#define SIGNAL_SCORE_EXCELLENT   85.0f
#define SIGNAL_SCORE_GOOD        70.0f
#define SIGNAL_SCORE_FAIR        50.0f
#define SIGNAL_SCORE_POOR        30.0f

// ===== NAMESPACE SESUAI MODEMMANAGER.H =====
namespace SignalAnalysis {
    // Signal quality descriptions - fungsi utama yang dipanggil ModemManager
    String getSignalQualityDescription(int csq);
    String getRSRQQualityDescription(float rsrq);
    String getRSRPQualityDescription(float rsrp);
    
    // Signal analysis - fungsi analisis utama
    float calculateSignalScore(const SignalInfo& signalInfo);
    bool isSignalSuitableForOptimization(const SignalInfo& signalInfo);
    
    // Additional helper functions
    bool isSignalExcellent(int csq);
    bool isSignalGood(int csq);
    bool isSignalFair(int csq);
    bool isSignalPoor(int csq);
    
    // RSRQ quality checks
    bool isRSRQExcellent(float rsrq);
    bool isRSRQGood(float rsrq);
    bool isRSRQFair(float rsrq);
    bool isRSRQPoor(float rsrq);
    
    // RSRP quality checks
    bool isRSRPExcellent(float rsrp);
    bool isRSRPGood(float rsrp);
    bool isRSRPFair(float rsrp);
    bool isRSRPPoor(float rsrp);
    
    // Internal scoring functions
    float calculateCSQScore(int csq);
    float calculateRSRQScore(float rsrq);
    float calculateRSRPScore(float rsrp);
}

#endif // SIGNAL_ANALYSIS_H