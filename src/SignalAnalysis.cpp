#include "SignalAnalysis.h"
#include "Config.h"        // Include Config.h untuk konstanta
#include "ModemManager.h"  // For SignalInfo struct

// ===== IMPLEMENTASI NAMESPACE SignalAnalysis =====

// CSQ Signal Quality Description
String SignalAnalysis::getSignalQualityDescription(int csq) {
    if (csq == 99) {
        return "UNKNOWN";
    } else if (csq >= CSQ_EXCELLENT_THRESHOLD) {
        return "EXCELLENT";
    } else if (csq >= CSQ_GOOD_THRESHOLD) {
        return "GOOD";
    } else if (csq >= CSQ_FAIR_THRESHOLD) {
        return "FAIR";
    } else if (csq >= CSQ_POOR_THRESHOLD) {
        return "POOR";
    } else {
        return "VERY POOR";
    }
}

// RSRQ Quality Description
String SignalAnalysis::getRSRQQualityDescription(float rsrq) {
    if (rsrq == RSRQ_INVALID_VALUE) {
        return "INVALID";
    } else if (rsrq >= RSRQ_EXCELLENT_THRESHOLD) {
        return "EXCELLENT";
    } else if (rsrq >= RSRQ_GOOD_THRESHOLD) {
        return "GOOD";
    } else if (rsrq >= RSRQ_FAIR_THRESHOLD) {
        return "FAIR";
    } else if (rsrq >= RSRQ_POOR_THRESHOLD) {
        return "POOR";
    } else {
        return "VERY POOR";
    }
}

// RSRP Quality Description
String SignalAnalysis::getRSRPQualityDescription(float rsrp) {
    if (rsrp == RSRP_INVALID_VALUE) {
        return "INVALID";
    } else if (rsrp >= RSRP_EXCELLENT_THRESHOLD) {
        return "EXCELLENT";
    } else if (rsrp >= RSRP_GOOD_THRESHOLD) {
        return "GOOD";
    } else if (rsrp >= RSRP_FAIR_THRESHOLD) {
        return "FAIR";
    } else if (rsrp >= RSRP_POOR_THRESHOLD) {
        return "POOR";
    } else {
        return "VERY POOR";
    }
}

// Calculate overall signal score (0-100)
float SignalAnalysis::calculateSignalScore(const SignalInfo& signalInfo) {
    float totalScore = 0.0f;
    int validMetrics = 0;
    
    // CSQ Score (weight: 40%)
    if (signalInfo.csq != 99) {
        totalScore += calculateCSQScore(signalInfo.csq) * 0.4f;
        validMetrics++;
    }
    
    // RSRQ Score (weight: 30%)
    if (signalInfo.rsrqValid) {
        totalScore += calculateRSRQScore(signalInfo.rsrq) * 0.3f;
        validMetrics++;
    }
    
    // RSRP Score (weight: 30%)
    if (signalInfo.rsrpValid) {
        totalScore += calculateRSRPScore(signalInfo.rsrp) * 0.3f;
        validMetrics++;
    }
    
    // If only CSQ is available, use full weight
    if (validMetrics == 1 && signalInfo.csq != 99) {
        return calculateCSQScore(signalInfo.csq);
    }
    
    // If no valid metrics, return 0
    if (validMetrics == 0) {
        return 0.0f;
    }
    
    // Normalize score based on available metrics
    return totalScore / (validMetrics * 0.333f); // Adjust for weight distribution
}

// Check if signal is suitable for optimization (FIXED FOR COMPATIBILITY)
bool SignalAnalysis::isSignalSuitableForOptimization(const SignalInfo& signalInfo) {
    float score = calculateSignalScore(signalInfo);
    
    // Consider signal suitable if:
    // 1. Overall score >= 50 (FAIR or better)
    // 2. CSQ >= 10 (minimum usable signal) - use konstanta dari Config.h
    // 3. If LTE metrics available, RSRP >= -110 dBm
    
    if (score < SIGNAL_SCORE_FAIR) {
        return false;
    }
    
    if (signalInfo.csq < SIGNAL_WEAK_THRESHOLD) {  // From Config.h
        return false;
    }
    
    if (signalInfo.rsrpValid && signalInfo.rsrp < RSRP_POOR_THRESHOLD) {  // From Config.h
        return false;
    }
    
    return true;
}

// CSQ threshold checks (NAMESPACE FUNCTIONS)
bool SignalAnalysis::isSignalExcellent(int csq) {
    return (csq >= CSQ_EXCELLENT_THRESHOLD && csq != 99);
}

bool SignalAnalysis::isSignalGood(int csq) {
    return (csq >= CSQ_GOOD_THRESHOLD && csq < CSQ_EXCELLENT_THRESHOLD);
}

bool SignalAnalysis::isSignalFair(int csq) {
    return (csq >= CSQ_FAIR_THRESHOLD && csq < CSQ_GOOD_THRESHOLD);
}

bool SignalAnalysis::isSignalPoor(int csq) {
    return (csq >= CSQ_POOR_THRESHOLD && csq < CSQ_FAIR_THRESHOLD);
}

// RSRQ threshold checks (NAMESPACE FUNCTIONS)
bool SignalAnalysis::isRSRQExcellent(float rsrq) {
    return (rsrq >= RSRQ_EXCELLENT_THRESHOLD && rsrq != RSRQ_INVALID_VALUE);
}

bool SignalAnalysis::isRSRQGood(float rsrq) {
    return (rsrq >= RSRQ_GOOD_THRESHOLD_SA && rsrq < RSRQ_EXCELLENT_THRESHOLD);
}

bool SignalAnalysis::isRSRQFair(float rsrq) {
    return (rsrq >= RSRQ_FAIR_THRESHOLD_SA && rsrq < RSRQ_GOOD_THRESHOLD_SA);
}

bool SignalAnalysis::isRSRQPoor(float rsrq) {
    return (rsrq >= RSRQ_POOR_THRESHOLD_SA && rsrq < RSRQ_FAIR_THRESHOLD_SA);
}

// RSRP threshold checks (NAMESPACE FUNCTIONS)
bool SignalAnalysis::isRSRPExcellent(float rsrp) {
    return (rsrp >= RSRP_EXCELLENT_THRESHOLD && rsrp != RSRP_INVALID_VALUE);
}

bool SignalAnalysis::isRSRPGood(float rsrp) {
    return (rsrp >= RSRP_GOOD_THRESHOLD_SA && rsrp < RSRP_EXCELLENT_THRESHOLD);
}

bool SignalAnalysis::isRSRPFair(float rsrp) {
    return (rsrp >= RSRP_FAIR_THRESHOLD_SA && rsrp < RSRP_GOOD_THRESHOLD_SA);
}

bool SignalAnalysis::isRSRPPoor(float rsrp) {
    return (rsrp >= RSRP_POOR_THRESHOLD_SA && rsrp < RSRP_FAIR_THRESHOLD_SA);
}

// Internal scoring functions (NAMESPACE FUNCTIONS)
float SignalAnalysis::calculateCSQScore(int csq) {
    if (csq == 99) return 0.0f;
    
    // Convert CSQ (0-31) to score (0-100)
    // CSQ 31 = 100%, CSQ 0 = 0%
    float score = (float(csq) / 31.0f) * 100.0f;
    
    // Apply quality curve (better scores for higher CSQ)
    if (csq >= CSQ_EXCELLENT_THRESHOLD) {
        score = 90.0f + (score - 80.6f) * 0.5f; // 90-100%
    } else if (csq >= CSQ_GOOD_THRESHOLD) {
        score = 70.0f + (score - 64.5f) * 1.25f; // 70-90%
    } else if (csq >= CSQ_FAIR_THRESHOLD) {
        score = 50.0f + (score - 48.4f) * 1.25f; // 50-70%
    } else if (csq >= CSQ_POOR_THRESHOLD) {
        score = 25.0f + (score - 32.3f) * 1.5f; // 25-50%
    } else {
        score = score * 0.8f; // 0-25%
    }
    
    return constrain(score, 0.0f, 100.0f);
}

float SignalAnalysis::calculateRSRQScore(float rsrq) {
    if (rsrq == RSRQ_INVALID_VALUE) return 0.0f;
    
    // RSRQ range: -34 to -1 dB (higher is better)
    // Convert to 0-100 scale
    float normalizedRsrq = (rsrq + 34.0f) / 33.0f; // 0.0 to 1.0
    float score = normalizedRsrq * 100.0f;
    
    // Apply quality curve using SA constants
    if (rsrq >= RSRQ_EXCELLENT_THRESHOLD) {
        score = 90.0f + (score - 78.8f) * 0.5f;
    } else if (rsrq >= RSRQ_GOOD_THRESHOLD_SA) {
        score = 70.0f + (score - 66.7f) * 1.6f;
    } else if (rsrq >= RSRQ_FAIR_THRESHOLD_SA) {
        score = 50.0f + (score - 57.6f) * 2.2f;
    } else if (rsrq >= RSRQ_POOR_THRESHOLD_SA) {
        score = 25.0f + (score - 42.4f) * 1.6f;
    } else {
        score = score * 0.6f;
    }
    
    return constrain(score, 0.0f, 100.0f);
}

float SignalAnalysis::calculateRSRPScore(float rsrp) {
    if (rsrp == RSRP_INVALID_VALUE) return 0.0f;
    
    // RSRP range: -141 to -44 dBm (higher is better)
    // Convert to 0-100 scale
    float normalizedRsrp = (rsrp + 141.0f) / 97.0f; // 0.0 to 1.0
    float score = normalizedRsrp * 100.0f;
    
    // Apply quality curve using SA constants
    if (rsrp >= RSRP_EXCELLENT_THRESHOLD) {
        score = 90.0f + (score - 62.9f) * 0.6f;
    } else if (rsrp >= RSRP_GOOD_THRESHOLD_SA) {
        score = 70.0f + (score - 52.6f) * 1.9f;
    } else if (rsrp >= RSRP_FAIR_THRESHOLD_SA) {
        score = 50.0f + (score - 42.3f) * 1.9f;
    } else if (rsrp >= RSRP_POOR_THRESHOLD_SA) {
        score = 25.0f + (score - 32.0f) * 2.4f;
    } else {
        score = score * 0.8f;
    }
    
    return constrain(score, 0.0f, 100.0f);
}