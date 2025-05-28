// unit_tests.ino
/**
 * Unit Testing Framework for ESP32 System
 * Compile and upload this instead of main.ino to run tests
 */

#include <Arduino.h>
#include "Logger.h"
#include "Utils.h"
#include "Config.h"

// Test framework
class TestFramework {
private:
  static int testsRun;
  static int testsPassed;
  static int testsFailed;
  
public:
  static void begin() {
    Serial.begin(115200);
    delay(1000);
    Logger::init(&Serial, LOG_DEBUG);
    
    Serial.println("\n=== UNIT TEST FRAMEWORK ===");
    testsRun = 0;
    testsPassed = 0;
    testsFailed = 0;
  }
  
  static void end() {
    Serial.println("\n=== TEST SUMMARY ===");
    Serial.print("Tests run: ");
    Serial.println(testsRun);
    Serial.print("Passed: ");
    Serial.println(testsPassed);
    Serial.print("Failed: ");
    Serial.println(testsFailed);
    Serial.println("==================");
  }
  
  static void test(const char* name, bool condition) {
    testsRun++;
    Serial.print("[TEST] ");
    Serial.print(name);
    Serial.print(": ");
    
    if (condition) {
      Serial.println("PASS");
      testsPassed++;
    } else {
      Serial.println("FAIL");
      testsFailed++;
    }
  }
  
  static void assertEqual(const char* name, int expected, int actual) {
    test(name, expected == actual);
    if (expected != actual) {
      Serial.print("  Expected: ");
      Serial.print(expected);
      Serial.print(", Actual: ");
      Serial.println(actual);
    }
  }
  
  static void assertEqual(const char* name, const char* expected, const char* actual) {
    test(name, strcmp(expected, actual) == 0);
    if (strcmp(expected, actual) != 0) {
      Serial.print("  Expected: '");
      Serial.print(expected);
      Serial.print("', Actual: '");
      Serial.print(actual);
      Serial.println("'");
    }
  }
  
  static void assertFloat(const char* name, float expected, float actual, float tolerance = 0.001) {
    test(name, abs(expected - actual) < tolerance);
    if (abs(expected - actual) >= tolerance) {
      Serial.print("  Expected: ");
      Serial.print(expected, 6);
      Serial.print(", Actual: ");
      Serial.println(actual, 6);
    }
  }
};

int TestFramework::testsRun = 0;
int TestFramework::testsPassed = 0;
int TestFramework::testsFailed = 0;

// Test Logger
void testLogger() {
  Serial.println("\n--- Testing Logger ---");
  
  // Test log levels
  Logger::setLevel(LOG_ERROR);
  TestFramework::assertEqual("Log level set", LOG_ERROR, Logger::getLevel());
  
  Logger::setLevel(LOG_DEBUG);
  TestFramework::assertEqual("Log level set to debug", LOG_DEBUG, Logger::getLevel());
  
  // Test time string format
  const char* timeStr = Logger::getTimeString();
  TestFramework::test("Time string not null", timeStr != nullptr);
  TestFramework::test("Time string has brackets", timeStr[0] == '[');
}

// Test Utils
void testUtils() {
  Serial.println("\n--- Testing Utils ---");
  
  // Test ISO8601 formatting
  char buffer[30];
  Utils::formatISO8601(buffer, sizeof(buffer), 2025, 1, 15, 10, 30, 45, 7);
  TestFramework::assertEqual("ISO8601 format", "2025-01-15T17:30:45+07:00", buffer);
  
  // Test with hour overflow
  Utils::formatISO8601(buffer, sizeof(buffer), 2025, 1, 31, 23, 30, 45, 7);
  TestFramework::assertEqual("ISO8601 with day rollover", "2025-02-01T06:30:45+07:00", buffer);
  
  // Test uptime formatting
  String uptime = Utils::formatUptime(3661000); // 1h 1m 1s
  TestFramework::assertEqual("Uptime format", "1h 1m 1s", uptime.c_str());
  
  // Test signal quality strings
  TestFramework::assertEqual("Signal weak", "Weak", Utils::getSignalQualityString(5));
  TestFramework::assertEqual("Signal OK", "OK", Utils::getSignalQualityString(12));
  TestFramework::assertEqual("Signal good", "Good", Utils::getSignalQualityString(18));
  TestFramework::assertEqual("Signal excellent", "Excellent", Utils::getSignalQualityString(25));
  
  // Test HTTP error strings
  TestFramework::assertEqual("HTTP error -1", "CONNECTION_FAILED", 
                           Utils::getHttpErrorString(-1));
  TestFramework::assertEqual("HTTP error -3", "CONNECTION_REFUSED", 
                           Utils::getHttpErrorString(-3));
}

// Test retry operation
void testRetryOperation() {
  Serial.println("\n--- Testing Retry Operation ---");
  
  int attemptCount = 0;
  
  // Test successful retry
  bool result = Utils::retryOperation(
    "TEST",
    "successful operation",
    [&attemptCount]() {
      attemptCount++;
      return attemptCount == 2; // Succeed on second attempt
    },
    3,
    100
  );
  
  TestFramework::test("Retry succeeds on second attempt", result);
  TestFramework::assertEqual("Attempt count", 2, attemptCount);
  
  // Test failed retry
  attemptCount = 0;
  result = Utils::retryOperation(
    "TEST",
    "failing operation",
    [&attemptCount]() {
      attemptCount++;
      return false; // Always fail
    },
    3,
    100
  );
  
  TestFramework::test("Retry fails after max attempts", !result);
  TestFramework::assertEqual("Max attempts reached", 3, attemptCount);
}

// Mock GPS data for testing
class MockGpsData {
public:
  static void testGpsTimestamp() {
    Serial.println("\n--- Testing GPS Timestamp ---");
    
    char timestamp[30];
    
    // Test with valid GPS data
    // Would need actual GPS manager instance to test fully
    // This is a placeholder for the test structure
    
    TestFramework::test("GPS timestamp test placeholder", true);
  }
};

// Test configuration values
void testConfig() {
  Serial.println("\n--- Testing Configuration ---");
  
  TestFramework::assertEqual("GPS send interval", 5000, (int)GPS_SEND_INTERVAL);
  TestFramework::assertEqual("Relay check interval", 10000, (int)RELAY_CHECK_INTERVAL);
  TestFramework::assertEqual("Device ID", 1, DEVICE_ID);
  TestFramework::assertEqual("UTC offset", 7, UTC_OFFSET);
  TestFramework::test("Server host not empty", strlen(SERVER_HOST) > 0);
}

// Performance tests
void testPerformance() {
  Serial.println("\n--- Performance Tests ---");
  
  // Test watchdog feed time
  unsigned long start = millis();
  for (int i = 0; i < 100; i++) {
    Utils::feedWatchdog();
  }
  unsigned long duration = millis() - start;
  
  TestFramework::test("Watchdog feed < 10ms per 100 calls", duration < 10);
  
  // Test safe delay accuracy
  start = millis();
  Utils::safeDelay(100);
  duration = millis() - start;
  
  TestFramework::test("Safe delay accuracy", duration >= 100 && duration <= 110);
}

// Memory tests
void testMemory() {
  Serial.println("\n--- Memory Tests ---");
  
  int freeHeap = ESP.getFreeHeap();
  TestFramework::test("Free heap > 100KB", freeHeap > 100000);
  
  // Test string operations don't leak
  int heapBefore = ESP.getFreeHeap();
  for (int i = 0; i < 100; i++) {
    String test = "Test string " + String(i);
    test.toUpperCase();
  }
  int heapAfter = ESP.getFreeHeap();
  
  TestFramework::test("No significant memory leak", abs(heapBefore - heapAfter) < 1000);
}

// Main test setup
void setup() {
  TestFramework::begin();
  
  // Run all test suites
  testLogger();
  testUtils();
  testRetryOperation();
  testConfig();
  MockGpsData::testGpsTimestamp();
  testPerformance();
  testMemory();
  
  TestFramework::end();
}

void loop() {
  // Nothing to do in loop for tests
  delay(1000);
}