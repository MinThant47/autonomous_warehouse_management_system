#include <WiFi.h>
#include <HTTPClient.h>

// IMPORTANT: Update this if your Python server's IP address is different
const char* SERVER_URL = "http://192.168.116.49:5000/log";

struct AgvLogData {
    unsigned long startTime;
    unsigned long totalRunTime;
    
    // PID Metrics
    float totalAbsoluteError;
    unsigned long errorSampleCount;
    float avgError;
    float maxError;
    
    // Oscillation & Motor Effort
    unsigned long zeroCrossings;
    float lastErrorSign;
    float totalMotorCorrection;
    float avgMotorCorrection;
    
    // Navigation & Tasks
    int rfidCount;
    
    // Network Timing
    unsigned long waitStartTime;
    unsigned long totalWaitTime;

    // Placeholder for future task timing
    unsigned long currentTaskStartTime;
    unsigned long totalTaskTime; 
};

AgvLogData loggerState;

void startDataLogging() {
    loggerState.startTime = millis();
    loggerState.totalRunTime = 0;
    
    loggerState.totalAbsoluteError = 0.0;
    loggerState.errorSampleCount = 0;
    loggerState.avgError = 0.0;
    loggerState.maxError = 0.0;
    
    loggerState.zeroCrossings = 0;
    loggerState.lastErrorSign = 0.0;
    loggerState.totalMotorCorrection = 0.0;
    loggerState.avgMotorCorrection = 0.0;
    
    loggerState.rfidCount = 0;
    loggerState.totalWaitTime = 0;
    loggerState.totalTaskTime = 0;
    
    Serial.println("Data logging started.");
}

void updateDataLogging(float currentError, float currentCorrection) {
    float absError = abs(currentError);
    
    loggerState.totalAbsoluteError += absError;
    loggerState.totalMotorCorrection += abs(currentCorrection);
    loggerState.errorSampleCount++;
    
    if (absError > loggerState.maxError) {
        loggerState.maxError = absError;
    }
    
    // Track Oscillation (Zero-Crossings)
    if (currentError != 0) {
        float currentSign = (currentError > 0) ? 1.0 : -1.0;
        if (loggerState.lastErrorSign != 0 && currentSign != loggerState.lastErrorSign) {
            loggerState.zeroCrossings++;
        }
        loggerState.lastErrorSign = currentSign;
    }
}

void startWaitTimer() {
    loggerState.waitStartTime = millis();
}

void stopWaitTimer() {
    loggerState.totalWaitTime += (millis() - loggerState.waitStartTime);
}

void startTaskTimer() {
    loggerState.currentTaskStartTime = millis();
}

void stopTaskTimer() {
    loggerState.totalTaskTime += (millis() - loggerState.currentTaskStartTime);
}

void incrementRFIDCount() {
    loggerState.rfidCount++;
}

void finishDataLogging() {
    loggerState.totalRunTime = millis() - loggerState.startTime;
    
    if (loggerState.errorSampleCount > 0) {
        loggerState.avgError = loggerState.totalAbsoluteError / loggerState.errorSampleCount;
        loggerState.avgMotorCorrection = loggerState.totalMotorCorrection / loggerState.errorSampleCount;
    } else {
        loggerState.avgError = 0.0;
        loggerState.avgMotorCorrection = 0.0;
    }
    
    Serial.println("Data logging finished.");
}

bool sendDataToServer() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Error: WiFi not connected.");
        return false;
    }

    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");

    String jsonPayload = "{";
    jsonPayload += "\"avg_error\":" + String(loggerState.avgError, 4) + ",";
    jsonPayload += "\"max_error\":" + String(loggerState.maxError, 4) + ",";
    jsonPayload += "\"zero_crossings\":" + String(loggerState.zeroCrossings) + ",";
    jsonPayload += "\"avg_correction\":" + String(loggerState.avgMotorCorrection, 4) + ",";
    jsonPayload += "\"rfid_count\":" + String(loggerState.rfidCount) + ",";
    jsonPayload += "\"wait_time\":" + String(loggerState.totalWaitTime) + ",";
    jsonPayload += "\"run_time\":" + String(loggerState.totalRunTime);
    jsonPayload += "}";

    int httpResponseCode = http.POST(jsonPayload);
    bool isSuccessful = false;

    if (httpResponseCode > 0) {
        Serial.print("HTTP POST Success. Code: ");
        Serial.println(httpResponseCode);
        isSuccessful = true;
    } else {
        Serial.print("HTTP POST Error: ");
        Serial.println(http.errorToString(httpResponseCode).c_str());
    }

    http.end(); 
    return isSuccessful;
}