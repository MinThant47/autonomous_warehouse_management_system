//=====================================================
// LIBRARIES
//=====================================================
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h> 
#include "DataLogger.h" 

//=====================================================
// 1. ENCODER CALIBRATION SETTINGS (VERSION 2)
//=====================================================
#define ENCODER_L_PIN 3  // <-- Update this to your real Left Encoder OUT A pin
#define ENCODER_R_PIN 4  // <-- Update this to your real Right Encoder OUT A pin

// *** INSERT YOUR MAGIC NUMBER HERE LATER ***
// How many encoder ticks does it take for the wheels to execute a perfect 90-degree pivot?
long TICKS_FOR_90_DEGREE = 170; 

// Volatile variables because they are updated in the background by interrupts
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Interrupt Service Routines (ISRs) - These count the ticks instantly
void IRAM_ATTR leftEncoderISR() { leftTicks++; }
void IRAM_ATTR rightEncoderISR() { rightTicks++; }

//=====================================================
// 2. USER SETTINGS
//=====================================================
// Network Credentials
const char* WIFI_SSID = "cam";
const char* WIFI_PASSWORD = "screenshare123";
const char* MQTT_HOST = "192.168.116.49";  
const int MQTT_PORT = 1883;

// MQTT Topics
const char* TOPIC_PUBLISH_NODE = "agv/R1/node";
const char* TOPIC_SUBSCRIBE_CMD = "agv/R1/command";
const char* TOPIC_TELEMETRY = "agv/R1/telemetry"; // New Telemetry Topic

// PID 
float Kp = 20.0;
float Ki = 0.0;
float Kd = 20.0;

bool DEBUG = true;
unsigned long stopImmunityTimer = 0;

// Speed 
#define BASE_SPEED 180
#define TURN_SPEED 195 

#define MAX_PWM 255
#define MIN_PWM 0
#define LINE_DETECTED HIGH

//=====================================================
// PIN DEFINITIONS
//=====================================================
// Motors
#define L_IN1 8
#define L_IN2 9
#define L_PWM 10
#define R_IN1 11
#define R_IN2 12
#define R_PWM 13

// Sensors
#define S1 18
#define S2 19
#define S3 1
#define S4 20
#define S5 7

// RFID
#define RFID_SS   6
#define RFID_RST  40
#define RFID_SCK  36
#define RFID_MISO 37
#define RFID_MOSI 35

//=====================================================
// ENUMS & GLOBALS
//=====================================================
enum RobotState {
    FOLLOW_LINE,
    TURNING,
    STOPPED,
    WAITING_FOR_SERVER 
};

enum RobotCommand {
  CMD_NONE,
  CMD_STOP,
  CMD_LEFT,
  CMD_RIGHT,
  CMD_STRAIGHT,
  CMD_TURN_BACK
};

RobotState state = FOLLOW_LINE;
RobotCommand currentCommand = CMD_NONE;

// Network Objects
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// PID & Motor variables
float error = 0, previousError = 0, integral = 0, derivative = 0, correction = 0;
int currentLeftPWM = 0;  // Added for telemetry
int currentRightPWM = 0; // Added for telemetry
unsigned long lastReconnectAttempt = 0;
int sensorValue[5];

// Telemetry Timing
unsigned long lastTelemetryTime = 0;
const int TELEMETRY_INTERVAL = 500; // Broadcast twice a second

// RFID object
MFRC522 rfid(RFID_SS, RFID_RST);
String lastUID = "";

// Route context supplied by the backend with each command.  The backend path
// includes the RFID node just confirmed, so this count represents only nodes
// still ahead of the AGV.  It lets an all-white line end be acknowledged only
// on the final, non-RFID leg of a route.
String activeGoal = "";
int remainingPathNodes = 0;
bool terminalArrivalPending = false;
bool terminalArrivalReported = false;

void stopMotor();
void executeCommand();
void printState();
bool publishNodeReport(const char* identifierKey, const String& identifier, const char* eventType = nullptr);

//=====================================================
// NETWORK & TELEMETRY FUNCTIONS
//=====================================================
void setupWiFi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("\n--- MQTT COMMAND RECEIVED ---");
    Serial.print("\nTopic: ");
    Serial.println(topic);

    // 1. Capture the raw payload as a String for safe handling and debugging
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.print("Raw Payload: ");
    Serial.println(message);

    String action = "";
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    // 2. Extract action and retain the route context supplied by the backend.
    // Plain-text commands remain supported for manual testing, but cannot arm
    // a virtual final-arrival report because they contain no route plan.
    if (!error && doc.is<JsonObject>() && doc.containsKey("action")) {
        action = doc["action"].as<String>();
        activeGoal = doc["goal"].is<const char*>() ? doc["goal"].as<String>() : "";

        JsonArray path = doc["path"].as<JsonArray>();
        remainingPathNodes = (path.isNull() || path.size() == 0)
            ? 0
            : static_cast<int>(path.size()) - 1;
        terminalArrivalPending = false;
        terminalArrivalReported = false;

        // Example command at Yellow_3_J:
        // path = ["Yellow_3_J", "Yellow_3"] -> one remaining node, Yellow_3.
        if (remainingPathNodes == 1 && activeGoal.length() > 0) {
            String remainingNode = path[1].as<String>();
            terminalArrivalPending = (remainingNode == activeGoal);
        }
        Serial.println("Format: JSON parsed successfully.");
    } else {
        Serial.println("Format: Not JSON. Falling back to plain text.");
        action = message;
        activeGoal = "";
        remainingPathNodes = 0;
        terminalArrivalPending = false;
        terminalArrivalReported = false;
    }

    action.trim(); // Remove any hidden newline characters (\n or \r)
    action.toUpperCase(); 
    Serial.print("Final Parsed Action: ");
    Serial.println(action);

    // 3. Fix the State Machine Lockout
    // Now accepts commands if it's waiting at an RFID tag OR sitting idle/stopped
    if (state == WAITING_FOR_SERVER || state == STOPPED) {
        
        if (action == "LEFT") currentCommand = CMD_LEFT;
        else if (action == "RIGHT") currentCommand = CMD_RIGHT;
        // Added "START" as a valid synonym to trigger the robot to move forward
        else if (action == "STRAIGHT" || action == "FORWARD" || action == "FOLLOW_LINE" || action == "FOLLOW LINE" || action == "START") currentCommand = CMD_STRAIGHT;
        else if (action == "TURN_BACK") currentCommand = CMD_TURN_BACK;
        else if (action == "STOP") currentCommand = CMD_STOP;
        else {
            Serial.println("Status: Invalid action name. Ignored.");
            return;
        }
        
        // Only trigger this if we were actively waiting on a tag read
        if (state == WAITING_FOR_SERVER) {
            stopWaitTimer(); 
        }
        
        Serial.println("Status: Command Accepted! Executing...");
        executeCommand();
        
    } else {
        Serial.print("Status: Command Ignored. Robot is actively busy. Current State ID: ");
        Serial.println(state);
    }
    Serial.println("-----------------------------\n");
}

boolean reconnectMqtt() {
    if (mqtt.connect("agv-r1-esp32")) {
        Serial.println("Connected to MQTT broker");
        mqtt.subscribe(TOPIC_SUBSCRIBE_CMD);
    }
    return mqtt.connected();
}

void publishTelemetry() {
    JsonDocument doc;

    // 1. Robot Status
    switch(state) {
        case FOLLOW_LINE: doc["status"] = "FOLLOW_LINE"; break;
        case TURNING: doc["status"] = "TURNING"; break;
        case STOPPED: doc["status"] = "STOPPED"; break;
        case WAITING_FOR_SERVER: doc["status"] = "WAITING_FOR_SERVER"; break;
    }

    // 2. IR Sensors (Creates a JSON Array: [0, 1, 0, 0, 0])
    JsonArray ir_array = doc["ir_sensors"].to<JsonArray>();
    for(int i = 0; i < 5; i++) {
        ir_array.add(sensorValue[i]);
    }

    // 3. RFID Status
    doc["last_rfid"] = (lastUID == "") ? "NONE" : lastUID;

    // 4. PID Values
    doc["error"] = error;
    doc["correction"] = correction;

    // 5. Motor PWMs
    doc["pwm_L"] = currentLeftPWM;
    doc["pwm_R"] = currentRightPWM;

    // 6. Encoder Ticks
    doc["ticks_L"] = leftTicks;
    doc["ticks_R"] = rightTicks;

    // Serialize and publish (Increased buffer to 512 for the larger payload)
    char telemetryBuffer[512];
    serializeJson(doc, telemetryBuffer);
    mqtt.publish(TOPIC_TELEMETRY, telemetryBuffer);
}

//=====================================================
// SETUP
//=====================================================
void setup() {
    Serial.begin(115200);

    // Motors
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    // Sensors
    pinMode(S1, INPUT);
    pinMode(S2, INPUT);
    pinMode(S3, INPUT);
    pinMode(S4, INPUT);
    pinMode(S5, INPUT);

    // Encoders
    pinMode(ENCODER_L_PIN, INPUT_PULLUP);
    pinMode(ENCODER_R_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), rightEncoderISR, RISING);

    // PWM
    ledcAttach(L_PWM, 1000, 8);
    ledcAttach(R_PWM, 1000, 8);

    // RFID
    SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
    rfid.PCD_Init();

    setupWiFi();
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512);

    startDataLogging(); 
    Serial.println("AGV Ready & Logging Started");
}

//=====================================================
// LOOP
//=====================================================
void loop() {
    if (!mqtt.connected()) {
        stopMotor(); 
        long now = millis();
        if (now - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = now;
            if (reconnectMqtt()) {
                lastReconnectAttempt = 0;
            }
        }
    } else {
        mqtt.loop();
        
        // Broadcast Telemetry Data Live
        if (millis() - lastTelemetryTime > TELEMETRY_INTERVAL) {
            lastTelemetryTime = millis();
            publishTelemetry();
        }
    }

    if (mqtt.connected()) {
        readSensors();
        checkRFID();
        updateRobot();
    }
}

//=====================================================
// HIGH LEVEL FUNCTIONS
//=====================================================
void updateRobot() {
    switch(state) {
        case FOLLOW_LINE:
            followLine();
            break;
        case TURNING:
            updateTurning();
            break;
        case STOPPED: {  
            readSensors();
            int blackSensors = 0;
            for(int i = 0; i < 5; i++) {
                if (sensorValue[i] == LINE_DETECTED) blackSensors++;
            }
            if (blackSensors >= 4) {
                state = FOLLOW_LINE;
                currentCommand = CMD_NONE;
                stopImmunityTimer = millis();
                startDataLogging(); 
            } else {
                stopMotor();
            }
            break;
        } 
        case WAITING_FOR_SERVER:
            stopMotor();
            break;
    }
}

void followLine() {
    if(shouldStop()) {
        stopMotor();

        // A line end is a task arrival only when the backend's remaining plan
        // contains exactly the final goal.  Any other all-white reading is a
        // line/sensor fault and must never advance the warehouse task.
        if (terminalArrivalPending && !terminalArrivalReported) {
            terminalArrivalReported = true;
            state = WAITING_FOR_SERVER;
            currentCommand = CMD_NONE;
            startWaitTimer();
            publishNodeReport("node_id", activeGoal, "line_end_arrival");
            finishDataLogging();
            sendDataToServer();
        } else {
            state = STOPPED;
            currentCommand = CMD_STOP;
            Serial.println("All-white detected before final route node; task not acknowledged.");
        }
        return;
    }

    calculatePID();
    driveRobot();
    updateDataLogging(error, correction); 
}

void executeCommand() {
    switch(currentCommand) {
        case CMD_STOP:
            state = STOPPED;
            break;
        case CMD_LEFT:
        case CMD_RIGHT:
        case CMD_TURN_BACK:
            leftTicks = 0;
            rightTicks = 0;
            state = TURNING;
            break;
        case CMD_STRAIGHT:
            currentCommand = CMD_NONE;
            state = FOLLOW_LINE;
            
            // Add these two lines to give the robot time to find the line again
            stopImmunityTimer = millis(); 
            startDataLogging(); 
            break;
        default:
            break;
    }
}

//=====================================================
// RFID
//=====================================================
void checkRFID() {
    if(state != FOLLOW_LINE) return;

    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

    String currentUID = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
        currentUID += String(rfid.uid.uidByte[i], HEX);
    }
    currentUID.toUpperCase();
    rfid.PICC_HaltA(); 

    if(currentUID == lastUID) return;
    lastUID = currentUID;

    incrementRFIDCount(); 
    stopMotor();
    state = WAITING_FOR_SERVER;
    startWaitTimer(); 

    publishNodeReport("rfid_id", currentUID);
}

bool publishNodeReport(const char* identifierKey, const String& identifier, const char* eventType) {
    JsonDocument doc;
    doc[identifierKey] = identifier;
    if (eventType != nullptr) doc["event"] = eventType;

    char payload[192];
    serializeJson(doc, payload, sizeof(payload));
    bool published = mqtt.publish(TOPIC_PUBLISH_NODE, payload);
    Serial.print("Node report: ");
    Serial.println(payload);
    return published;
}

//=====================================================
// NAVIGATION (ENCODER-BASED TURNING)
//=====================================================
void updateTurning() {
    long targetTicks = (currentCommand == CMD_TURN_BACK) ? (TICKS_FOR_90_DEGREE * 2.3) : TICKS_FOR_90_DEGREE;

    if (leftTicks >= targetTicks || rightTicks >= targetTicks) {
        finishTurn();
        return;
    }

    switch(currentCommand) {
        case CMD_LEFT:      rotateLeft(TURN_SPEED); break;
        case CMD_TURN_BACK: rotateLeft(TURN_SPEED); break;
        case CMD_RIGHT:     rotateRight(TURN_SPEED); break;
        default:            stopMotor(); return;
    }
}

void finishTurn() {
    stopMotor(); 

    integral = 0;
    previousError = 0;
    error = 0;
    correction = 0;

    currentCommand = CMD_NONE;
    state = FOLLOW_LINE;
}

//=====================================================
// PID & SENSORS
//=====================================================
void readSensors() {
    sensorValue[0] = digitalRead(S1);
    sensorValue[1] = digitalRead(S2);
    sensorValue[2] = digitalRead(S3);
    sensorValue[3] = digitalRead(S4);
    sensorValue[4] = digitalRead(S5);
}

bool shouldStop() {
    if(state != FOLLOW_LINE) return false;
    if (millis() - stopImmunityTimer < 1500) return false;

    int blackCount = 0;
    for (int i = 0; i < 5; i++) {
        if (sensorValue[i] == LINE_DETECTED) blackCount++;
    }

    static unsigned long whiteTimer = 0;
    if (blackCount == 0) {
        if (whiteTimer == 0) whiteTimer = millis();
        else if (millis() - whiteTimer > 150) {
            whiteTimer = 0;
            return true;
        }
    } else {
        whiteTimer = 0;
    }
    return false;
}

float getLinePosition() {
    int weight[5] = {-4, -1, 0, 1, 4};
    int sum = 0, count = 0;

    for (int i = 0; i < 5; i++) {
        if (sensorValue[i] == LINE_DETECTED) {
            sum += weight[i];
            count++;
        }
    }
    if (count == 0) return 0; 
    return (float)sum / count;
}

void calculatePID() {
    error = getLinePosition();
    integral += error;
    integral = constrain(integral,-50,50);
    derivative = error - previousError;
    correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;
}

void driveRobot() {
    int leftMotor = BASE_SPEED + correction;
    int rightMotor = BASE_SPEED - correction;
    moveMotor(leftMotor, rightMotor);
}

//=====================================================
// MOTOR CONTROLS
//=====================================================
// In your main .ino file, update moveMotor():
void moveMotor(int leftPWM, int rightPWM) {
    if (leftPWM >= 0) {
        digitalWrite(L_IN1, HIGH);
        digitalWrite(L_IN2, LOW);
        // Restored to 0: allows the wheel to cut power for sharp turns
        if (leftPWM > 0 && leftPWM < 170) leftPWM = 0; 
    } else {
        digitalWrite(L_IN1, LOW);
        digitalWrite(L_IN2, HIGH);
        leftPWM = abs(leftPWM); 
        if (leftPWM > 0 && leftPWM < 170) leftPWM = 170;
    }

    if (rightPWM >= 0) {
        digitalWrite(R_IN1, HIGH);
        digitalWrite(R_IN2, LOW);
        // Restored to 0: allows the wheel to cut power for sharp turns
        if (rightPWM > 0 && rightPWM < 170) rightPWM = 0; 
    } else {
        digitalWrite(R_IN1, LOW);
        digitalWrite(R_IN2, HIGH);
        rightPWM = abs(rightPWM);
        if (rightPWM > 0 && rightPWM < 170) rightPWM = 170;
    }

    leftPWM = constrain(leftPWM, 0, MAX_PWM);
    rightPWM = constrain(rightPWM, 0, MAX_PWM);

    currentLeftPWM = leftPWM;
    currentRightPWM = rightPWM;

    ledcWrite(L_PWM, leftPWM);
    ledcWrite(R_PWM, rightPWM);
}

void stopMotor() {
    currentLeftPWM = 0;
    currentRightPWM = 0;

    ledcWrite(L_PWM,0);
    ledcWrite(R_PWM,0);
    digitalWrite(L_IN1,LOW);
    digitalWrite(L_IN2,LOW);
    digitalWrite(R_IN1,LOW);
    digitalWrite(R_IN2,LOW);
}

void rotateLeft(int speed) {
    currentLeftPWM = -speed; // Negative to signify reverse in your dashboard
    currentRightPWM = speed;

    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    ledcWrite(L_PWM, speed);
    ledcWrite(R_PWM, speed);
}

void rotateRight(int speed) {
    currentLeftPWM = speed; 
    currentRightPWM = -speed;

    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    ledcWrite(L_PWM, speed);
    ledcWrite(R_PWM, speed);
}
