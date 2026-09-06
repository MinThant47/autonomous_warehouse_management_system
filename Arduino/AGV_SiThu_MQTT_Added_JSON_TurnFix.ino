//=====================================================
// LIBRARIES
//=====================================================
#include <WiFi.h>

#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoJson.h> // New library for JSON parsing

//=====================================================
// USER SETTINGS
//=====================================================
// Network Credentials
const char* WIFI_SSID = "your-wifi";
const char* WIFI_PASSWORD = "your-wifi-password";
const char* MQTT_HOST = "192.168.1.50";  // Server IP
const int MQTT_PORT = 1883;

// MQTT Topics
const char* TOPIC_PUBLISH_NODE = "agv/R1/node";
const char* TOPIC_SUBSCRIBE_CMD = "agv/R1/command";

// PID - Tuned for smoother line following (less shaking)
float Kp = 25.0; 
float Ki = 0.0;
float Kd = 8.0;

bool DEBUG = true;

// Speed - Lowered slightly to prevent overshooting turns and reduce wobbles
#define BASE_SPEED 190
#define TURN_SPEED 195

#define MAX_PWM 255
#define MIN_PWM 0

// Sensor Logic (LINE_DETECTED)
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
#define S4 21
#define S5 7

// RFID
#define RFID_SS   6
#define RFID_RST  40
#define RFID_SCK  36
#define RFID_MISO 37
#define RFID_MOSI 35

//=====================================================
// ENUMS
//=====================================================
enum RobotState {
    FOLLOW_LINE,
    TURNING,
    STOPPED,
    WAITING_FOR_SERVER // New state to pause operations until command arrives
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

enum TurnPhase {
    LEAVE_LINE,
    FIND_LINE
};

//=====================================================
// GLOBAL VARIABLES
//=====================================================
// Network Objects
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// PID variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float correction = 0;

unsigned long turnStartTime = 0;
unsigned long lastReconnectAttempt = 0;

// Sensor array
int sensorValue[5];

// RFID object
MFRC522 rfid(RFID_SS, RFID_RST);
String lastUID = "";

// Navigation variables
TurnPhase turnPhase = LEAVE_LINE;
int lineCount = 0;

//=====================================================
// FUNCTION PROTOTYPES
//=====================================================
void stopMotor();
void executeCommand();
void printState();

//=====================================================
// NETWORK FUNCTIONS
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
    Serial.print("Received Message via MQTT on topic: ");
    Serial.println(topic);

    // Create a JSON document to hold the incoming data
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    // Check if the JSON is valid
    if (error) {
        Serial.print("JSON Parsing failed: ");
        Serial.println(error.c_str());
        return; // Exit if the message isn't valid JSON
    }

    // Extract the "action" value from the JSON
    String action = doc["action"].as<String>();
    action.toUpperCase(); // Ensure uppercase for safety

    Serial.print("Parsed Action: ");
    Serial.println(action);

    // Only process commands if we are actually waiting for the server
    if (state == WAITING_FOR_SERVER) {
        if (action == "LEFT") {
            currentCommand = CMD_LEFT;
        } 
        else if (action == "RIGHT") {
            currentCommand = CMD_RIGHT;
        } 
        else if (action == "STRAIGHT" || action == "FORWARD") {
            // Mapped "FORWARD" to CMD_STRAIGHT so the robot continues on the line
            currentCommand = CMD_STRAIGHT;
        } 
        else if (action == "TURN_BACK") {
            currentCommand = CMD_TURN_BACK;
        } 
        else if (action == "STOP") {
            currentCommand = CMD_STOP;
        } 
        else {
            Serial.println("Invalid action received inside JSON.");
            return; 
        }

        executeCommand();
    }
}

boolean reconnectMqtt() {
    if (mqtt.connect("agv-r1-esp32")) {
        Serial.println("Connected to MQTT broker");
        mqtt.subscribe(TOPIC_SUBSCRIBE_CMD);
    }
    return mqtt.connected();
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

    // PWM
    ledcAttach(L_PWM, 1000, 8);
    ledcAttach(R_PWM, 1000, 8);

    // RFID
    SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
    rfid.PCD_Init();

    // Network setup
    setupWiFi();
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);

    Serial.println("AGV Ready");
}

//=====================================================
// LOOP
//=====================================================
void loop() {
    // Non-blocking MQTT reconnection to prevent code freezing
    if (!mqtt.connected()) {
        stopMotor(); // Safety measure: halt AGV if network drops
        long now = millis();
        if (now - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = now;
            if (reconnectMqtt()) {
                lastReconnectAttempt = 0;
            }
        }
    } else {
        mqtt.loop();
    }

    // Only process navigation if we are connected to the network
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
        case STOPPED:
        case WAITING_FOR_SERVER:
            stopMotor();
            break;
    }
}

void followLine() {
    if(shouldStop()) {
        Serial.println("STOP CONDITION");
        stopMotor();
        return;
    }

    calculatePID();
    driveRobot();

    if(DEBUG) {
        // printSensors();
    }
}

void executeCommand() {
    switch(currentCommand) {
        case CMD_STOP:
            state = STOPPED;
            break;

        case CMD_LEFT:
        case CMD_RIGHT:
        case CMD_TURN_BACK:
            turnPhase = LEAVE_LINE;
            lineCount = 0;
            turnStartTime = millis();
            state = TURNING;
            break;

        case CMD_STRAIGHT:
            currentCommand = CMD_NONE;
            state = FOLLOW_LINE;
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

    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
        return;
    }

    String currentUID = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
        // Keep leading zeroes so the published UID matches rfid_node_map.json.
        // For example, byte 0x0E must be published as "0E", not "E".
        if (rfid.uid.uidByte[i] < 0x10) currentUID += "0";
        currentUID += String(rfid.uid.uidByte[i], HEX);
    }
    currentUID.toUpperCase();
    rfid.PICC_HaltA(); // Halt reading quickly to resume processing

    // Prevent re-triggering on the exact same card in the same pass
    if(currentUID == lastUID) return;
    lastUID = currentUID;

    Serial.print("RFID UID detected: ");
    Serial.println(currentUID);

    // 1. Stop the robot immediately
    stopMotor();
    
    // 2. Change state so it waits for MQTT command
    state = WAITING_FOR_SERVER;

    // 3. Publish the UID to the server
    mqtt.publish(TOPIC_PUBLISH_NODE, currentUID.c_str());
    Serial.println("Sent UID to server. Waiting for command...");
    
    if(DEBUG) { printState(); }
}

//=====================================================
// NAVIGATION (Turning Logic)
//=====================================================
bool centerOnLine() {
    return sensorValue[2] == LINE_DETECTED;
}

//=====================================================
// NAVIGATION
//=====================================================

void updateTurning()
{
    readSensors();  
    
    // Rotate according to command using your original single TURN_SPEED
    switch(currentCommand)
    {
        case CMD_LEFT:
        case CMD_TURN_BACK:
            rotateLeft(TURN_SPEED);
            break;

        case CMD_RIGHT:
            rotateRight(TURN_SPEED);
            break;

        default:
            stopMotor();
            return;
    }

    switch(turnPhase)
    {
        case LEAVE_LINE:
            // STRICT CLEARANCE: Ensure S2, S3, and S4 have ALL completely cleared the starting line
            // This prevents premature state changes at wide intersections
            if ((millis() - turnStartTime > 150) && 
                (sensorValue[1] != LINE_DETECTED) && 
                (sensorValue[2] != LINE_DETECTED) && 
                (sensorValue[3] != LINE_DETECTED))
            {
                turnPhase = FIND_LINE;
                Serial.println("Cleared old line -> Entering FIND_LINE");
            }
            break;

        case FIND_LINE:
            // DIRECTIONAL SENSOR ALIGNMENT: Catch the line slightly earlier
            if (isTargetLineDetected())
            {
                if (currentCommand == CMD_TURN_BACK)
                {
                    lineCount++;
                    if (lineCount < 2)
                    {
                        turnPhase = LEAVE_LINE;
                        turnStartTime = millis();
                    }
                    else
                    {
                        finishTurn();
                    }
                }
                else
                {
                    finishTurn();
                }
            }              
            break;
    }
}

// Helper function to detect the incoming line based on turn direction
bool isTargetLineDetected()
{
    // For Left turns, S2 (left inner) or S3 (center) will touch the line first
    if (currentCommand == CMD_LEFT || currentCommand == CMD_TURN_BACK)
    {
        return (sensorValue[1] == LINE_DETECTED || sensorValue[2] == LINE_DETECTED);
    }
    // For Right turns, S4 (right inner) or S3 (center) will touch the line first
    else if (currentCommand == CMD_RIGHT)
    {
        return (sensorValue[3] == LINE_DETECTED || sensorValue[2] == LINE_DETECTED);
    }
    
    // Fallback
    return centerOnLine();
}

void finishTurn() {
    integral = 0;
    previousError = 0;
    error = 0;
    correction = 0;

    turnPhase = LEAVE_LINE;
    lineCount = 0;

    currentCommand = CMD_NONE;
    state = FOLLOW_LINE;
    Serial.println("Turn Complete");
    printState();
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

    int blackCount = 0;
    for (int i = 0; i < 5; i++) {
        if (sensorValue[i] == LINE_DETECTED) blackCount++;
    }

    static unsigned long whiteTimer = 0;
    static bool onWhite = false;

    if (blackCount == 0) {
        if (!onWhite) {
            onWhite = true;
            whiteTimer = millis();
        } 
        else if (millis() - whiteTimer > 250) {
            return true; 
        }
    } else {
        onWhite = false;
    }
    return false;
}

float getLinePosition() {
    int weight[5] = {-2, -1, 0, 1, 2};
    int sum = 0;
    int count = 0;

    for (int i = 0; i < 5; i++) {
        if (sensorValue[i] == LINE_DETECTED) {
            sum += weight[i];
            count++;
        }
    }
    if (count == 0) return previousError;
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
void moveMotor(int leftPWM, int rightPWM) {
    leftPWM = constrain(leftPWM, MIN_PWM, MAX_PWM);
    rightPWM = constrain(rightPWM, MIN_PWM, MAX_PWM);

    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);

    ledcWrite(L_PWM, leftPWM);
    ledcWrite(R_PWM, rightPWM);
}

void stopMotor() {
    ledcWrite(L_PWM,0);
    ledcWrite(R_PWM,0);
    digitalWrite(L_IN1,LOW);
    digitalWrite(L_IN2,LOW);
    digitalWrite(R_IN1,LOW);
    digitalWrite(R_IN2,LOW);
}

void rotateLeft(int speed) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    ledcWrite(L_PWM, speed);
    ledcWrite(R_PWM, speed);
}

void rotateRight(int speed) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    ledcWrite(L_PWM, speed);
    ledcWrite(R_PWM, speed);
}

//=====================================================
// DEBUG
//=====================================================
void printState() {
    Serial.print("State = ");
    switch(state) {
        case FOLLOW_LINE: Serial.println("FOLLOW_LINE"); break;
        case TURNING: Serial.println("TURNING"); break;
        case STOPPED: Serial.println("STOPPED"); break;
        case WAITING_FOR_SERVER: Serial.println("WAITING_FOR_SERVER"); break;
    }
}
