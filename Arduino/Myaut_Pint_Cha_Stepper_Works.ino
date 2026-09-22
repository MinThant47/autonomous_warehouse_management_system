
// =====================================================
// STEPPER MOTOR
// =====================================================

#define STEP_IN1 14
#define STEP_IN2 15
#define STEP_IN3 16
#define STEP_IN4 17

//Includes the Arduino Stepper Library
#include <Stepper.h>

// Defines the number of steps per rotation
const int stepsPerRevolution = 2038;

// Creates an instance of stepper class
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence
Stepper myStepper = Stepper(stepsPerRevolution, STEP_IN1, STEP_IN3, STEP_IN2, STEP_IN4);

// =====================================================
// DC MOTORS
// =====================================================

#define L_IN1 8
#define L_IN2 9
#define L_PWM 10

#define R_IN1 11
#define R_IN2 12
#define R_PWM 13

int motorSpeed = 200;

// =====================================================
// ENCODERS
// =====================================================

#define ENC_L_A 2
#define ENC_L_B 3

#define ENC_R_A 4
#define ENC_R_B 5

volatile long leftEncoder = 0;
volatile long rightEncoder = 0;

// =====================================================
// PWM
// =====================================================

#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

// =====================================================
// ENCODER ISR
// =====================================================

void IRAM_ATTR leftEncoderISR()
{
    if (digitalRead(ENC_L_B))
        leftEncoder++;
    else
        leftEncoder--;
}

void IRAM_ATTR rightEncoderISR()
{
    if (digitalRead(ENC_R_B))
        rightEncoder++;
    else
        rightEncoder--;
}

// =====================================================
// MOTOR FUNCTIONS
// =====================================================

void stopMotors()
{
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);

    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);

    ledcWrite(L_PWM, 0);
    ledcWrite(R_PWM, 0);

    Serial.println("STOP");
}

void moveForward()
{
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);

    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);

    ledcWrite(L_PWM, motorSpeed);
    ledcWrite(R_PWM, motorSpeed);

    Serial.println("FORWARD");
}

void moveBackward()
{
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);

    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);

    ledcWrite(L_PWM, motorSpeed);
    ledcWrite(R_PWM, motorSpeed);

    Serial.println("BACKWARD");
}

// =====================================================
// TIMED MOTOR MOVES
// =====================================================

void moveForwardTimed(int ms)
{
    moveForward();
    delay(ms);
    stopMotors();
}

void moveBackwardTimed(int ms)
{
    moveBackward();
    delay(ms);
    stopMotors();
}

// =====================================================
// STEPPER FUNCTIONS
// =====================================================

void stepperUp(){
    myStepper.setSpeed(12);
	myStepper.step(stepsPerRevolution*2);
}

void stepperDown(){
	myStepper.setSpeed(12);
	myStepper.step(-stepsPerRevolution*2);
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    // DC Motor Pins
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);

    pinMode(R_IN1, OUTPUT);
    pinMode(R_IN2, OUTPUT);

    // Encoder Pins
    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);

    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);

    // PWM
    ledcAttach(L_PWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(R_PWM, PWM_FREQ, PWM_RESOLUTION);

    // Encoder Interrupts
    attachInterrupt(
        digitalPinToInterrupt(ENC_L_A),
        leftEncoderISR,
        RISING);

    attachInterrupt(
        digitalPinToInterrupt(ENC_R_A),
        rightEncoderISR,
        RISING);

    stopMotors();

    Serial.println("SYSTEM READY");
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    // =========================================
    // FORWARD -> UP -> BACKWARD
    // =========================================

    moveForwardTimed(3000);

    delay(1000);

    stepperUp();

    delay(1000);

    moveBackwardTimed(3000);

    delay(1000);

    // =========================================
    // FORWARD -> DOWN -> BACKWARD
    // =========================================

    moveForwardTimed(3000);

    delay(1000);

    stepperDown();

    delay(1000);

    moveBackwardTimed(3000);

    delay(1000);

    // Encoder values
    Serial.print("Left Encoder = ");
    Serial.print(leftEncoder);

    Serial.print("    Right Encoder = ");
    Serial.println(rightEncoder);
}