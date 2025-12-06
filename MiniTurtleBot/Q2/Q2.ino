#include <Arduino.h>

// ---------- MOTOR DRIVER PINS ----------
#define MTA1 A6
#define MTA2 A7
#define MTB1 D8
#define MTB2 D9

// ---------- LED PINS ----------
#define LED1 D6
#define LED2 D7

// ---------- PWM CONFIG ----------
const int pwmFreq = 500;
const int pwmResolution = 10;
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- MOTOR SPEED ----------
const int motorSpeed = 600;  // Range: 0-1023 (open-loop)
const int motorReverseSpeed = -600;

// ---------- DISTANCE & TIMING ----------
// Assumptions:
// - Robot speed at motorSpeed=600 is approximately 0.4 m/s (400 mm/s)
// - 1 meter = 2500 ms at this speed (1000 mm / 400 mm/s = 2.5 s)
const unsigned long DISTANCE_TIME = 6000;  // Time to travel 1 meter (ms)

// ---------- STATE MACHINE ----------
enum State {
  IDLE,
  MOVING_FORWARD,
  MOVING_BACKWARD,
  COMPLETE
};

State currentState = IDLE;
unsigned long stateStartTime = 0;
unsigned long blink_timer = 0;
bool led_state = false;

// ---------- Motor PWM Setup ----------
void setMotorA(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    ledcWrite(pwmA1_Ch, pwm);
    ledcWrite(pwmA2_Ch, 0);
  } else {
    ledcWrite(pwmA1_Ch, 0);
    ledcWrite(pwmA2_Ch, -pwm);
  }
}

void setMotorB(int pwm) {
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    ledcWrite(pwmB1_Ch, 0);
    ledcWrite(pwmB2_Ch, pwm);
  } else {
    ledcWrite(pwmB1_Ch, -pwm);
    ledcWrite(pwmB2_Ch, 0);
  }
}

void stopMotors() {
  setMotorA(0);
  setMotorB(0);
}

void driveForward(int speed) {
  setMotorA(speed);
  setMotorB(speed);
}

void driveBackward(int speed) {
  setMotorA(speed);
  setMotorB(speed);
}

// ---------- LED Control ----------
void ledsOn() {
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
}

void ledsOff() {
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
}

void updateLEDs(bool isMoving) {
  if (!isMoving) {
    // Robot not moving: LEDs solid ON
    ledsOn();
  } else {
    // Robot moving: LEDs blinking (200ms period)
    unsigned long now = millis();
    if (now - blink_timer >= 200) {
      led_state = !led_state;
      if (led_state) {
        ledsOn();
      } else {
        ledsOff();
      }
      blink_timer = now;
    }
  }
}

// ---------- Setup Motor PWM ----------
void setupMotorPWM() {
  ledcSetup(pwmA1_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmA2_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmB1_Ch, pwmFreq, pwmResolution);
  ledcSetup(pwmB2_Ch, pwmFreq, pwmResolution);
  ledcAttachPin(MTA1, pwmA1_Ch);
  ledcAttachPin(MTA2, pwmA2_Ch);
  ledcAttachPin(MTB1, pwmB1_Ch);
  ledcAttachPin(MTB2, pwmB2_Ch);
}

// ---------- State Machine Update ----------
void updateStateMachine() {
  unsigned long elapsed = millis() - stateStartTime;

  switch (currentState) {
    case IDLE:
      // Initial state - transition to forward movement
      driveForward(motorSpeed);
      currentState = MOVING_FORWARD;
      stateStartTime = millis();
      Serial.println("Starting: Drive Forward 1m");
      break;

    case MOVING_FORWARD:
      // Moving forward - check if time elapsed
      if (elapsed >= DISTANCE_TIME) {
        stopMotors();
        currentState = MOVING_BACKWARD;
        stateStartTime = millis();
        Serial.println("Switching: Drive Backward 1m");
        delay(500);  // Brief pause between movements
      }
      break;

    case MOVING_BACKWARD:
      // Moving backward - check if time elapsed
      if (elapsed >= DISTANCE_TIME) {
        stopMotors();
        currentState = COMPLETE;
        stateStartTime = millis();
        Serial.println("Complete: Robot at starting position");
      } else {
        driveBackward(motorReverseSpeed);
      }
      break;

    case COMPLETE:
      // Motion complete - keep motors stopped
      stopMotors();
      break;
  }
}

// ---------- Main Setup ----------
void setup() {
  Serial.begin(115200);

  // LED setup
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  ledsOn();  // Solid ON while idle

  // Motor PWM setup
  setupMotorPWM();

  // Initialize motors to stopped
  stopMotors();

  Serial.println("\n=== Open-Loop Motion Control ===");
  Serial.println("Motion: 1m Forward + 1m Backward");
  Serial.println("LED: ON (idle) -> Blinking (moving)");
}

// ---------- Main Loop ----------
void loop() {
  // Update state machine
  updateStateMachine();

  // Update LED based on motion state
  bool isMoving = (currentState == MOVING_FORWARD || currentState == MOVING_BACKWARD);
  updateLEDs(isMoving);

  // Small delay to prevent excessive CPU usage
  delayMicroseconds(100);
}
