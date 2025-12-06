#include <Arduino.h>
#include <Wire.h>

// ---------- MOTOR DRIVER PINS ----------
#define MTA1 A6
#define MTA2 A7
#define MTB1 D8
#define MTB2 D9

// ---------- ENCODER PINS ----------
#define MEA1 D10
#define MEA2 D11
#define MEB1 D12
#define MEB2 D13

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
const int motorSpeed = 600;  // Range: 0-1023

// ---------- ENCODER SETTINGS ----------
const int ticksPerRev = 12 * 4;  // 12 ticks per channel, 4x decoding
const float gearRatio = 30.0;
const float wheelDiameter = 2;  // 65 mm wheel diameter in meters
const float wheelCircumference = PI * wheelDiameter;
const float distancePerTick = wheelCircumference / (ticksPerRev * gearRatio);

// ---------- DISTANCE TARGET ----------
const float DISTANCE_TARGET = 1.0;  // 1 meter
const float DISTANCE_TOLERANCE = 0.05;  // ±5 cm tolerance

// ---------- ENCODER & ISR ----------
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

void IRAM_ATTR handleEncoderA() {
  uint8_t state = READ_ENC_A();
  uint8_t combo = (prevStateA << 2) | state;
  switch (combo) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCountA++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCountA--; break;
  }
  prevStateA = state;
}

void IRAM_ATTR handleEncoderB() {
  uint8_t state = READ_ENC_B();
  uint8_t combo = (prevStateB << 2) | state;
  switch (combo) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCountB++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCountB--; break;
  }
  prevStateB = state;
}

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

// Encoder tracking
long encoderStartA = 0;
long encoderStartB = 0;

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

// ---------- Closed-Loop Motor Control ----------
void updateMotorsWithFeedback(int targetSpeed, long targetDistance) {
  // Calculate current distance traveled
  long deltaA = encoderCountA - encoderStartA;
  long deltaB = encoderCountB - encoderStartB;

  // Average distance from both encoders
  float distanceA = deltaA * distancePerTick;
  float distanceB = deltaB * distancePerTick;
  float avgDistance = (distanceA + distanceB) / 2.0;

  // Speed correction: if one wheel is faster, reduce its speed
  int speedA = targetSpeed;
  int speedB = targetSpeed;

  // Simple correction: adjust slower motor or reduce faster motor
  if (deltaA > deltaB) {
    // Motor A is faster, slow it down slightly
    speedA = constrain(targetSpeed - 20, 0, 1023);
  } else if (deltaB > deltaA) {
    // Motor B is faster, slow it down slightly
    speedB = constrain(targetSpeed - 20, 0, 1023);
  }

  // Apply motor speeds
  setMotorA(speedA);
  setMotorB(speedB);

  // Debug output every 500ms
  static unsigned long lastDebug = 0;
  unsigned long now = millis();
  if (now - lastDebug >= 500) {
    Serial.printf("Distance: %.3f m | A: %ld ticks | B: %ld ticks\n", avgDistance, deltaA, deltaB);
    lastDebug = now;
  }
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

// ---------- Setup Encoder Interrupts ----------
void setupEncoderInterrupt() {
  pinMode(MEA1, INPUT);
  pinMode(MEA2, INPUT);
  pinMode(MEB1, INPUT);
  pinMode(MEB2, INPUT);
  
  prevStateA = READ_ENC_A();
  prevStateB = READ_ENC_B();
  
  attachInterrupt(digitalPinToInterrupt(MEA1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEA2), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB1), handleEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB2), handleEncoderB, CHANGE);
}

// ---------- Get Average Distance Traveled ----------
float getAverageDistance() {
  long deltaA = encoderCountA - encoderStartA;
  long deltaB = encoderCountB - encoderStartB;
  
  float distanceA = deltaA * distancePerTick;
  float distanceB = deltaB * distancePerTick;
  
  return (distanceA + distanceB) / 2.0;
}

// ---------- State Machine Update ----------
void updateStateMachine() {
  float currentDistance = getAverageDistance();

  switch (currentState) {
    case IDLE:
      // Reset encoders and start forward movement
      encoderStartA = encoderCountA;
      encoderStartB = encoderCountB;
      currentState = MOVING_FORWARD;
      stateStartTime = millis();
      Serial.println("\n=== Starting: Drive Forward 1m ===");
      break;

    case MOVING_FORWARD:
      // Check if distance target reached
      if (currentDistance >= DISTANCE_TARGET - DISTANCE_TOLERANCE) {
        stopMotors();
        currentState = MOVING_BACKWARD;
        stateStartTime = millis();
        Serial.printf("Forward complete: %.3f m traveled\n", currentDistance);
        Serial.println("Brief pause...");
        delay(500);
        
        // Reset encoders for backward journey
        encoderStartA = encoderCountA;
        encoderStartB = encoderCountB;
        Serial.println("=== Starting: Drive Backward 1m ===");
      } else {
        // Apply closed-loop control with speed correction
        updateMotorsWithFeedback(motorSpeed, (long)(DISTANCE_TARGET / distancePerTick));
      }
      break;

    case MOVING_BACKWARD:
      // Check if distance target reached (moving backward, so negative direction)
      if (currentDistance >= DISTANCE_TARGET - DISTANCE_TOLERANCE) {
        stopMotors();
        currentState = COMPLETE;
        stateStartTime = millis();
        Serial.printf("Backward complete: %.3f m traveled\n", currentDistance);
        Serial.println("Complete: Robot at starting position");
      } else {
        // Move backward with speed correction
        setMotorA(-motorSpeed);
        setMotorB(-motorSpeed);
        
        // Simple speed correction for backward motion
        long deltaA = abs(encoderCountA - encoderStartA);
        long deltaB = abs(encoderCountB - encoderStartB);
        
        if (deltaA > deltaB) {
          setMotorA(-(motorSpeed - 20));
        } else if (deltaB > deltaA) {
          setMotorB(-(motorSpeed - 20));
        }
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

  // Encoder setup
  setupEncoderInterrupt();

  // Initialize motors to stopped
  stopMotors();

  Serial.println("\n==========================================");
  Serial.println("=== Closed-Loop Motion Control (Q3) ===");
  Serial.println("Motion: 1m Forward + 1m Backward");
  Serial.println("Feedback: Wheel encoders + speed correction");
  Serial.println("LED: ON (idle) -> Blinking (moving)");
  Serial.println("==========================================\n");

  delay(1000);
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
