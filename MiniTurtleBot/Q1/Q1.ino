#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include "MPU6050.h"

// ---------- I2C PINS ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- OLED CONFIG ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- VL53L0X CONFIG ----------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
int distanceCM = 0;

// ---------- IMU CONFIG ----------
MPU6050 imu;

// ---------- MOTOR DRIVER ----------
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

// ---------- SERVO PWM PINS ----------
#define SERVO3 D3

// ---------- IR PINS ----------
#define IRD1 D2

// ---------- PWM CONFIG ----------
const int pwmFreq = 500;
const int pwmResolution = 10;
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;
const int pwmServoFreq = 50;
const int pwmServoResolution = 14;
const int pwmServo3_Ch = 4;

// ---------- Encoder & Motion ----------
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

// ---------- Encoder Settings ----------
const int ticksPerRev = 12 * 4;
const float gearRatio = 30.0;

#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

// ---------- Motor PWM ----------
int motorPowerA = 500;
int motorPowerB = 500;

// ---------- STATE MACHINE ----------
enum State {
  EXAMPLE_1_LED,
  EXAMPLE_2_OLED,
  EXAMPLE_3_IMU,
  EXAMPLE_4_MOTOR_ENCODER,
  EXAMPLE_5_SERVO,
  EXAMPLE_6_LIDAR,
  EXAMPLE_7_IR,
  STATE_COUNT
};

State currentState = EXAMPLE_1_LED;
unsigned long stateStartTime = 0;
const unsigned long STATE_DURATION = 10000; // 10 seconds

// ---------- ISR ----------
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

// ---------- Motor Control ----------
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

// ---------- Servo Control ----------
uint32_t angleToDuty(int angle) {
  int pulse = map(angle, 0, 180, 1000, 2000);
  int duty = (pulse * ((1 << pwmServoResolution) - 1)) / 20000UL;
  return duty;
}

// ---------- Setup Functions ----------
void setupEncoderInterrupt() {
  pinMode(MEA1, INPUT); pinMode(MEA2, INPUT);
  pinMode(MEB1, INPUT); pinMode(MEB2, INPUT);
  prevStateA = READ_ENC_A();
  prevStateB = READ_ENC_B();
  attachInterrupt(digitalPinToInterrupt(MEA1), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEA2), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB1), handleEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MEB2), handleEncoderB, CHANGE);
}

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

void setupServo() {
  ledcSetup(pwmServo3_Ch, pwmServoFreq, pwmServoResolution);
  ledcAttachPin(SERVO3, pwmServo3_Ch);
}

void initializeAllHardware() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setRotation(2);

  // LED setup
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Motor and Encoder setup
  setupEncoderInterrupt();
  setupMotorPWM();

  // Servo setup
  setupServo();

  // VL53L0X setup
  if (!lox.begin()) {
    Serial.println("VL53L0X not found");
  }

  // MPU6050 setup
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("MPU6050 not connected!");
  }
}

// ---------- Example 1: LED Blink ----------
void example1_loop() {
  unsigned long elapsed = millis() - stateStartTime;
  
  // Blink pattern: 500ms on, 500ms off
  unsigned long cycle = elapsed % 1000;
  
  if (cycle < 500) {
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }

  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Example 1: LED");
  display.println("Blinking...");
  display.print("Time: ");
  display.print(elapsed / 1000);
  display.println("s");
  display.display();
}

// ---------- Example 2: OLED Static Display ----------
void example2_loop() {
  unsigned long elapsed = millis() - stateStartTime;

  // Only update once at the start
  if (elapsed < 100) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Example 2: OLED");
    display.println("Hello from");
    display.println("ESP32-S3!");
    display.println("");
    display.drawRect(0, 32, 128, 32, SSD1306_WHITE);
    display.setCursor(20, 40);
    display.println("Mini TurtleBot");
    display.display();
  }
}

// ---------- Example 3: IMU Accelerometer ----------
void example3_loop() {
  int16_t ax, ay, az;
  imu.getAcceleration(&ax, &ay, &az);

  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Example 3: IMU");
  display.println("Accel (g):");
  display.print("X: "); display.println(ax_g, 2);
  display.print("Y: "); display.println(ay_g, 2);
  display.print("Z: "); display.println(az_g, 2);
  display.display();
}

// ---------- Example 4: Motor & Encoder ----------
void example4_loop() {
  static unsigned long lastTime = 0;
  static long lastCountA = 0, lastCountB = 0;
  unsigned long elapsed = millis() - stateStartTime;

  // Update every 100ms
  if (elapsed - lastTime >= 100 || lastTime == 0) {
    long deltaA = encoderCountA - lastCountA;
    long deltaB = encoderCountB - lastCountB;
    lastCountA = encoderCountA;
    lastCountB = encoderCountB;

    float ticksPerSecA = deltaA * 10.0;
    float ticksPerSecB = deltaB * 10.0;

    float rpmA = (ticksPerSecA * 60.0) / (ticksPerRev * gearRatio);
    float rpmB = (ticksPerSecB * 60.0) / (ticksPerRev * gearRatio);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Example 4:");
    display.println("Motor & Encoder");
    display.printf("A RPM: %.1f\n", fabs(rpmA));
    display.printf("B RPM: %.1f\n", fabs(rpmB));
    display.display();

    lastTime = elapsed;
  }

  // Run motors at moderate speed
  setMotorA(400);
  setMotorB(400);
}

// ---------- Example 5: Servo ----------
void example5_loop() {
  unsigned long elapsed = millis() - stateStartTime;

  // Sweep servo from 0 to 180 degrees
  int angle = (elapsed * 180) / 10000; // 0-180 over 10 seconds
  angle = constrain(angle, 0, 180);

  uint32_t duty = angleToDuty(angle);
  ledcWrite(pwmServo3_Ch, duty);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Example 5:");
  display.println("Servo Sweep");
  display.print("Angle: ");
  display.print(angle);
  display.println(" deg");
  display.display();
}

// ---------- Example 6: LIDAR ----------
void example6_loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    distanceCM = measure.RangeMilliMeter / 10;
  } else {
    distanceCM = 0;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Example 6:");
  display.println("LIDAR Distance");
  display.print("Distance: ");
  display.print(distanceCM);
  display.println(" cm");
  display.display();
}

// ---------- Example 7: IR Sensor ----------
void example7_loop() {
  int irValue = analogRead(IRD1);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Example 7:");
  display.println("IR Sensor");
  display.print("Value: ");
  display.println(irValue);
  display.display();
}

// ---------- State Transition Logic ----------
void updateState() {
  unsigned long elapsed = millis() - stateStartTime;

  if (elapsed >= STATE_DURATION) {
    // Switch to next state
    currentState = (State)((currentState + 1) % STATE_COUNT);
    stateStartTime = millis();

    // Stop motors when switching away from motor example
    if (currentState != EXAMPLE_4_MOTOR_ENCODER) {
      setMotorA(0);
      setMotorB(0);
    }

    // Display state transition
    Serial.print("Switching to Example ");
    Serial.println((int)currentState + 1);
  }
}

// ---------- Main Setup ----------
void setup() {
  initializeAllHardware();
  stateStartTime = millis();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Example Sequencer");
  display.println("Starting...");
  display.display();

  delay(1000); // Okay to use once at startup
}

// ---------- Main Loop ----------
void loop() {
  // Execute current example
  switch (currentState) {
    case EXAMPLE_1_LED:
      example1_loop();
      break;
    case EXAMPLE_2_OLED:
      example2_loop();
      break;
    case EXAMPLE_3_IMU:
      example3_loop();
      break;
    case EXAMPLE_4_MOTOR_ENCODER:
      example4_loop();
      break;
    case EXAMPLE_5_SERVO:
      example5_loop();
      break;
    case EXAMPLE_6_LIDAR:
      example6_loop();
      break;
    case EXAMPLE_7_IR:
      example7_loop();
      break;
  }

  // Check if time to switch to next state
  updateState();

  // Small non-blocking delay to prevent excessive CPU usage
  delayMicroseconds(100);
}
