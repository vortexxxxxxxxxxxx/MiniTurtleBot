#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"

// ---------- I2C ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- IMU ----------
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

// ---------- PWM CONFIG ----------
const int pwmFreq = 500;
const int pwmResolution = 10;
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- Encoder & Motion ----------
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

// ---------- Encoder Settings ----------
const int ticksPerRev = 12 * 4;  // 12 ticks per channel, 4x decoding
const float gearRatio = 30.0;     // Update if gear reduction is used

#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

// ---------- Robot/Drive Params ----------
const float wheelDiameter_mm = 65.0;          // adjust to your wheel
const float wheelCircumference_mm = PI * wheelDiameter_mm;
const float mmPerTick = wheelCircumference_mm / (ticksPerRev * gearRatio);
const float mmPerCm = 10.0;

// ---------- Control Gains ----------
int basePWM = -350;              // forward base power (negative per base_setup)
int minForwardPWM = -150;        // ensure wheels keep moving forward
int maxPWM = 800;                // safety cap
float kP_heading = 10.0;         // heading correction gain (rate-based)
float kP_gyro = 0.0;             // optionally use gyro z correction if set > 0
int motorTrim = 0;               // static offset to balance motors (A gets +trim, B gets -trim)

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

// ---------- Motor PWM ----------
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

void setupEncoderInterrupt(){
	pinMode(MEA1, INPUT); pinMode(MEA2, INPUT);
	pinMode(MEB1, INPUT); pinMode(MEB2, INPUT);
	prevStateA = READ_ENC_A();
	prevStateB = READ_ENC_B();
	attachInterrupt(digitalPinToInterrupt(MEA1), handleEncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(MEA2), handleEncoderA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(MEB1), handleEncoderB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(MEB2), handleEncoderB, CHANGE);
}

void setupMotorPWM(){
	ledcSetup(pwmA1_Ch, pwmFreq, pwmResolution);
	ledcSetup(pwmA2_Ch, pwmFreq, pwmResolution);
	ledcSetup(pwmB1_Ch, pwmFreq, pwmResolution);
	ledcSetup(pwmB2_Ch, pwmFreq, pwmResolution);
	ledcAttachPin(MTA1, pwmA1_Ch);
	ledcAttachPin(MTA2, pwmA2_Ch);
	ledcAttachPin(MTB1, pwmB1_Ch);
	ledcAttachPin(MTB2, pwmB2_Ch);
}

// ---------- Helpers ----------
void stopMotors(){ setMotorA(0); setMotorB(0); }

float getGyroZ(){
	int16_t gx, gy, gz;
	imu.getRotation(&gx, &gy, &gz);
	// Convert to deg/s if needed: sensitivity ~ 131 LSB/(deg/s) for ±250dps
	return gz / 131.0;
}

// Move straight for distance in cm, correcting heading drift only here
void straight(float distance_cm){
	long startA = encoderCountA;
	long startB = encoderCountB;
	float target_mm = distance_cm * mmPerCm;
	unsigned long lastUpdate = millis();
	long lastA = startA;
	long lastB = startB;

	while (true) {
		long ticksA = encoderCountA - startA;
		long ticksB = encoderCountB - startB;
		// Use absolute distance to avoid sign issues from encoder polarity
		float avg_mm = ((labs(ticksA) + labs(ticksB)) / 2.0) * mmPerTick;
		if (avg_mm >= target_mm) break;

		// Heading correction using encoder difference
		// Use instantaneous tick rates to correct heading (less bias from wheel mismatch)
		long dA = ticksA - lastA;
		long dB = ticksB - lastB;
		lastA = ticksA;
		lastB = ticksB;
		long diffRate = dA - dB;            // positive if A turning faster (robot curving left)
		float headingErr = (float)diffRate;

		// Optional gyro correction blended in
		float gyroErr = 0.0f;
		if (kP_gyro > 0.0f) gyroErr = getGyroZ();

		float correction = kP_heading * headingErr + kP_gyro * gyroErr;

		// With forward negative, to slow the side that's ahead, move its PWM toward 0
		int pwmA = basePWM + motorTrim + (int)correction; // slow A if ahead
		int pwmB = basePWM - motorTrim - (int)correction; // speed up B if behind

		// Keep both motors in forward range (negative), avoid sign flip
		pwmA = constrain(pwmA, -maxPWM, minForwardPWM);
		pwmB = constrain(pwmB, -maxPWM, minForwardPWM);

		setMotorA(pwmA);
		setMotorB(pwmB);

		// small sleep to allow ISR updates
		delay(10);

		// watchdog UI every 100ms
		if (millis() - lastUpdate > 100) {
			display.clearDisplay();
			display.setCursor(0,0);
			display.setTextSize(1);
			display.setRotation(2);
			display.setTextColor(SSD1306_WHITE);
			display.printf("Straight: %.1f/%.1f mm\n", avg_mm, target_mm);
			display.printf("A:%ld B:%ld dA:%ld dB:%ld\n", ticksA, ticksB, dA, dB);
			display.printf("pA:%d pB:%d\n", pwmA, pwmB);
			display.display();
			lastUpdate = millis();
		}
	}

	stopMotors();
	delay(200);
}

// Turn precisely ~90 degrees using differential drive; use encoders symmetric and optional gyro accumulation
void turn90(bool clockwise){
	// Pivot in place using gyro to detect 90 degrees for robustness
	int turnPWM = 300;
	float targetDeg = 90.0f;
	float accumulatedDeg = 0.0f;
	unsigned long last = micros();

	if (clockwise) {
		setMotorA(-turnPWM);  // A forward
		setMotorB(turnPWM);   // B reverse
	} else {
		setMotorA(turnPWM);   // A reverse
		setMotorB(-turnPWM);  // B forward
	}

	while (accumulatedDeg < targetDeg) {
		unsigned long now = micros();
		float dt = (now - last) / 1000000.0f; // seconds
		last = now;
		float z = getGyroZ();                 // deg/s
		// Direction: for clockwise, expect negative or positive depending on IMU orientation.
		// Use absolute contribution to ensure accumulation.
		accumulatedDeg += fabs(z) * dt;
		delay(2);
	}

	stopMotors();
	delay(300);
}

void setup() {
	Serial.begin(115200);
	Wire.begin(SDA_PIN, SCL_PIN);

	// OLED init
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setRotation(2);
	display.clearDisplay();
	display.setCursor(0, 0);
	display.println("Square Straight Init...");
	display.display();

	// IMU init
	imu.initialize();

	setupEncoderInterrupt();
	setupMotorPWM();

	stopMotors();
	delay(300);
}

void loop() {
	// Trace a square: 4 sides
	const float side_cm = 20.0; // short distance per side; adjust

	for (int i = 0; i < 4; i++) {
		straight(side_cm);           // heading correction only inside here
		turn90(true);                // 90° clockwise
	}

	// After finishing, stop and idle
	stopMotors();
	while (true) {
		display.clearDisplay();
		display.setCursor(0,0);
		display.setTextSize(1);
		display.setRotation(2);
		display.setTextColor(SSD1306_WHITE);
		display.println("Square complete");
		display.display();
		delay(1000);
	}
}

