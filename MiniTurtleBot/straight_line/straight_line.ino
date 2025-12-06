// MiniTurtleBot: Drive straight with encoder-based heading correction
// Board: Arduino Nano ESP32 (ESP32-S3)

#include <Arduino.h>

// ---------- MOTOR DRIVER PINS (match MotorEncoder_code.ino) ----------
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
const int pwmFreq = 500;           // Hz
const int pwmResolution = 10;      // bits (0..1023)
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- Drive Parameters (tune these) ----------
const int BASE_POWER = 550;        // forward base power
const unsigned long DRIVE_MS = 2000; // duration to drive forward
const int MIN_EFFECTIVE_POWER = 120; // overcome static friction

// PID gains for heading correction using encoder difference
const float Kp = 0.9f;             // proportional
const float Ki = 0.0f;             // integral (start at 0)
const float Kd = 0.0f;             // derivative (optional)

// ---------- Encoders ----------
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

// ---------- Motor Helpers ----------
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

// ---------- Setup ----------
void setup() {
	// PWM init
	ledcSetup(pwmA1_Ch, pwmFreq, pwmResolution);
	ledcSetup(pwmA2_Ch, pwmFreq, pwmResolution);
	ledcSetup(pwmB1_Ch, pwmFreq, pwmResolution);
	ledcSetup(pwmB2_Ch, pwmFreq, pwmResolution);
	ledcAttachPin(MTA1, pwmA1_Ch);
	ledcAttachPin(MTA2, pwmA2_Ch);
	ledcAttachPin(MTB1, pwmB1_Ch);
	ledcAttachPin(MTB2, pwmB2_Ch);

	setupEncoderInterrupt();

	stopMotors();
	delay(300);
}

// ---------- PID variables ----------
float integralErr = 0.0f;
float lastErr = 0.0f;

// ---------- Drive straight ----------
void loop() {
	static bool driving = false;
	static unsigned long tStart = 0;

	if (!driving) {
		// Reset PID and encoders before starting
		noInterrupts();
		encoderCountA = 0;
		encoderCountB = 0;
		interrupts();
		integralErr = 0.0f;
		lastErr = 0.0f;

		tStart = millis();
		driving = true;
	}

	if (driving) {
		// Heading error: difference in encoder ticks (A - B)
		long countA, countB;
		noInterrupts();
		countA = encoderCountA;
		countB = encoderCountB;
		interrupts();

		float err = (float)(countA - countB);
		integralErr += err;
		float deriv = err - lastErr;
		lastErr = err;

		// PID output -> differential adjustment
		float adjust = Kp * err + Ki * integralErr + Kd * deriv;

		// Compute motor commands
		int cmdA = BASE_POWER - (int)adjust; // if A runs ahead (err>0), slow A
		int cmdB = BASE_POWER + (int)adjust; // and speed B to catch up

		// Ensure effective power over minimum
		if (cmdA > 0 && cmdA < MIN_EFFECTIVE_POWER) cmdA = MIN_EFFECTIVE_POWER;
		if (cmdB > 0 && cmdB < MIN_EFFECTIVE_POWER) cmdB = MIN_EFFECTIVE_POWER;

		setMotorA(cmdA);
		setMotorB(cmdB);

		// End after duration
		if (millis() - tStart >= DRIVE_MS) {
			stopMotors();
			driving = false;
			// Small pause before next cycle
			delay(500);
		}
	} else {
		// Idle – do nothing (or restart automatically)
		// Uncomment to auto-repeat: driving = true;
	}
}

