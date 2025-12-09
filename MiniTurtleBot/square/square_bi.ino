// MiniTurtleBot: trace right-facing square then left-facing square
// Board: Arduino Nano ESP32 (ESP32-S3)

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- MOTOR DRIVER PINS (match MotorEncoder_code.ino) ----------
#define MTA1 A6
#define MTA2 A7
#define MTB1 D8
#define MTB2 D9

// ---------- PWM CONFIG ----------
const int pwmFreq = 500;           // Hz
const int pwmResolution = 10;      // bits (0..1023)
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- I2C (OLED) ----------
#define SDA_PIN A4
#define SCL_PIN A5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- Encoders ----------
#define MEA1 D10
#define MEA2 D11
#define MEB1 D12
#define MEB2 D13

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

// 12 ticks per channel, using 4x decoding; gear ratio if applicable
const int ticksPerRev = 12 * 4;
const float gearRatio = 30.0; // update if different on your gearbox

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

// ---------- Robot Geometry ----------
const float wheelRadius_m = 0.014; // 28 mm diameter wheel -> 0.014 m radius
const float trackWidth_m  = 0.089; // distance between left/right wheel contacts

// ---------- Motion Parameters (Straight) ----------
// Per-wheel forward power for straight segments (tune individually)
const int FORWARD_POWER_A = 500;   // RIGHT Motor forward power (straight)
const int FORWARD_POWER_B = 600;   // LEFT Motor forward power (straight)
const int BURST_MS = 1350;         // forward burst duration
const int STOP_MS = 250;           // stop between actions

// ---------- Motion Parameters (Right Turn) ----------
// Per-wheel turn power for right turns (A forward, B backward)
const int TURN_RIGHT_POWER_A = 500;   // Motor A power during right turn (forward)
const int TURN_RIGHT_POWER_B = 520;   // Motor B power during right turn (reverse)
const int TURN_RIGHT_MS = 260;        // approx 90° (tune)

// ---------- Motion Parameters (Left Turn) ----------
// Per-wheel turn power for left turns (A backward, B forward)
const int TURN_LEFT_POWER_A = 520;    // Motor A power during left turn (reverse)
const int TURN_LEFT_POWER_B = 500;    // Motor B power during left turn (forward)
const int TURN_LEFT_MS = 260;         // approx 90° (tune)

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

// Optional: simple ramp to reduce jerk
void rampMotors(int targetA, int targetB, int steps = 6, int stepDelay = 20) {
	int a = 0, b = 0;
	for (int i = 1; i <= steps; i++) {
		a = (targetA * i) / steps;
		b = (targetB * i) / steps;
		setMotorA(a);
		setMotorB(b);
		delay(stepDelay);
	}
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

	// I2C/OLED init
	Wire.begin(SDA_PIN, SCL_PIN);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.clearDisplay();
	display.setTextSize(1);
	display.setRotation(2);
	display.setTextColor(SSD1306_WHITE);
	display.display();

	// Encoders
	setupEncoderInterrupt();

	stopMotors();
	delay(300);
}

// ---------- State Machine ----------
enum MoveState { START_RIGHT, FWD_RIGHT, PAUSE_AFTER_FWD_RIGHT, TURN_RIGHT_S, PAUSE_AFTER_TURN_RIGHT,
				 EXTRA_TURN_RIGHT, PAUSE_AFTER_EXTRA_TURN_RIGHT, EXTRA_FWD_RIGHT, PAUSE_AFTER_EXTRA_FWD_RIGHT,
				 START_LEFT, FWD_LEFT, PAUSE_AFTER_FWD_LEFT, TURN_LEFT_S, PAUSE_AFTER_TURN_LEFT,
				 DONE };
MoveState state = START_RIGHT;
unsigned long t0 = 0;
int sidesRemainingRight = 4;  // repeat forward + right turn 4 times
int sidesRemainingLeft = 4;   // repeat forward + left turn 4 times
bool leftInitialTurnDone = false; // ensure first left action is a turn, not a straight

void loop() {
	// --- Telemetry update every 100 ms ---
	static unsigned long lastTelem = 0;
	static long lastCountA = 0, lastCountB = 0;
	unsigned long now = millis();
	if (now - lastTelem >= 100) {
		long dA = encoderCountA - lastCountA;
		long dB = encoderCountB - lastCountB;
		lastCountA = encoderCountA;
		lastCountB = encoderCountB;

		float ticksPerSecA = dA * 10.0f;
		float ticksPerSecB = dB * 10.0f;
		// gearbox output shaft RPM (signed)
		float rpmA = (ticksPerSecA * 60.0f) / (ticksPerRev * gearRatio);
		float rpmB = (ticksPerSecB * 60.0f) / (ticksPerRev * gearRatio);

		// Tangential wheel speeds (m/s)
		const float twoPiR_over_60 = (2.0f * 3.1415926f * wheelRadius_m) / 60.0f;
		float v_right = rpmA * twoPiR_over_60; // Motor A = Right
		float v_left  = rpmB * twoPiR_over_60; // Motor B = Left

		// Differential drive linear and angular velocity
		float v_lin = 0.5f * (v_right + v_left);             // m/s
		float v_ang = (v_right - v_left) / trackWidth_m;     // rad/s

		// OLED render
		display.clearDisplay();
		display.setCursor(0, 0);
		display.printf("A(R) RPM: % .1f\n", rpmA);
		display.printf("B(L) RPM: % .1f\n", rpmB);
		display.printf("v: % .3f m/s\n", v_lin);
		display.printf("w: % .2f rad/s\n", v_ang);
		display.display();

		lastTelem = now;
	}

	switch (state) {
		// ---- Right-facing square ----
		case START_RIGHT:
			if (sidesRemainingRight <= 0) {
				state = START_LEFT;
				break;
			}
			rampMotors(FORWARD_POWER_A, FORWARD_POWER_B);
			t0 = millis();
			state = FWD_RIGHT;
			break;

		case FWD_RIGHT:
			setMotorA(FORWARD_POWER_A);
			setMotorB(FORWARD_POWER_B);
			if (millis() - t0 >= BURST_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_FWD_RIGHT;
			}
			break;

		case PAUSE_AFTER_FWD_RIGHT:
			if (millis() - t0 >= STOP_MS) {
				// Right turn: A forward, B backward (per-wheel tuning)
				rampMotors(TURN_RIGHT_POWER_A, -TURN_RIGHT_POWER_B);
				t0 = millis();
				state = TURN_RIGHT_S;
			}
			break;

		case TURN_RIGHT_S:
			setMotorA(TURN_RIGHT_POWER_A);
			setMotorB(-TURN_RIGHT_POWER_B);
			if (millis() - t0 >= TURN_RIGHT_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_TURN_RIGHT;
			}
			break;

		case PAUSE_AFTER_TURN_RIGHT:
			if (millis() - t0 >= STOP_MS) {
				sidesRemainingRight--;
				if (sidesRemainingRight > 0) {
					state = START_RIGHT;
				} else {
					// Completed the 4th right turn; perform one extra 90° right turn
					rampMotors(TURN_RIGHT_POWER_A, -TURN_RIGHT_POWER_B);
					t0 = millis();
					state = EXTRA_TURN_RIGHT;
				}
			}
			break;

		case EXTRA_TURN_RIGHT:
			setMotorA(TURN_RIGHT_POWER_A);
			setMotorB(-TURN_RIGHT_POWER_B);
			if (millis() - t0 >= TURN_RIGHT_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_EXTRA_TURN_RIGHT;
			}
			break;

		case PAUSE_AFTER_EXTRA_TURN_RIGHT:
			if (millis() - t0 >= STOP_MS) {
				// After extra right turn, do one straight segment
				rampMotors(FORWARD_POWER_A, FORWARD_POWER_B);
				t0 = millis();
				state = EXTRA_FWD_RIGHT;
			}
			break;

		case EXTRA_FWD_RIGHT:
			setMotorA(FORWARD_POWER_A);
			setMotorB(FORWARD_POWER_B);
			if (millis() - t0 >= BURST_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_EXTRA_FWD_RIGHT;
			}
			break;

		case PAUSE_AFTER_EXTRA_FWD_RIGHT:
			if (millis() - t0 >= STOP_MS) {
				// Now begin the left-facing square
				state = START_LEFT;
			}
			break;

		// ---- Left-facing square ----
		case START_LEFT:
			if (sidesRemainingLeft <= 0) {
				state = DONE;
				break;
			}
			if (!leftInitialTurnDone) {
				// Begin left-facing sequence with an immediate left turn
				rampMotors(-TURN_LEFT_POWER_A, TURN_LEFT_POWER_B);
				t0 = millis();
				state = TURN_LEFT_S;
				leftInitialTurnDone = true;
			} else {
				rampMotors(FORWARD_POWER_A, FORWARD_POWER_B);
				t0 = millis();
				state = FWD_LEFT;
			}
			break;

		case FWD_LEFT:
			setMotorA(FORWARD_POWER_A);
			setMotorB(FORWARD_POWER_B);
			if (millis() - t0 >= BURST_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_FWD_LEFT;
			}
			break;

		case PAUSE_AFTER_FWD_LEFT:
			if (millis() - t0 >= STOP_MS) {
				// Left turn: A backward, B forward (per-wheel tuning)
				rampMotors(-TURN_LEFT_POWER_A, TURN_LEFT_POWER_B);
				t0 = millis();
				state = TURN_LEFT_S;
			}
			break;

		case TURN_LEFT_S:
			setMotorA(-TURN_LEFT_POWER_A);
			setMotorB(TURN_LEFT_POWER_B);
			if (millis() - t0 >= TURN_LEFT_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_TURN_LEFT;
			}
			break;

		case PAUSE_AFTER_TURN_LEFT:
			if (millis() - t0 >= STOP_MS) {
				sidesRemainingLeft--;
				if (sidesRemainingLeft > 0) {
					state = START_LEFT;
				} else {
					state = DONE;
				}
			}
			break;

		case DONE:
			stopMotors();
			delay(50);
			break;
	}
}

