// MiniTurtleBot: forward, right 90°, forward
// Board: Arduino Nano ESP32 (ESP32-S3)

#include <Arduino.h>

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

// ---------- Motion Parameters ----------
// Per-wheel forward power for straight segments (tune individually)
const int FORWARD_POWER_A = 500;   // RIGHT Motor forward power (straight)
const int FORWARD_POWER_B = 600;   // LEFT Motor forward power (straight) 
// Per-wheel turn power for right turns (A forward, B backward)
const int TURN_POWER_A = 500;      // Motor A power during right turn (forward)
const int TURN_POWER_B = 520;      // Motor B power during right turn (reverse)
const int BURST_MS = 1350;          // forward burst duration
const int TURN_RIGHT_MS = 260;     // approx 90° (tune)
const int STOP_MS = 250;           // stop between actions

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

	stopMotors();
	delay(300);
}

// ---------- State Machine ----------
enum MoveState { START, FWD, PAUSE_AFTER_FWD, TURN_RIGHT, PAUSE_AFTER_TURN, DONE };
MoveState state = START;
unsigned long t0 = 0;
int sidesRemaining = 4;  // repeat forward + right turn 4 times to make a square

void loop() {
	switch (state) {
		case START:
			// Begin a new side: forward burst
			if (sidesRemaining <= 0) {
				state = DONE;
				break;
			}
			rampMotors(FORWARD_POWER_A, FORWARD_POWER_B);
			t0 = millis();
			state = FWD;
			break;

		case FWD:
			setMotorA(FORWARD_POWER_A);
			setMotorB(FORWARD_POWER_B);
			if (millis() - t0 >= BURST_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_FWD;
			}
			break;

		case PAUSE_AFTER_FWD:
			if (millis() - t0 >= STOP_MS) {
				// Right turn: A forward, B backward (per-wheel tuning)
				rampMotors(TURN_POWER_A, -TURN_POWER_B);
				t0 = millis();
				state = TURN_RIGHT;
			}
			break;

		case TURN_RIGHT:
			setMotorA(TURN_POWER_A);
			setMotorB(-TURN_POWER_B);
			if (millis() - t0 >= TURN_RIGHT_MS) {
				stopMotors();
				t0 = millis();
				state = PAUSE_AFTER_TURN;
			}
			break;

		case PAUSE_AFTER_TURN:
			if (millis() - t0 >= STOP_MS) {
				// Completed one side and a right turn; decrement and start next side
				sidesRemaining--;
				state = START;
			}
			break;

		case DONE:
			stopMotors();
			// hold here
			delay(50);
			break;
	}
}

