#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MPU6050.h"  // Electronic Cats library
#include <WiFi.h>
#include <WebServer.h>

// ---------- I2C ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- MOTOR DRIVER (ESP32-S3 LEDC PWM) ----------
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
const int pwmResolution = 10; // duty 0..1023
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- IMU ----------
MPU6050 imu;
bool imuOk = false;

// ---------- Encoder & Motion ----------
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

// 12 ticks per channel, use 4x decoding; update if different
const int ticksPerRev = 12 * 4;
const float wheelDiameterMM = 65.0;  // example 65mm mini wheel; adjust to your robot
const float gearRatio = 30.0;        // set per gearbox
const float mmPerWheelRev = PI * wheelDiameterMM;
const float mmPerMotorRev = mmPerWheelRev / gearRatio;
const float mmPerTick = mmPerMotorRev / ticksPerRev;
const float trackWidthMM = 120.0f; // approx wheel-to-wheel distance; adjust to your robot

#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

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

// ---------- Motor control ----------
static int motorPowerA = 0; // -1023..1023
static int motorPowerB = 0; // -1023..1023

void setMotorA(int pwm) {
	pwm = constrain(pwm, -1023, 1023);
	if (pwm >= 0) { ledcWrite(pwmA1_Ch, pwm); ledcWrite(pwmA2_Ch, 0); }
	else { ledcWrite(pwmA1_Ch, 0); ledcWrite(pwmA2_Ch, -pwm); }
}
void setMotorB(int pwm) {
	pwm = constrain(pwm, -1023, 1023);
	if (pwm >= 0) { ledcWrite(pwmB1_Ch, 0); ledcWrite(pwmB2_Ch, pwm); }
	else { ledcWrite(pwmB1_Ch, -pwm); ledcWrite(pwmB2_Ch, 0); }
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

// ---------- Simple PID ----------
struct PID {
	float kp, ki, kd;
	float i, lastErr;
	float iMin, iMax;
	void reset(){ i=0; lastErr=0; }
	float update(float err, float dt){
		i += err * dt;
		if (i > iMax) i = iMax; else if (i < iMin) i = iMin;
		float d = (err - lastErr) / dt;
		lastErr = err;
		return kp*err + ki*i + kd*d;
	}
};

// Distance control (mm) and heading control (deg)
PID distPid{0.8f, 0.0f, 0.05f, 0.0f, -500.0f, 500.0f};
PID headPid{5.0f, 0.02f, 0.05f, 0.0f, -300.0f, 300.0f};
// Additional direct yaw-based trim (PWM per degree of yaw error)
float kYawTrim = 8.0f; // tune: increase if not straight enough; lower if oscillating

// ---------- Motion goal ----------
volatile long startCountA = 0, startCountB = 0;
float targetSec = 0.0f; // run duration target in seconds
bool driving = false;
float baseMaxPwm = 600; // default speed limit when going straight
float headingRefDeg = 0.0f; // target heading
unsigned long startMs = 0; // timestamp when GO pressed

// ---------- IMU helpers ----------
static float yawDeg = 0.0f;
static float gyroZ_dps = 0.0f; // IMU angular velocity (deg/s)
static float linVel_mm_s = 0.0f; // linear velocity from encoders (mm/s)
static float angVel_deg_s = 0.0f; // angular velocity (deg/s)

// basic complimentary-like yaw from gyro Z only; for simplicity
void updateIMU(float dt){
	if (!imuOk) return;
	int16_t gx, gy, gz; // raw gyro
	// Electronic Cats MPU6050: getRotation returns deg/s scaled by 131 if raw; here use getRotation
	imu.getRotation(&gx, &gy, &gz);
	gyroZ_dps = gz / 131.0f; // approx for ±250 dps range
	yawDeg += gyroZ_dps * dt; // integrate
	if (yawDeg > 180) yawDeg -= 360; else if (yawDeg < -180) yawDeg += 360;
}

// ---------- Utility ----------
float ticksToMM(long ticksA, long ticksB){
	float mmA = ticksA * mmPerTick;
	float mmB = ticksB * mmPerTick;
	return (mmA + mmB) * 0.5f;
}

void stopMotion(){
	driving = false;
	setMotorA(0); setMotorB(0);
}

// ---------- Setup ----------
// ---------- Wi-Fi control (declare before use) ----------
WebServer server(80);
const char* apSSID = "MiniTurtleBot";
const char* apPASS = "turtle123"; // change as desired

String htmlPage();
void setupWiFiServer();

// ---------- Serial commands disabled ----------
void handleSerial(){ /* no-op (wireless control via Wi-Fi) */ }

void setup() {
	Serial.begin(115200);
	Serial.setTimeout(20);

	Wire.begin(SDA_PIN, SCL_PIN);

	// OLED
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.clearDisplay();
	display.setTextSize(1);
	display.setRotation(2);
	display.setTextColor(SSD1306_WHITE);
	display.setCursor(0,0);
	display.println("Straight Line Ctrl");
	display.display();

	// IMU
	imu.initialize();
	imuOk = imu.testConnection();
	if (imuOk){ yawDeg = 0; }

	// Motors + Encoders
	setupMotorPWM();
	setupEncoderInterrupt();

	// Idle message
	stopMotion();

	// Wi-Fi AP + HTTP server
	setupWiFiServer();
}

// ---------- Command Interface ----------
// GO <cm> <pwm>  (e.g., GO 100 700)
// STOP
// Removed: legacy Serial command handler (Wi‑Fi based control used)

// ---------- Main loop ----------
void loop() {
	static unsigned long lastMs = millis();
	unsigned long now = millis();
	float dt = (now - lastMs) / 1000.0f;
	if (dt <= 0) dt = 0.001f;
	lastMs = now;

	// HTTP server
	server.handleClient();
	updateIMU(dt);

	// Telemetry calc (time-based)
	long dA = encoderCountA - startCountA;
	long dB = encoderCountB - startCountB;
	float elapsedSec = driving ? (millis() - startMs) / 1000.0f : 0.0f;

	// Estimate velocities
	static long prevCountA = 0, prevCountB = 0;
	long dTicksA_dt = (encoderCountA - prevCountA);
	long dTicksB_dt = (encoderCountB - prevCountB);
	prevCountA = encoderCountA;
	prevCountB = encoderCountB;
	if (dt > 0) {
		float vA = (dTicksA_dt * mmPerTick) / dt; // mm/s
		float vB = (dTicksB_dt * mmPerTick) / dt; // mm/s
		linVel_mm_s = 0.5f * (vA + vB);
		// Use IMU for angular velocity if available; else estimate from encoders
		if (imuOk) angVel_deg_s = gyroZ_dps;
		else {
			float omega_rad_s = (vB - vA) / trackWidthMM; // rad/s
			angVel_deg_s = omega_rad_s * (180.0f / PI);
		}
	}

	if (driving){
		// Use constant user-commanded speed; time only governs stop
		float baseTarget = -baseMaxPwm; // negative = forward
		float ramp = elapsedSec < 0.5f ? (elapsedSec / 0.5f) : 1.0f;
		float base = baseTarget * ramp;

		// Heading PID:
		float headErr = 0.0f;
		if (imuOk){
			// shortest angular error
			headErr = headingRefDeg - yawDeg;
			if (headErr > 180) headErr -= 360; else if (headErr < -180) headErr += 360;
		} else {
			// Fallback: use encoder difference as heading proxy
			headErr = (float)(dA - dB) * 0.02f; // tune factor
		}
		// Deadband to avoid chattering on small yaw errors
		if (fabs(headErr) < 0.5f) headErr = 0.0f;

		// PID turn plus direct yaw trim for stronger correction
		float pidTurn = headPid.update(headErr, dt);
		float trimTurn = kYawTrim * headErr; // PWM units per degree
		float turn = pidTurn + trimTurn;

		// Limit turn to an adaptive fraction of base to avoid stopping a wheel
		float maxTurn = 0.55f * fabs(base);
		if (maxTurn < 100.0f) maxTurn = 100.0f;
		if (turn > maxTurn) turn = maxTurn; else if (turn < -maxTurn) turn = -maxTurn;

		float leftCmd = base - turn;
		float rightCmd = base + turn;

		motorPowerA = (int)constrain(leftCmd, -1023, 1023);
		motorPowerB = (int)constrain(rightCmd, -1023, 1023);
		setMotorA(motorPowerA);
		setMotorB(motorPowerB);

		// Stop when target time reached
		if (elapsedSec >= targetSec) {
			stopMotion();
		}
	}

	// OLED status
	display.clearDisplay();
	display.setCursor(0,0);
	display.printf("Drv:%d Tgt:%.1fs\n", driving, targetSec);
	display.printf("Time:%.1fs\n", elapsedSec);
	display.printf("Yaw:%.1f deg IMU:%d\n", yawDeg, imuOk);
	display.printf("A:%d B:%d\n", motorPowerA, motorPowerB);
	display.printf("V:%.0f mm/s  w:%.1f deg/s\n", linVel_mm_s, angVel_deg_s);
	display.display();

	// Loop rate ~50Hz
	delay(20);
}

// ---------- Wi-Fi control ----------
// Configure soft-AP and a tiny HTTP control server

String htmlPage(){
	// minimal control UI
	String h;
	h += "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>";
	h += "<style>body{font-family:sans-serif;margin:16px}button{padding:10px 16px;margin:6px;font-size:16px}</style></head><body>";
	h += "<h3>Mini TurtleBot - Straight Line</h3>";
	h += "<div><input id=sec type=number value=5 min=0.1 step=0.1 style='width:80px'> sec \n";
	h += "<input id=pwm type=number value=700 min=150 max=1023 style='width:80px'> pwm</div>";
	h += "<div><button onclick=go()>GO</button><button onclick=stop()>STOP</button></div>";
	h += "<pre id=stat></pre>";
	h += "<script>function go(){const sec=document.getElementById('sec').value;const pwm=document.getElementById('pwm').value;fetch(`/go?sec=${sec}&pwm=${pwm}`)};function stop(){fetch('/stop')};async function tick(){try{const r=await fetch('/status');const j=await r.json();document.getElementById('stat').textContent=JSON.stringify(j,null,2);}catch(e){} setTimeout(tick,500);} tick();</script>";
	h += "</body></html>";
	return h;
}

void setupWiFiServer(){
	WiFi.mode(WIFI_AP);
	WiFi.softAP(apSSID, apPASS);

	server.on("/", HTTP_GET, [](){ server.send(200, "text/html", htmlPage()); });
	server.on("/go", HTTP_GET, [](){
		if (!server.hasArg("sec")) { server.send(400, "text/plain", "missing sec"); return; }
		float sec = server.arg("sec").toFloat();
		int pwm = baseMaxPwm;
		if (server.hasArg("pwm")) pwm = server.arg("pwm").toInt();

		targetSec = max(0.1f, sec);
		baseMaxPwm = constrain(pwm, 150, 1023);
		startCountA = encoderCountA; startCountB = encoderCountB;
		distPid.reset(); headPid.reset();
		headingRefDeg = yawDeg; // lock current heading
		startMs = millis();
		driving = true;
		// Provide an initial kick in the forward direction using the same convention (-PWM forward)
		int kick = -max(150, (int)baseMaxPwm/4);
		setMotorA(kick);
		setMotorB(kick);
		server.send(200, "text/plain", "OK");
	});

	server.on("/stop", HTTP_GET, [](){ stopMotion(); server.send(200, "text/plain", "OK"); });

	server.on("/status", HTTP_GET, [](){
		long dA = encoderCountA - startCountA;
		long dB = encoderCountB - startCountB;
		float elapsedSec = driving ? (millis() - startMs) / 1000.0f : 0.0f;
		char buf[256];
		snprintf(buf, sizeof(buf),
			"{\"driving\":%s,\"target_sec\":%.1f,\"elapsed_sec\":%.1f,\"yaw_deg\":%.1f,\"imu\":%s,\"pwmA\":%d,\"pwmB\":%d}",
			driving?"true":"false", targetSec, elapsedSec, yawDeg, imuOk?"true":"false", motorPowerA, motorPowerB);
		server.send(200, "application/json", String(buf));
	});

	server.begin();
}

