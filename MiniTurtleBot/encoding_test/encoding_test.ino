// Minimal encoder-only sketch: tracks revolutions per wheel and shows on OLED

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- I2C ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- ENCODER PINS ----------
#define MEA1 D10
#define MEA2 D11
#define MEB1 D12
#define MEB2 D13

// ---------- Encoder state ----------
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

// ---------- Encoder Settings ----------
const int ticksPerRev = 12 * 4;  // 12 ticks per channel, 4x decoding
const float gearRatio = 58;     // Update if gear reduction is used

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

void setup() {
	Serial.begin(115200);
	Wire.begin(SDA_PIN, SCL_PIN);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(SSD1306_WHITE);
	display.setRotation(2); // match orientation used previously

	setupEncoderInterrupt();
}

void loop() {
	static long lastCountA = 0, lastCountB = 0;
	static unsigned long lastTime = 0;

	unsigned long now = millis();
	if (now - lastTime >= 200) { // simple periodic update
		long countA = encoderCountA;
		long countB = encoderCountB;

		// Revolutions at the motor shaft (before gearbox) considering gear ratio
		// Each wheel revolution equals ticksPerRev * gearRatio encoder ticks
		float revA = (float)countA / (ticksPerRev * gearRatio);
		float revB = (float)countB / (ticksPerRev * gearRatio);

		display.clearDisplay();
		display.setCursor(0, 0);
		display.printf("Wheel A\nCounts: %ld\nRevs: %.3f\n\n", countA, revA);
		display.printf("Wheel B\nCounts: %ld\nRevs: %.3f", countB, revB);
		display.display();

		lastTime = now;
	}
}
