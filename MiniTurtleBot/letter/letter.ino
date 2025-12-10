#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ------------Motors-----------
#define MTA1 A6
#define MTA2 A7
#define MTB1 D8
#define MTB2 D9 

#define MEA1 D10
#define MEA2 D11
#define MEB1 D12
#define MEB2 D13

const int pwmFreq = 500;
const int pwmResolution = 10;
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
volatile uint8_t prevStateA = 0, prevStateB = 0;

const int ticksPerRev = 12 * 4;
const float gearRatio = 30.0;

#define READ_ENC_A() ((digitalRead(MEA1) << 1) | digitalRead(MEA2))
#define READ_ENC_B() ((digitalRead(MEB1) << 1) | digitalRead(MEB2))

void IRAM_ATTR handleEncoderA(){
  uint8_t state = READ_ENC_A();
  uint8_t combo = (prevStateA << 2) | state;
  switch (combo) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCountA++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCountA--; break;
  }
  prevStateA = state;
}

void IRAM_ATTR handleEncoderB(){
  uint8_t state = READ_ENC_B();
  uint8_t combo = (prevStateB << 2) | state;
  switch (combo) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: encoderCountB++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: encoderCountB--; break;
  }
  prevStateB = state;
}

void setMotorA(int pwm){
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) {
    ledcWrite(pwmA1_Ch, pwm);
    ledcWrite(pwmA2_Ch, 0);
  } else {
    ledcWrite(pwmA1_Ch, 0);
    ledcWrite(pwmA2_Ch, -pwm);
  }
}

void setMotorB(int pwm){
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

// ---------- Wi-Fi ----------
const char* ssid     = "TurtleBot";
const char* password = "123";

// ---------- UDP ----------
WiFiUDP udp;
const int UDP_PORT = 5601;
char incoming[32];

// ---------- Telemetry (encoder counts to controller) ----------
const int TELEM_PORT = 5602;               // PC receiver port
const unsigned long TELEM_PERIOD_MS = 50;  // ~20 Hz
IPAddress controllerIP;                    // learned from last command packet
bool controllerKnown = false;
unsigned long lastTelemMs = 0;

// ---------- I2C ----------
#define SDA A4
#define SCL A5

// ---------- LED ----------
#define LED1 D6
#define LED2 D7

void setup() {
  // OLED init
  Serial.begin(115200);  
  Serial.setTimeout(20);
  Wire.begin(SDA, SCL);  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);  // OLED init failed
  }
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.display();

  setupEncoderInterrupt();
  setupMotorPWM();

  // WiFi Connect
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    display.print(".");
    display.display();
  }

  // Display IP address
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connected");
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.display();
  delay(1500);

  // Start UDP
  udp.begin(UDP_PORT);
  display.println("UDP Ready");
  display.display();
  delay(1000);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incoming, sizeof(incoming) - 1);
    if (len > 0) incoming[len] = '\0';

    // Learn controller IP from the incoming command
    controllerIP = udp.remoteIP();
    controllerKnown = true;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("IP ESP: ");
    display.println(WiFi.localIP());
    display.println("IP from computer: "); display.println(udp.remoteIP());
    display.println("Received:");
    display.println(incoming);

    // LED control
    if (strcmp(incoming, "L1") == 0) {
      setMotorA(200); 
      setMotorB(-200);
    }
    else if (strcmp(incoming, "L0") == 0) {
    setMotorA(1); 
    setMotorB(1);      
    }

    else if (strcmp(incoming, "R1") == 0){
      setMotorA(-200); 
      setMotorB(200);
    } 
    else if (strcmp(incoming, "R0") == 0){
      setMotorA(1); 
      setMotorB(1);
    }
     
    else if (strcmp(incoming, "F1") == 0){
      setMotorA(200); 
      setMotorB(200);
    } 
    else if (strcmp(incoming, "F0") == 0){
      setMotorA(1); 
      setMotorB(1);
    } 
    else if (strcmp(incoming, "B1") == 0){
      setMotorA(-200); 
      setMotorB(-200);
    } 
    else if (strcmp(incoming, "B0") == 0){
      setMotorA(1); 
      setMotorB(1);
    } 

    display.display();
  }

  // Periodic telemetry: send absolute encoder counts to controller
  unsigned long now = millis();
  if (controllerKnown && (now - lastTelemMs >= TELEM_PERIOD_MS)) {
    lastTelemMs = now;
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "{\"A\":%ld,\"B\":%ld}", encoderCountA, encoderCountB);
    udp.beginPacket(controllerIP, TELEM_PORT);
    udp.write((const uint8_t*)buf, n);
    udp.endPacket();
  }
}