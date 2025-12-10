#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>

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

// ---------- PWM CONFIG ----------
const int pwmFreq = 500;
const int pwmResolution = 10;
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- Wi-Fi ----------
const char* ssid     = "SpectrumSetup-09"; // set to your WiFi SSID
const char* password = "leaderstatue564";       // set to your WiFi password

// ---------- UDP ----------
WiFiUDP udp;
const int CMD_PORT   = 5601; // PC -> ESP commands
const int LIDAR_PORT = 5603; // ESP -> PC lidar samples
char incoming[32];
IPAddress controllerIP; // learned from last command packet
bool controllerKnown = false;

// ---------- I2C ----------
#define SDA A4
#define SCL A5

// ---------- LiDAR ----------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ---------- Motor helpers ----------
void setMotorA(int pwm){
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) { ledcWrite(pwmA1_Ch, pwm); ledcWrite(pwmA2_Ch, 0); }
  else { ledcWrite(pwmA1_Ch, 0); ledcWrite(pwmA2_Ch, -pwm); }
}
void setMotorB(int pwm){
  pwm = constrain(pwm, -1023, 1023);
  if (pwm >= 0) { ledcWrite(pwmB1_Ch, 0); ledcWrite(pwmB2_Ch, pwm); }
  else { ledcWrite(pwmB1_Ch, -pwm); ledcWrite(pwmB2_Ch, 0); }
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

// ---------- Spin scan ----------
// Basic timed spin for ~360 deg; tune duration and speed for your robot
const int spinPWM = 220;            // wheel PWM for spin
const unsigned long spinMs = 2500;  // total spin time to approximate 360°

void sendLidarSample(float angDeg, int distCm){
  if (!controllerKnown) return;
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "{\"ang_deg\":%.1f,\"dist_cm\":%d}", angDeg, distCm);
  udp.beginPacket(controllerIP, LIDAR_PORT);
  udp.write((const uint8_t*)buf, n);
  udp.endPacket();
}

void performSpinScan(){
  // Indicate scanning
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Scanning 360...");
  display.display();

  VL53L0X_RangingMeasurementData_t measure;
  unsigned long start = millis();
  unsigned long now;
  // Start spin: opposite wheel directions
  setMotorA(spinPWM);
  setMotorB(-spinPWM);
  while ((now = millis()) - start < spinMs){
    // Fraction of spin -> angle
    float frac = (float)(now - start) / (float)spinMs;
    float angDeg = frac * 360.0f;
    // Lidar measurement
    lox.rangingTest(&measure, false);
    int distCm = 0;
    if (measure.RangeStatus != 4){
      distCm = (int)(measure.RangeMilliMeter / 10);
    }
    // Send sample
    sendLidarSample(angDeg, distCm);
    // Mild pacing
    delay(50);
  }
  // Stop motors
  setMotorA(0);
  setMotorB(0);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Scan complete");
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  Wire.begin(SDA, SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { while (1); }
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Heatmap Mode");
  display.display();

  setupMotorPWM();

  // LIDAR init
  if (!lox.begin()){
    display.clearDisplay(); display.setCursor(0,0); display.println("LiDAR ERR"); display.display();
  }

  // WiFi Connect
  WiFi.begin(ssid, password);
  display.clearDisplay(); display.setCursor(0,0); display.println("Connecting WiFi..."); display.display();
  while (WiFi.status() != WL_CONNECTED) { delay(300); display.print("."); display.display(); }

  // IP and UDP
  display.clearDisplay(); display.setCursor(0,0); display.println("WiFi OK"); display.print("IP: "); display.println(WiFi.localIP()); display.display();
  udp.begin(CMD_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incoming, sizeof(incoming) - 1);
    if (len > 0) incoming[len] = '\0';
    controllerIP = udp.remoteIP();
    controllerKnown = true;

    // Basic drive commands (same mapping as letter)
    if (strcmp(incoming, "L1") == 0){ setMotorA(200); setMotorB(-200); }
    else if (strcmp(incoming, "L0") == 0){ setMotorA(0); setMotorB(0); }
    else if (strcmp(incoming, "R1") == 0){ setMotorA(-200); setMotorB(200); }
    else if (strcmp(incoming, "R0") == 0){ setMotorA(0); setMotorB(0); }
    else if (strcmp(incoming, "F1") == 0){ setMotorA(200); setMotorB(200); }
    else if (strcmp(incoming, "F0") == 0){ setMotorA(0); setMotorB(0); }
    else if (strcmp(incoming, "B1") == 0){ setMotorA(-200); setMotorB(-200); }
    else if (strcmp(incoming, "B0") == 0){ setMotorA(0); setMotorB(0); }
    else if (strcmp(incoming, "S1") == 0){
      performSpinScan();
    }

    // Status on OLED
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Heatmap Mode");
    display.print("ESP IP: "); display.println(WiFi.localIP());
    display.print("PC IP:  "); display.println(controllerIP);
    display.print("Cmd: "); display.println(incoming);
    display.display();
  }

  delay(10);
}
