#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

// ---------- I2C ----------
#define SDA_PIN A4
#define SCL_PIN A5

// ---------- Wi‑Fi / Server (declare before use) ----------
WebServer server(80);
const char* apSSID = "MiniTurtle-Line";
const char* apPASS = "turtle123";
void setupWiFiPanel();

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

// ---------- PWM CONFIG ----------
const int pwmFreq = 500;
const int pwmResolution = 10; // duty 0..1023
const int pwmA1_Ch = 0;
const int pwmA2_Ch = 1;
const int pwmB1_Ch = 2;
const int pwmB2_Ch = 3;

// ---------- IR SENSORS ----------
#define IR_LEFT  D2   // adjust if wiring differs; must be ADC-capable pin
#define IR_RIGHT A1   // ADC-capable

// Expected values: ~1700 on black line, ~3900-4000 on white
// We'll invert readings so higher means "more line" (black)
inline int readIRLeft()  { return analogRead(IR_LEFT); }
inline int readIRRight() { return analogRead(IR_RIGHT); }

// ---------- Motor control ----------
static int motorPowerA = 0; // -1023..1023 (negative = forward)
static int motorPowerB = 0; // -1023..1023 (negative = forward)

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

// ---------- Line following control ----------
struct PID { float kp, ki, kd; float i, last; float iMin, iMax; void reset(){i=0; last=0;} float update(float e,float dt){ i+=e*dt; if(i>iMax)i=iMax; if(i<iMin)i=iMin; float d=(e-last)/dt; last=e; return kp*e+ki*i+kd*d; }};

// Error is based on differential reflectance: left-black minus right-black
// Convert raw ADC (0..4095) to line-affinity: lineAffinity = clamp((whiteRef - adc), 0..range)
// For simplicity, normalize: lineAffinity = (whiteRef - adc)
// Threshold-based ranges:
// White range (needs adjustment): adc > 3000
// Black range (on track):        adc < 3000
int lineThreshold = 3000; // made mutable so it can be tuned via web UI

// Tunables
int basePWMLeft  = 175;      // user base speed for left wheel (forward uses negative PWM)
int basePWMRight = 210;      // user base speed for right wheel (forward uses negative PWM)
int minPWM  = 110;           // minimum per-wheel to avoid stall
float steerGain = 0.08f;     // direct proportional steering from sensor error (PWM per normalized unit)
PID steerPid{0.10f, 0.0f, 0.02f, 0.0f, -300.0f, 300.0f}; // mild PID on sensor error

// Helper to normalize sensor towards [0..1] line affinity (1 = black, 0 = white)
inline float affinity(int adc){
  // Binary affinity: 1.0 when on black (adc < threshold), 0.0 on white
  return adc < lineThreshold ? 1.0f : 0.0f;
}

void setup(){
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Line Follow");
  display.display();

  // Sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Motors
  setupMotorPWM();

  // Start moving forward gently
  setMotorA(-minPWM);
  setMotorB(-minPWM);

  // Wi‑Fi control panel
  setupWiFiPanel();
}
// Pause/resume control from web UI
volatile bool paused = false;

// Simple state machine for recovery when off the black line
enum FollowState { NORMAL_FOLLOW, BACKUP, RECOVER };
FollowState state = NORMAL_FOLLOW;
unsigned long stateStartMs = 0;
unsigned long backupDurationMs = 500;  // reverse for this long (mutable)
unsigned long recoverDurationMs = 250; // apply corrective turn after backup (mutable)
int recoverTurnPWM = 0; // signed turn magnitude used during RECOVER

void loop(){
  static unsigned long lastMs = millis();
  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  lastMs = now;

  // Service HTTP requests (Wi‑Fi tuning panel)
  server.handleClient();

  // If paused, keep motors stopped and skip control logic
  if (paused) {
    setMotorA(0); setMotorB(0);
    motorPowerA = 0; motorPowerB = 0;
    // Minimal OLED status while paused
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("PAUSED");
    display.display();
    delay(20);
    return;
  }

  // Read sensors
  int rawL = readIRLeft();
  int rawR = readIRRight();
  float aL = affinity(rawL);
  float aR = affinity(rawR);

  // Error: positive if left sees more black than right (needs right motor faster or left slower)
  float err = aL - aR; // range roughly [-1..1]

  // Check off-line conditions: sensor reading indicates white (not black)
  bool leftWhite = (rawL > lineThreshold);
  bool rightWhite = (rawR > lineThreshold);

  // State transitions
  if (state == NORMAL_FOLLOW && (leftWhite || rightWhite)){
    state = BACKUP;
    stateStartMs = millis();
    // Choose recover direction opposite the white sensor
    // Positive recoverTurnPWM steers right wheel faster (turn right), negative steers left (turn left)
    int baseRecover = 140; // mild steering during RECOVER
    recoverTurnPWM = rightWhite ? -baseRecover : (leftWhite ? baseRecover : 0);
  }
  if (state == BACKUP && (millis() - stateStartMs >= backupDurationMs)){
    state = RECOVER;
    stateStartMs = millis();
  }
  if (state == RECOVER && (millis() - stateStartMs >= recoverDurationMs)){
    state = NORMAL_FOLLOW;
  }

  int leftCmd, rightCmd;
  int baseL = -constrain(basePWMLeft,  minPWM, 1023);
  int baseR = -constrain(basePWMRight, minPWM, 1023);

  if (state == BACKUP){
    // Reverse both motors briefly to back up
    int backPWM = constrain(minPWM + 40, minPWM, 1023);
    leftCmd  = backPWM;  // positive = reverse per motor driver convention
    rightCmd = backPWM;
  } else if (state == RECOVER){
    // Apply corrective turn while moving forward slowly
    int forwardPWM = -(minPWM + 20);
    leftCmd  = forwardPWM - recoverTurnPWM;
    rightCmd = forwardPWM + recoverTurnPWM;
    // Ensure minimum forward magnitude
    if (leftCmd > -minPWM)  leftCmd  = -minPWM;
    if (rightCmd > -minPWM) rightCmd = -minPWM;
  } else {
    // NORMAL_FOLLOW: Steering command = proportional + PID smoothing
    float avgBase = 0.5f * (basePWMLeft + basePWMRight);
    float steerP = steerGain * err * avgBase; // scale by average base speed
    float steerPID = steerPid.update(err, dt);
    float turn = steerP + steerPID; // in PWM units

    leftCmd  = baseL - (int)turn;
    rightCmd = baseR + (int)turn;

    // Keep wheels above minimum magnitude while moving forward
    if (leftCmd > -minPWM)  leftCmd  = -minPWM;
    if (rightCmd > -minPWM) rightCmd = -minPWM;
  }

  motorPowerA = constrain(leftCmd, -1023, 1023);
  motorPowerB = constrain(rightCmd, -1023, 1023);
  setMotorA(motorPowerA);
  setMotorB(motorPowerB);

  // Telemetry
  display.clearDisplay();
  display.setCursor(0,0);
  // Raw sensor values and normalized affinity
  display.printf("IR L:%4d R:%4d\n", rawL, rawR);
  display.printf("affL:%.2f affR:%.2f\n", aL, aR);
  // Error and steering
  if (state == NORMAL_FOLLOW) {
    float avgBase = 0.5f * (basePWMLeft + basePWMRight);
    float steerP = steerGain * err * avgBase;
    float steerPID = steerPid.last; // show last PID error contribution
    display.printf("err:%.2f\n", err);
  } else if (state == BACKUP) {
    display.printf("BACKUP %lums\n", (unsigned long)(backupDurationMs - (millis()-stateStartMs)));
  } else {
    display.printf("RECOVER turn:%d\n", recoverTurnPWM);
  }
  // Motor commands and per-wheel bases
  display.printf("baseL:%d baseR:%d\n", baseL, baseR);
  display.printf("PWM A:%d B:%d\n", motorPowerA, motorPowerB);

  // Simple directional indicator (arrow left/right based on error)
  int cx = 64, cy = 54; // near bottom center
  int dx = (int)(err * 20); // scale to pixels
  display.drawLine(cx-15, cy, cx+15, cy, SSD1306_WHITE); // baseline
  display.fillCircle(cx + dx, cy, 2, SSD1306_WHITE);
  if (state == BACKUP) {
    display.setCursor(40, 54); display.print("<< BACKUP >>");
  } else if (state == RECOVER) {
    if (recoverTurnPWM < 0) { display.setCursor(0, 54); display.print("< LEFT TURN"); }
    else if (recoverTurnPWM > 0) { display.setCursor(80, 54); display.print("RIGHT TURN >"); }
    else { display.setCursor(48, 54); display.print("CENTER"); }
  } else {
    if (dx < -2) { display.setCursor(0, 54); display.print("< LEFT"); }
    else if (dx > 2) { display.setCursor(96, 54); display.print("RIGHT >"); }
    else { display.setCursor(48, 54); display.print("CENTER"); }
  }
  display.display();

  delay(10); // ~100 Hz control
}

// ------------------- Live Tuning Web UI -------------------

String controlPage(){
  String h;
  h += "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'/>";
  h += "<style>body{font-family:sans-serif;margin:14px} input{width:90px} button{padding:8px 14px;margin:6px} .row{margin:6px 0}</style></head><body>";
  h += "<h3>MiniTurtleBot Line Follow Tuning</h3>";
  h += "<div class=row><button onclick=stop()>STOP</button> <button onclick=resume()>RESUME</button></div>";
  h += "<div class=row>baseL <input id=baseL type=number step=10 min=0 max=1023> baseR <input id=baseR type=number step=10 min=0 max=1023></div>";
  h += "<div class=row>minPWM <input id=minPWM type=number step=5 min=0 max=1023> steerGain <input id=steerGain type=number step=0.01></div>";
  h += "<div class=row>PID kp <input id=kp type=number step=0.01> ki <input id=ki type=number step=0.001> kd <input id=kd type=number step=0.01></div>";
  h += "<div class=row>threshold <input id=th type=number step=10 min=0 max=4095></div>";
  h += "<div class=row>backup(ms) <input id=backup type=number step=10 min=0> recover(ms) <input id=recover type=number step=10 min=0></div>";
  h += "<div class=row><button onclick=apply()>Apply</button> <button onclick=refresh()>Refresh</button></div>";
  h += "<pre id=stat></pre>";
    h += "<script>let editing=false;function bindEditGuards(){document.querySelectorAll('input').forEach(el=>{el.addEventListener('focus',()=>editing=true);el.addEventListener('blur',()=>editing=false);});}\n"
      "async function refresh(){const r=await fetch('/status');const j=await r.json();\n"
      "if(!editing){document.getElementById('baseL').value=j.baseL;document.getElementById('baseR').value=j.baseR;\n"
      "document.getElementById('minPWM').value=j.minPWM;document.getElementById('steerGain').value=j.steerGain;\n"
      "document.getElementById('kp').value=j.kp;document.getElementById('ki').value=j.ki;document.getElementById('kd').value=j.kd;\n"
      "document.getElementById('th').value=j.threshold;document.getElementById('backup').value=j.backupMs;document.getElementById('recover').value=j.recoverMs;}\n"
      "document.getElementById('stat').textContent=JSON.stringify(j,null,2);}\n"
       "async function apply(){const p=new URLSearchParams();\n"
       "p.append('baseL',document.getElementById('baseL').value);\n"
       "p.append('baseR',document.getElementById('baseR').value);\n"
       "p.append('minPWM',document.getElementById('minPWM').value);\n"
       "p.append('steerGain',document.getElementById('steerGain').value);\n"
       "p.append('kp',document.getElementById('kp').value);\n"
       "p.append('ki',document.getElementById('ki').value);\n"
       "p.append('kd',document.getElementById('kd').value);\n"
       "p.append('threshold',document.getElementById('th').value);\n"
       "p.append('backupMs',document.getElementById('backup').value);\n"
       "p.append('recoverMs',document.getElementById('recover').value);\n"
       "await fetch('/apply',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()});refresh();}\n"
       "function stop(){fetch('/stop');} function resume(){fetch('/resume');}\n"
      "bindEditGuards();refresh();setInterval(refresh,1000);</script>";
  h += "</body></html>";
  return h;
}

void setupWiFiPanel(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID, apPASS);
  IPAddress ip = WiFi.softAPIP();
  // Show AP IP on OLED for convenience
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Line Follow");
  display.print("AP IP: "); display.println(ip.toString());
  display.display();
  server.on("/", HTTP_GET, [](){ server.send(200, "text/html", controlPage()); });
  server.on("/status", HTTP_GET, [](){
    char buf[512];
    snprintf(buf, sizeof(buf),
      "{\"baseL\":%d,\"baseR\":%d,\"minPWM\":%d,\"steerGain\":%.3f,\"kp\":%.3f,\"ki\":%.4f,\"kd\":%.3f,\"threshold\":%d,\"backupMs\":%lu,\"recoverMs\":%lu}",
      basePWMLeft, basePWMRight, minPWM, steerGain, steerPid.kp, steerPid.ki, steerPid.kd, lineThreshold, (unsigned long)backupDurationMs, (unsigned long)recoverDurationMs);
    server.send(200, "application/json", String(buf));
  });
  server.on("/apply", HTTP_POST, [](){
    if (server.hasArg("baseL")) basePWMLeft = constrain(server.arg("baseL").toInt(), 0, 1023);
    if (server.hasArg("baseR")) basePWMRight = constrain(server.arg("baseR").toInt(), 0, 1023);
    if (server.hasArg("minPWM")) minPWM = constrain(server.arg("minPWM").toInt(), 0, 1023);
    if (server.hasArg("steerGain")) steerGain = server.arg("steerGain").toFloat();
    if (server.hasArg("kp")) steerPid.kp = server.arg("kp").toFloat();
    if (server.hasArg("ki")) steerPid.ki = server.arg("ki").toFloat();
    if (server.hasArg("kd")) steerPid.kd = server.arg("kd").toFloat();
    if (server.hasArg("threshold")) lineThreshold = constrain(server.arg("threshold").toInt(), 0, 4095);
    if (server.hasArg("backupMs")) backupDurationMs = (unsigned long)server.arg("backupMs").toInt();
    if (server.hasArg("recoverMs")) recoverDurationMs = (unsigned long)server.arg("recoverMs").toInt();
    server.send(200, "text/plain", "OK");
  });
  server.on("/stop", HTTP_GET, [](){
    paused = true;
    setMotorA(0); setMotorB(0);
    server.send(200, "text/plain", "OK");
  });
  server.on("/resume", HTTP_GET, [](){
    paused = false;
    server.send(200, "text/plain", "OK");
  });
  server.begin();
}
