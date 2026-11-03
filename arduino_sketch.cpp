/*
  DEXA - force-controlled adaptive hand (starter firmware)
  Board : ESP32 DevKit (WROOM-32)
  Libs  : Adafruit PWM Servo Driver Library (install via Library Manager)

  2-DAY BUILD (default): USE_PCA9685 = 0, 4 servos straight on ESP32 pins
    thumb GPIO13, index GPIO14, middle GPIO27, ring+pinky (one shared servo) GPIO26
    Servo red -> 5V/3A supply +, brown -> supply GND (common with ESP32 GND), orange -> GPIO
    Force sensors only on thumb(32), index(33), middle(34). Ring/pinky follow middle.

  Wiring (PCA9685 version, set USE_PCA9685 = 1)
  ------
  ESP32 GPIO21 (SDA) -> PCA9685 SDA      ESP32 GPIO22 (SCL) -> PCA9685 SCL
  ESP32 3V3 -> PCA9685 VCC (logic only)  ESP32 GND <-> PCA9685 GND <-> supply GND
  5V/3A supply -> PCA9685 V+ screw terminal (servo power) + 470-1000 uF cap across it
  Servos on PCA9685 channels 0..4 = thumb, index, middle, ring, pinky
  FSR divider per finger: 3V3 -- FSR -- node -- 10k -- GND ; node -> ESP32 ADC pin
  FSR pins (all ADC1, safe with Wi-Fi): thumb 32, index 33, middle 34, ring 35, pinky 36

  Control (Wi-Fi AP "DEXA", password "serbot123", open http://192.168.4.1)
  Serial (115200) shortcuts: p=power n=pinch t=tripod f=fragile o=open s=stop
*/

// ---- 2-DAY BUILD OPTIONS ----
#define USE_PCA9685 0   // 0 = servos wired straight to ESP32 pins (no PCA9685 needed)
                        // 1 = use a PCA9685 board over I2C (channels 0..4)
// Fingers that actually have a force sensor (bit0 thumb ... bit4 pinky).
// Unsensed fingers just follow the middle finger. Put ring+pinky tendons on ONE
// shared servo (index 3) and leave index 4 unused.
const uint8_t SENSED = 0b00111;   // thumb, index, middle

#include <WiFi.h>
#include <WebServer.h>
#if USE_PCA9685
  #include <Wire.h>
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm(0x40);
#else
  #include <ESP32Servo.h>     // install "ESP32Servo" from Library Manager
  Servo servos[5];
  // thumb, index, middle, ring+pinky (shared), spare
  const uint8_t SERVO_PIN[5] = {13, 14, 27, 26, 25};
#endif
WebServer server(80);

const uint8_t N = 5;
const uint8_t FSR_PIN[N] = {32, 33, 34, 35, 36};

// ---- CALIBRATE THESE on the real hand (servo angle in degrees) ----
// Closure 0.0 = finger open, 1.0 = finger fully curled. If a finger moves the
// wrong way, swap its OPEN and CLOSE values.
float OPEN_ANG[N]  = { 20,  20,  20,  20,  20};
float CLOSE_ANG[N] = {160, 160, 160, 160, 160};

const uint16_t SERVOMIN = 150;       // PCA9685 ticks at 50 Hz for ~0 deg
const uint16_t SERVOMAX = 600;       // ticks for ~180 deg

// ---- Control tuning ----
const float KP         = 0.00002f;   // closure change per ADC count of error
const float MAX_STEP   = 0.02f;      // max closure change per 20 ms cycle
const float DEADBAND   = 80.0f;      // ADC counts of tolerance around setpoint
const float SLIP_DROP  = 300.0f;     // force drop within 100 ms that counts as slip
const float SLIP_BOOST = 0.05f;      // extra closure applied on slip

struct Grasp { const char* name; uint8_t mask; float force; };
// mask bits: bit0 thumb, bit1 index, bit2 middle, bit3 ring, bit4 pinky
Grasp GRASPS[] = {
  {"POWER",   0b11111, 1500},
  {"PINCH",   0b00011,  800},
  {"TRIPOD",  0b00111,  800},
  {"FRAGILE", 0b11111,  400},
};
const int NGRASPS = sizeof(GRASPS) / sizeof(GRASPS[0]);

enum State { IDLE, GRASP, RELEASE };
State state = IDLE;
int   grasp = -1;

float force[N]   = {0};
float prevForce[N] = {0};
float closure[N] = {0};
int   slipCount  = 0;
unsigned long lastCtl = 0, lastSlipCheck = 0;

void writeFinger(uint8_t i) {
  float deg = OPEN_ANG[i] + closure[i] * (CLOSE_ANG[i] - OPEN_ANG[i]);
  float lo = min(OPEN_ANG[i], CLOSE_ANG[i]);
  float hi = max(OPEN_ANG[i], CLOSE_ANG[i]);
  deg = constrain(deg, lo, hi);
#if USE_PCA9685
  uint16_t ticks = SERVOMIN + (deg / 180.0f) * (SERVOMAX - SERVOMIN);
  pwm.setPWM(i, 0, ticks);
#else
  if (i < 4) servos[i].write((int)deg);   // servo 4 is unused in the 4-servo build
#endif
}

void setGrasp(int g) { grasp = g; state = GRASP; }
void releaseHand()   { state = RELEASE; }
void stopHand()      { state = IDLE; }       // freeze in place

void controlStep() {
  // 1) read and smooth force sensors
  for (uint8_t i = 0; i < N; i++) {
    if (!(SENSED & (1 << i))) { force[i] = 0; continue; }
    float raw = analogRead(FSR_PIN[i]);
    force[i] = 0.7f * force[i] + 0.3f * raw;
  }

  // 2) act on state
  if (state == RELEASE) {
    bool allOpen = true;
    for (uint8_t i = 0; i < N; i++) {
      closure[i] = max(0.0f, closure[i] - 0.03f);
      if (closure[i] > 0.0f) allOpen = false;
    }
    if (allOpen) state = IDLE;
  }
  else if (state == GRASP && grasp >= 0) {
    Grasp &g = GRASPS[grasp];
    for (uint8_t i = 0; i < N; i++) {
      if (!(g.mask & (1 << i))) {                 // finger not part of this grasp
        closure[i] = max(0.0f, closure[i] - 0.02f);
        continue;
      }
      if (!(SENSED & (1 << i))) {                 // unsensed finger follows the middle finger
        closure[i] = closure[2];
        continue;
      }
      float err = g.force - force[i];
      if (err > DEADBAND)        closure[i] += min(MAX_STEP, KP * err);
      else if (err < -DEADBAND)  closure[i] -= min(MAX_STEP, KP * (-err));
      closure[i] = constrain(closure[i], 0.0f, 1.0f);
    }

    // 3) slip detection every 100 ms
    if (millis() - lastSlipCheck >= 100) {
      lastSlipCheck = millis();
      for (uint8_t i = 0; i < N; i++) {
        if (!(g.mask & (1 << i)) || !(SENSED & (1 << i))) continue;
        bool wasInContact = prevForce[i] > 0.5f * g.force;
        if (wasInContact && (prevForce[i] - force[i]) > SLIP_DROP) {
          closure[i] = min(1.0f, closure[i] + SLIP_BOOST);
          slipCount++;
        }
        prevForce[i] = force[i];
      }
    }
  }

  // 4) drive servos
  for (uint8_t i = 0; i < N; i++) writeFinger(i);
}

// ---------------- Web dashboard ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DEXA</title>
<style>
body{font-family:sans-serif;margin:16px;max-width:520px}
button{font-size:18px;padding:14px;margin:4px;border-radius:8px;border:0;background:#1f6feb;color:#fff}
.stop{background:#d1242f;width:96%}
.bar{height:14px;background:#1f6feb;width:0;border-radius:4px}
.row{margin:6px 0}
</style></head><body>
<h2>DEXA Hand</h2>
<div>
<button onclick="c('p')">Power</button><button onclick="c('n')">Pinch</button>
<button onclick="c('t')">Tripod</button><button onclick="c('f')">Fragile</button>
<button onclick="c('o')">Open</button>
<button class="stop" onclick="c('s')">STOP</button>
</div>
<p id="st"></p>
<div id="bars"></div>
<script>
const names=['Thumb','Index','Middle','Ring','Pinky'];
document.getElementById('bars').innerHTML=names.map((n,i)=>
 '<div class="row">'+n+' <span id="v'+i+'"></span><div class="bar" id="b'+i+'"></div></div>').join('');
function c(x){fetch('/cmd?c='+x);}
async function poll(){
 try{
  const j=await (await fetch('/state')).json();
  document.getElementById('st').innerText='Grasp: '+j.g+' | State: '+j.s+' | Slips: '+j.slips;
  j.f.forEach((v,i)=>{
   document.getElementById('v'+i).innerText=Math.round(v);
   document.getElementById('b'+i).style.width=Math.min(100,v/4095*100*2)+'%';
  });
 }catch(e){}
 setTimeout(poll,300);
}
poll();
</script></body></html>
)rawliteral";

void handleCmd(char c) {
  switch (c) {
    case 'p': setGrasp(0); break;
    case 'n': setGrasp(1); break;
    case 't': setGrasp(2); break;
    case 'f': setGrasp(3); break;
    case 'o': releaseHand(); break;
    case 's': stopHand();    break;
  }
}

void setup() {
  Serial.begin(115200);
#if USE_PCA9685
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
#else
  for (uint8_t i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(SERVO_PIN[i], 500, 2400);
  }
#endif
  analogReadResolution(12);

  for (uint8_t i = 0; i < N; i++) writeFinger(i);   // start open

  WiFi.softAP("DEXA", "serbot123");
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/cmd", []() {
    if (server.hasArg("c")) handleCmd(server.arg("c")[0]);
    server.send(200, "text/plain", "ok");
  });
  server.on("/state", []() {
    String j = "{\"g\":\"";
    j += (grasp >= 0 ? GRASPS[grasp].name : "NONE");
    j += "\",\"s\":\"";
    j += (state == GRASP ? "GRASP" : state == RELEASE ? "RELEASE" : "IDLE");
    j += "\",\"slips\":" + String(slipCount) + ",\"f\":[";
    for (uint8_t i = 0; i < N; i++) { j += String(force[i], 0); if (i < N - 1) j += ","; }
    j += "]}";
    server.send(200, "application/json", j);
  });
  server.begin();
  Serial.println("DEXA ready. AP: DEXA / serbot123 -> http://192.168.4.1");
}

void loop() {
  server.handleClient();
  if (Serial.available()) handleCmd((char)Serial.read());

  if (millis() - lastCtl >= 20) {          // 50 Hz control loop
    lastCtl = millis();
    controlStep();

    // CSV log for your test report (open Serial Monitor / Serial Plotter)
    Serial.print(millis());
    for (uint8_t i = 0; i < N; i++) { Serial.print(','); Serial.print(force[i], 0); }
    Serial.println();
  }
}