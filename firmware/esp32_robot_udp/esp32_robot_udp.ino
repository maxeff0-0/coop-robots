// ESP32 robot driver: receives 1-letter UDP commands from the laptop and drives TB6612.
//   F = forward, B = backward, L = spin left, R = spin right, S = stop
// Failsafe: if no command arrives for 500 ms, the robot stops.

#include <WiFi.h>
#include <WiFiUdp.h>

// ---- EDIT THESE ----
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
// --------------------
const int UDP_PORT = 4210;
const int SPEED = 150;                    // 0-255. Start low, raise after testing.
const unsigned long TIMEOUT_MS = 500;

// TB6612 pins (match your wiring)
const int AIN1 = 26, AIN2 = 27, PWMA = 25;   // left side motors
const int BIN1 = 14, BIN2 = 32, PWMB = 13;   // right side motors
const int STBY = 33;

WiFiUDP udp;
unsigned long lastCmd = 0;

void pwmInit(int pin, int ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, 1000, 8);
#else
  ledcSetup(ch, 1000, 8);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(int pin, int ch, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void side(int in1, int in2, int pwmPin, int ch, int dir) {
  digitalWrite(in1, dir > 0);
  digitalWrite(in2, dir < 0);
  pwmWrite(pwmPin, ch, dir == 0 ? 0 : SPEED);
}

void drive(int left, int right) {
  side(AIN1, AIN2, PWMA, 0, left);
  side(BIN1, BIN2, PWMB, 1, right);
}

void apply(char c) {
  switch (c) {
    case 'F': drive(1, 1);   break;
    case 'B': drive(-1, -1); break;
    case 'L': drive(-1, 1);  break;
    case 'R': drive(1, -1);  break;
    default:  drive(0, 0);   break;   // 'S' or anything unknown
  }
}

void setup() {
  Serial.begin(115200);
  int outs[] = {AIN1, AIN2, BIN1, BIN2, STBY};
  for (int p : outs) pinMode(p, OUTPUT);
  pwmInit(PWMA, 0);
  pwmInit(PWMB, 1);
  digitalWrite(STBY, HIGH);
  drive(0, 0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  WiFi.setSleep(false);
  udp.begin(UDP_PORT);
  Serial.println();
  Serial.print("Robot ready. IP: ");
  Serial.print(WiFi.localIP());
  Serial.print("  UDP port: ");
  Serial.println(UDP_PORT);
}

void loop() {
  int n = udp.parsePacket();
  if (n > 0) {
    char buf[16];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      lastCmd = millis();
      apply(buf[0]);
    }
  }
  if (millis() - lastCmd > TIMEOUT_MS) drive(0, 0);   // failsafe
}
