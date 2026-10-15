// ESP32 + TB6612FNG motor test (Motor A only, full speed, no PWM yet).
// Pins below are SUGGESTIONS. Match them to your wiring.
// Wiring: ESP32 GND <-> TB6612 GND (common ground!), TB6612 VCC = 3.3V,
// VM = motor supply (battery / LM2596 output), motor on A01/A02.

const int AIN1 = 26;
const int AIN2 = 27;
const int PWMA = 25;   // held HIGH for full speed in this first test
const int STBY = 33;   // must be HIGH or the driver stays asleep

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  digitalWrite(PWMA, HIGH);
}

void loop() {
  Serial.println("Forward");
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  delay(2000);

  Serial.println("Stop");
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  delay(1000);

  Serial.println("Reverse");
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  delay(2000);

  Serial.println("Stop");
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  delay(1000);
}
