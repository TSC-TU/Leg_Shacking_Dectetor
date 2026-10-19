#include <Wire.h>

const int MPU = 0x68;

// ---- Tunable params ----
const float SAMPLE_HZ  = 100.0;
const float HP_ALPHA   = 0.92;
const float TRIG_AMP   = 0.25;
const float FREQ_LOW   = 3.0;
const float FREQ_HIGH  = 12.0;
const int   WIN_MS     = 1000;
const int   MIN_CROSS  = 6;

// ---- State ----
float hpPrev = 0, rawPrev = 0;
int   crossCount = 0;
float lastSign = 0;
unsigned long winStart = 0;
unsigned long lastSample = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(8, 9);             // SDA=GPIO8, SCL=GPIO9

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x08);             // accel ±4g
  Wire.endTransmission(true);

  winStart = millis();
  Serial.println("Ready");
}

void readAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  ax = (rawX / 8192.0) * 9.81;
  ay = (rawY / 8192.0) * 9.81;
  az = (rawZ / 8192.0) * 9.81;
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample < (1000.0 / SAMPLE_HZ)) return;
  lastSample = now;

  float ax, ay, az;
  readAccel(ax, ay, az);

  float mag = sqrt(ax*ax + ay*ay + az*az);

  float hp = HP_ALPHA * (hpPrev + mag - rawPrev);
  rawPrev = mag;
  hpPrev  = hp;

  float sign = (hp > TRIG_AMP) ? 1 : (hp < -TRIG_AMP) ? -1 : 0;
  if (sign != 0 && sign != lastSign && lastSign != 0) {
    crossCount++;
  }
  if (sign != 0) lastSign = sign;

  if (now - winStart >= WIN_MS) {
    float freq = (crossCount / 2.0) * (1000.0 / WIN_MS);
    bool shaking = (crossCount >= MIN_CROSS) &&
                   (freq >= FREQ_LOW) && (freq <= FREQ_HIGH);

    Serial.print("crossings="); Serial.print(crossCount);
    Serial.print("  freq="); Serial.print(freq, 1);
    Serial.print("Hz  "); Serial.println(shaking ? "SHAKING!" : "ok");

    crossCount = 0;
    lastSign = 0;
    winStart = now;
  }
}