#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <TinyGPS++.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define PULSE_PIN 34

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
RTC_DS3231 rtc;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

int pulseThreshold = 550;
unsigned long lastBeatTime = 0;
int bpm = 0;
bool pulseDetected = false;

unsigned int stepCount = 0;
unsigned long lastStepTime = 0;
const float stepThreshold = 12.5;
const unsigned long stepDebounce = 300;

unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 250;

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Initializing System..."));
  display.display();

  if (!mpu.begin()) {
    Serial.println(F("Failed to find MPU6050 chip"));
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  pinMode(PULSE_PIN, INPUT);

  delay(1000);
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  readPulseSensor();
  detectSteps();

  if (millis() - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = millis();
    updateOLEDDisplay();
  }
}

void readPulseSensor() {
  int rawAnalog = analogRead(PULSE_PIN);
  unsigned long now = millis();

  if (rawAnalog > pulseThreshold && !pulseDetected) {
    pulseDetected = true;
    unsigned long ibi = now - lastBeatTime;
    lastBeatTime = now;

    if (ibi > 300 && ibi < 2000) {
      bpm = 60000 / ibi;
    }
  }

  if (rawAnalog < (pulseThreshold - 50)) {
    pulseDetected = false;
  }
}

void detectSteps() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelMagnitude = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );

  unsigned long now = millis();

  if (accelMagnitude > stepThreshold &&
      (now - lastStepTime > stepDebounce)) {
    stepCount++;
    lastStepTime = now;
  }
}

void updateOLEDDisplay() {
  display.clearDisplay();

  DateTime now = rtc.now();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  display.setCursor(68, 0);

  if (gps.altitude.isUpdated() || gps.altitude.isValid()) {
    display.printf("Alt:%04.0fm", gps.altitude.meters());
  } else {
    display.print(F("Alt: NoFix"));
  }

  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  display.setCursor(0, 16);
  display.setTextSize(2);

  if (bpm > 0) {
    display.printf("BPM: %d", bpm);
  } else {
    display.print(F("BPM: --"));
  }

  display.setCursor(0, 36);
  display.setTextSize(2);
  display.printf("Step: %u", stepCount);

  display.setTextSize(1);
  display.setCursor(0, 56);

  if (gps.location.isValid()) {
    display.printf("%.3f, %.3f", gps.location.lat(), gps.location.lng());
  } else {
    display.print(F("Searching Satellites..."));
  }

  display.display();
}