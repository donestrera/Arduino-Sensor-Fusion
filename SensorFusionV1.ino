#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

// Pins for sensors and components
#define PIR_PIN 3                 // PIR motion sensor pin
#define SMOKE_SENSOR_PIN A1       // Smoke sensor pin
#define DHT_PIN 2                 // DHT sensor pin
#define LED_PIN 13                // LED pin (shared for smoke sensor and PIR)
#define BELL_PIN 12               // Fire alarm bell pin

// PIR motion sensor variables
const unsigned long PIR_ACTIVE_DURATION = 120000; // 2 minutes
bool pirMotionDetected = false;
unsigned long pirMotionTimer = 0;
bool pirMotionPrinted = false;
bool pirNoMotionPrinted = true;

// Smoke sensor variables
const int SMOKE_THRESHOLD = 225;
const int SMOKE_HYSTERESIS = 5;
#define SAMPLE_SIZE 10
int smokeSamples[SAMPLE_SIZE];
int smokeIndex = 0;
int smokeTotal = 0;
bool smokeDetected = false;

// DHT sensor variables
#define DHTTYPE DHT22
DHT_Unified dht(DHT_PIN, DHTTYPE);
uint32_t dhtDelayMS;

// Function prototypes
void addSmokeSample(int value);
int getSmokeAverage();

void setup() {
  // General setup
  Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);
  pinMode(SMOKE_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BELL_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH); // LED off (HIGH)
  digitalWrite(BELL_PIN, LOW); // Alarm off (LOW)

  // Initialize DHT sensor
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dhtDelayMS = sensor.min_delay / 1000;

  // Initialize smoke sensor samples
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    smokeSamples[i] = 0;
  }
}

void loop() {
  // --- PIR Motion Sensor ---
  int pirState = digitalRead(PIR_PIN);

  if (pirState == HIGH) {
    if (!pirMotionPrinted) {
      Serial.println("Motion is Detected");
      pirMotionPrinted = true;
      pirNoMotionPrinted = false;
    }
    pirMotionDetected = true;
    pirMotionTimer = millis();
    digitalWrite(LED_PIN, LOW); // LED on
  }

  if (pirMotionDetected && (millis() - pirMotionTimer > PIR_ACTIVE_DURATION)) {
    pirMotionDetected = false;
    digitalWrite(LED_PIN, HIGH); // LED off
    if (!pirNoMotionPrinted) {
      Serial.println("Motion Ended");
      pirNoMotionPrinted = true;
      pirMotionPrinted = false;
    }
  }

  // --- Smoke Sensor ---
  int rawSmokeData = analogRead(SMOKE_SENSOR_PIN);
  addSmokeSample(rawSmokeData);
  int smokeAverage = getSmokeAverage();

  if (smokeAverage >= SMOKE_THRESHOLD + SMOKE_HYSTERESIS && !smokeDetected) {
    digitalWrite(LED_PIN, LOW); // LED on
    digitalWrite(BELL_PIN, HIGH); // Activate alarm
    Serial.println("SMOKE_DETECTED");
    smokeDetected = true;
  } else if (smokeAverage <= SMOKE_THRESHOLD - SMOKE_HYSTERESIS && smokeDetected) {
    digitalWrite(LED_PIN, HIGH); // LED off
    digitalWrite(BELL_PIN, LOW); // Deactivate alarm
    Serial.println("NO_SMOKE");
    smokeDetected = false;
  }

  // --- DHT Sensor ---
  delay(dhtDelayMS);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    Serial.print("Temperature: ");
    Serial.print(event.temperature);
    Serial.println("°C");
  } else {
    Serial.println("Error reading temperature!");
  }

  dht.humidity().getEvent(&event);
  if (!isnan(event.relative_humidity)) {
    Serial.print("Humidity: ");
    Serial.print(event.relative_humidity);
    Serial.println("%");
  } else {
    Serial.println("Error reading humidity!");
  }

  delay(500); // Reduce loop frequency
}

void addSmokeSample(int value) {
  smokeTotal -= smokeSamples[smokeIndex];
  smokeSamples[smokeIndex] = value;
  smokeTotal += value;
  smokeIndex = (smokeIndex + 1) % SAMPLE_SIZE;
}

int getSmokeAverage() {
  return smokeTotal / SAMPLE_SIZE;
}