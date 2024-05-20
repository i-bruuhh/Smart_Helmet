#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define I2C_SDA_PIN_1 18
#define I2C_SCL_PIN_1 19

#define I2C_SDA_PIN_2 26 
#define I2C_SCL_PIN_2 27 

const int touchPin1 = 5; // Connect first touch sensor to pin D5
const int touchPin2 = 35; // Connect second touch sensor to pin D35


#define LED_PIN_1 2  // Replace with the actual pin number where the first LED is connected
#define LED_PIN_2 4  // Replace with the actual pin number where the second LED is connected

#define BUZZER_PIN 21

Adafruit_ADXL345_Unified accel1 = Adafruit_ADXL345_Unified(0x1D);
Adafruit_ADXL345_Unified accel2 = Adafruit_ADXL345_Unified(0x53);

float oldX1 = 0, oldY1 = 0, oldZ1 = 0;
float oldX2 = 0, oldY2 = 0, oldZ2 = 0;
float deltaX1, deltaY1, deltaZ1;
float deltaX2, deltaY2, deltaZ2;

float crashSensitivity = 2.5;
float calibrationErrorX1 = 0, calibrationErrorZ1 = 0;
float calibrationErrorX2 = 0, calibrationErrorZ2 = 0;

unsigned long lastCrashTime = 0;
unsigned long crashCooldownTime = 3000;

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool buzzerOn = false;  // New variable to track the buzzer state
uint32_t value = 0;

#define SERVICE_UUID        "091d3cd1-95fc-4c15-a568-d599a72a4644"
#define CHAR1_UUID          "322c8568-d127-42b9-bea8-a4f2d94283aa"

struct AccelerationData {
  float averageX;
  float averageY;
  float averageZ;
  float crashDetected;
  float touch;  // Changed to int for swipe direction
};

AccelerationData accelerationData;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class CharacteristicCallBack : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string receivedValue = pChar->getValue();
    // Convert the received string to an integer
    int intValue = atoi(receivedValue.c_str());

    Serial.print("Received data as integer: ");
    Serial.println(intValue);

    // Calibration command
    if (intValue == 5) {
      // Set the calibration errors to zero
      calibrationErrorX1 = oldX1;
      calibrationErrorZ1 = oldZ1;
      calibrationErrorX2 = oldX2;
      calibrationErrorZ2 = oldZ2;
      Serial.println("Calibration completed. Resetting calibration errors.");
      // Set the last crash time
      lastCrashTime = millis();
    }

    // Stop the buzzer command
    else if (intValue == 20) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerOn = false;  // Update the buzzer state
    }

    // Process the received integer value as needed
  }
};

void displaySensorDetails(Adafruit_ADXL345_Unified &accel, int sensorNumber) {
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor "); Serial.print(sensorNumber); Serial.print(":       "); Serial.println(sensor.name);
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2"); 
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);

  Wire.begin(I2C_SDA_PIN_1, I2C_SCL_PIN_1);
  accel1.begin();
  accel1.setRange(ADXL345_RANGE_16_G);

  Wire.begin(I2C_SDA_PIN_2, I2C_SCL_PIN_2);
  accel2.begin();
  accel2.setRange(ADXL345_RANGE_16_G);

  pinMode(touchPin1, INPUT);
  pinMode(touchPin2, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  BLEDevice::init("SMART HELMET");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHAR1_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setCallbacks(new CharacteristicCallBack());

  BLEDescriptor* pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("Acceleration Data");
  pCharacteristic->addDescriptor(pDescr);

  BLE2902* pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");

  displaySensorDetails(accel1, 1);
  displaySensorDetails(accel2, 2);


  Serial.println("");
}

void loop() {

  sensors_event_t event1, event2;
  accel1.getEvent(&event1);
  accel2.getEvent(&event2);

  // Apply error correction for sensor 1
  float correctedX1 = event1.acceleration.x - calibrationErrorX1;
  float correctedZ1 = event1.acceleration.z - calibrationErrorZ1;

  // Apply error correction for sensor 2
  float correctedX2 = event2.acceleration.x - calibrationErrorX2;
  float correctedZ2 = event2.acceleration.z - calibrationErrorZ2;

  // Calculate average values using corrected values
  accelerationData.averageX = (correctedX1 + correctedX2) / 2.0;
  accelerationData.averageY = (event1.acceleration.y + event2.acceleration.y) / 2.0;
  accelerationData.averageZ = (correctedZ1 + correctedZ2) / 2.0;

  // Calculate change in values for sensor 1
  deltaX1 = abs(correctedX1 - oldX1);
  deltaY1 = abs(event1.acceleration.y - oldY1);
  deltaZ1 = abs(correctedZ1 - oldZ1);

  // Calculate change in values for sensor 2
  deltaX2 = abs(correctedX2 - oldX2);
  deltaY2 = abs(event2.acceleration.y - oldY2);
  deltaZ2 = abs(correctedZ2 - oldZ2);

  // Calculate magnitude from delta values for sensor 1
  float magnitude1 = sqrt(deltaX1 * deltaX1 + deltaY1 * deltaY1 + deltaZ1 * deltaZ1) / 9.8;

  // Calculate magnitude from delta values for sensor 2
  float magnitude2 = sqrt(deltaX2 * deltaX2 + deltaY2 * deltaY2 + deltaZ2 * deltaZ2) / 9.8;

  // Combine magnitudes (for crash detection)
  float deltaMagnitude = magnitude1 + magnitude2;

  // Check for crash based on combined magnitude and cooldown period
  accelerationData.crashDetected = 0;
  if (deltaMagnitude > crashSensitivity && (millis() - lastCrashTime > crashCooldownTime)) {
    Serial.println("Crash Detected!");
    Serial.print("Delta Magnitude: "); Serial.println(deltaMagnitude);

    // Turn on the buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;  // Update the buzzer state

    // Add any other actions you want to perform on crash detection here

    // Set the last crash time
    lastCrashTime = millis();

    accelerationData.crashDetected = 1;
    // Set the last crash time
    lastCrashTime = millis();
  }

  // Check if the buzzer is on and cooldown time has passed
  if (buzzerOn && (millis() - lastCrashTime > crashCooldownTime)) {
    // Turn off the buzzer
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;  // Update the buzzer state
  }

  // Update old values for sensor 1
  oldX1 = correctedX1;
  oldY1 = event1.acceleration.y;
  oldZ1 = correctedZ1;

  // Update old values for sensor 2
  oldX2 = correctedX2;
  oldY2 = event2.acceleration.y;
  oldZ2 = correctedZ2;

  int touch1 = digitalRead(touchPin1);
  int touch2 = digitalRead(touchPin2);

  if (touch1) {
    Serial.println("Touch sensor 1 Detected");
    accelerationData.touch = 1;
    // Control LEDs based on touch detection
    digitalWrite(LED_PIN_1, HIGH);  // Turn on the first LED
    delay(100);  // Adjust the delay as needed
    digitalWrite(LED_PIN_1, LOW);   // Turn off the first LED
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;  // Update the buzzer state
  } 
  
  else if (touch2) {
    Serial.println("Touch sensor 2 Detected");
    accelerationData.touch = 2;
    // Control LEDs based on touch detection
    digitalWrite(LED_PIN_2, HIGH);  // Turn on the first LED
    delay(100);  // Adjust the delay as needed
    digitalWrite(LED_PIN_2, LOW);   // Turn off the first LED
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;  // Update the buzzer state
  } 
  else {
    // Set the detected field to 1
    accelerationData.touch = 0;
  }

  if (deviceConnected) {
    // Send corrected average values via BLE
    pCharacteristic->setValue((uint8_t*)&accelerationData, sizeof(accelerationData));
    pCharacteristic->notify();
    delay(100);
  }

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

